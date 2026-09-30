package eu.leongorecki.deskmouse.recorder

import android.graphics.Bitmap
import android.hardware.SensorManager
import android.hardware.camera2.CameraManager
import android.os.SystemClock
import android.util.Log
import android.view.KeyEvent
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import java.io.File
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

private const val TAG = "DeskRec"

enum class Phase { Idle, Countdown, Metering, Recording, Saving, WaitingForPc, Streaming }

/** Owns camera + IMU for the activity's lifetime; drives live monitoring and recordings. */
class RecorderController(
    private val cm: CameraManager,
    private val sm: SensorManager,
    private val root: File,
    private val appVersion: String,
) : CameraCapture.FrameListener {
    val cam = CameraCapture(cm).also { it.listener = this }
    val imu = ImuLogger(sm)
    private val beeper = Beeper()
    private val worker = Executors.newSingleThreadExecutor()
    private val analyser = Executors.newSingleThreadExecutor()
    private val analysing = AtomicBoolean(false)

    val choices = listCamChoices(cm)

    // UI state
    var protocol by mutableStateOf(PROTOCOLS.first())
    var seconds by mutableStateOf(PROTOCOLS.first().seconds)
    var location by mutableStateOf("")
    var choice by mutableStateOf(choices.firstOrNull())
    var size by mutableStateOf(640 to 480)
    var fps by mutableStateOf(60)
    var exposureMs by mutableStateOf(2f)
    var raw by mutableStateOf(false)
    var phase by mutableStateOf(Phase.Idle)
    var remaining by mutableStateOf(0)
    var status by mutableStateOf("")
    var live by mutableStateOf("")
    var preview by mutableStateOf<Bitmap?>(null)
    var lastSummary by mutableStateOf("")

    @Volatile private var rec: Recording? = null
    @Volatile private var streamer: StreamServer? = null
    @Volatile private var stopRequested = false
    private var liveFrames = 0
    private var liveT0 = 0L
    private var liveFps = 0.0
    private var corners = 0
    private var satPct = 0f
    private var upDown = false
    private var downDown = false

    fun config() = CaptureConfig(
        choice = choice!!, width = size.first, height = size.second, fps = fps,
        exposureNs = (exposureMs * 1e6).toLong(), maxExposureNs = maxOf((exposureMs * 1e6).toLong(), 4_000_000L),
    )

    /** (Re)open the camera for live monitoring with the current settings. */
    fun startLive() = worker.execute {
        try {
            status = "opening camera…"
            cam.open(config())
            imu.stop(); imu.start(null)
            liveFrames = 0; liveT0 = 0
            status = ""
        } catch (e: Exception) {
            Log.e(TAG, "open failed", e); status = "camera error: ${e.message}"
        }
    }

    fun stopLive() {
        stopRequested = true // ends a running take; the worker then closes the camera
        worker.execute { cam.close(); imu.stop() }
    }

    fun release() {
        worker.execute { cam.quit(); imu.stop(); imu.quit(); beeper.release() }
        worker.shutdown(); analyser.shutdown()
    }

    fun record() {
        if (phase != Phase.Idle || choice == null) return
        stopRequested = false
        worker.execute { runRecording() }
    }

    private fun runRecording() {
        val cfg = config()
        try {
            if (!cam.isOpenWith(cfg)) cam.open(cfg)
            phase = Phase.Countdown
            for (i in 3 downTo 1) { remaining = i; beeper.tick(); Thread.sleep(1000) }
            phase = Phase.Metering
            cam.unlockExposure()
            cam.resetMetering()
            Thread.sleep(1500)
            cam.lockExposure()
            if (!cam.awaitExposureApplied()) Log.w(TAG, "manual exposure not confirmed in results")

            val r = Recording(root, protocol, location, cfg, raw)
            cam.frameMeta.clear()
            cam.collectMeta = true
            imu.stop()
            imu.start(File(r.dir, "imu.csv"))
            r.startNs = SystemClock.elapsedRealtimeNanos()
            rec = r
            phase = Phase.Recording
            beeper.start()
            val endAt = SystemClock.elapsedRealtime() + seconds * 1000L
            while (!stopRequested && SystemClock.elapsedRealtime() < endAt) {
                remaining = ((endAt - SystemClock.elapsedRealtime()) / 1000).toInt() + 1
                Thread.sleep(100)
            }
            finishRecording(r)
        } catch (e: Exception) {
            Log.e(TAG, "recording failed", e)
            beeper.error()
            status = "recording error: ${e.message}"
            rec = null
            phase = Phase.Idle
        }
    }

    /** MVP: stream camera + IMU to the PC until it disconnects or both volume keys are pressed. */
    fun stream() {
        if (phase != Phase.Idle || choice == null) return
        stopRequested = false
        worker.execute { runStream() }
    }

    private fun runStream() {
        val cfg = config()
        var s: StreamServer? = null
        try {
            if (!cam.isOpenWith(cfg)) cam.open(cfg)
            s = StreamServer()
            phase = Phase.WaitingForPc
            status = "waiting for PC on port ${s.port} (USB: adb forward; Wi-Fi: ${wifiIp() ?: "no Wi-Fi"})"
            if (!s.awaitClient { !stopRequested }) { s.close(); phase = Phase.Idle; status = ""; return }
            phase = Phase.Countdown
            for (i in 3 downTo 1) { remaining = i; beeper.tick(); Thread.sleep(1000) }
            phase = Phase.Metering
            cam.unlockExposure()
            cam.resetMetering()
            Thread.sleep(1500)
            cam.lockExposure()
            cam.awaitExposureApplied()
            val hello = org.json.JSONObject()
                .put("width", cfg.width).put("height", cfg.height).put("fps", cfg.fps).put("jpeg", s.jpeg)
                .put("exposure_ns", cam.appliedExposureNs).put("iso", cam.appliedIso)
                .put("camera_characteristics", DeviceCaps.camera(cm, cfg.choice.openId))
                .put("device", DeviceCaps.device())
            s.hello(hello.toString())
            imu.onSample = { t, k, x, y, z -> s.imu(t, k, x, y, z) }
            streamer = s
            phase = Phase.Streaming
            beeper.start()
            val t0 = SystemClock.elapsedRealtime()
            while (!stopRequested && s.connected) {
                remaining = ((SystemClock.elapsedRealtime() - t0) / 1000).toInt()
                status = "streaming: ${s.sentFrames.get()} frames sent, ${s.droppedFrames.get()} dropped"
                Thread.sleep(200)
            }
        } catch (e: Exception) {
            Log.e(TAG, "stream failed", e); beeper.error(); status = "stream error: ${e.message}"
        } finally {
            streamer = null
            imu.onSample = null
            s?.close()
            beeper.stop()
            cam.unlockExposure()
            lastSummary = "stream ended: ${s?.sentFrames?.get() ?: 0} frames sent, ${s?.droppedFrames?.get() ?: 0} dropped"
            phase = Phase.Idle
        }
    }

    /** The phone's IPv4 address on Wi-Fi, shown so the user can pass it to mvp_live.py --host. */
    private fun wifiIp(): String? = try {
        java.net.NetworkInterface.getNetworkInterfaces().toList()
            .filter { it.isUp && it.name.startsWith("wlan") }
            .flatMap { it.inetAddresses.toList() }
            .firstOrNull { it is java.net.Inet4Address }?.hostAddress
    } catch (_: Exception) { null }

    private fun finishRecording(r: Recording) {
        rec = null
        r.endNs = SystemClock.elapsedRealtimeNanos()
        // Barrier: let any frame already inside onFrame finish before closing the sink.
        val latch = CountDownLatch(1); cam.handler.post { latch.countDown() }; latch.await()
        beeper.stop()
        phase = Phase.Saving
        imu.stop()
        cam.collectMeta = false
        val notes = mutableListOf<String>()
        if (cam.exposureMode != "manual") notes += "exposure not manual: ${cam.exposureMode}"
        r.finish(cam, cm, sm, imu, appVersion, notes)
        val fpsMeasured = if (r.frameTs.size > 1) (r.frameTs.size - 1) * 1e9 / (r.frameTs.last() - r.frameTs.first()) else 0.0
        lastSummary = "%s\n%d frames @ %.1f fps, %d dropped · gyro %.0f Hz · accel %.0f Hz · exp %.2f ms ISO %d".format(
            r.dir.name, r.frameTs.size, fpsMeasured, r.droppedFrames,
            imu.rateHz(ImuLogger.Kind.GYRO), imu.rateHz(ImuLogger.Kind.ACCEL), cam.appliedExposureNs / 1e6, cam.appliedIso)
        Log.i(TAG, "saved ${r.dir}: $lastSummary")
        cam.unlockExposure()
        imu.start(null)
        liveFrames = 0; liveT0 = 0
        phase = Phase.Idle
    }

    fun saveDeviceCaps(): File {
        val f = File(root, "device_caps.json")
        f.writeText(DeviceCaps.all(cm, sm).toString(1))
        return f
    }

    /** Volume keys: labels while recording, both together = stop. Returns true if consumed. */
    fun onKey(keyCode: Int, down: Boolean, repeat: Boolean): Boolean {
        if (keyCode != KeyEvent.KEYCODE_VOLUME_UP && keyCode != KeyEvent.KEYCODE_VOLUME_DOWN) return false
        if (phase == Phase.Idle) return false
        if (phase == Phase.Streaming || phase == Phase.WaitingForPc) {
            if (repeat) return true
            if (keyCode == KeyEvent.KEYCODE_VOLUME_UP) upDown = down else downDown = down
            if (upDown && downDown) stopRequested = true
            return true
        }
        if (repeat) return true
        val up = keyCode == KeyEvent.KEYCODE_VOLUME_UP
        if (up) upDown = down else downDown = down
        rec?.label((if (up) "vol_up" else "vol_down") + (if (down) "_down" else "_up"))
        if (upDown && downDown) stopRequested = true
        return true
    }

    override fun onFrame(y: ByteArray, tNs: Long) {
        val r = rec
        if (r != null) {
            if (r.sink.write(y, tNs)) r.frameTs.add(tNs) else r.droppedFrames++
        }
        streamer?.let { st ->
            val skew = cam.lastResult?.get(android.hardware.camera2.CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW) ?: 0L
            st.frame(y, cam.config.width, cam.config.height, tNs, cam.appliedExposureNs.toInt(), skew.toInt())
        }
        liveFrames++
        val now = SystemClock.elapsedRealtimeNanos()
        if (liveT0 == 0L) { liveT0 = now; liveFrames = 0 }
        else if (now - liveT0 > 1_000_000_000L) {
            liveFps = liveFrames * 1e9 / (now - liveT0); liveT0 = now; liveFrames = 0
            live = "camera %.1f fps · gyro %.0f Hz · accel %.0f Hz · features %d · saturated %.1f%%".format(
                liveFps, imu.rateHz(ImuLogger.Kind.GYRO), imu.rateHz(ImuLogger.Kind.ACCEL), corners, satPct)
        }
        if (liveFrames % 6 == 0 && analysing.compareAndSet(false, true)) {
            val copy = y.copyOf()
            val w = cam.config.width
            val h = cam.config.height
            analyser.execute {
                try {
                    val res = FeatureCounter.analyse(copy, w, h)
                    corners = res.corners; satPct = res.saturatedFrac * 100
                    if (rec == null) preview = toBitmap(res.small, res.w, res.h)
                } finally { analysing.set(false) }
            }
        }
    }

    private fun toBitmap(g: ByteArray, w: Int, h: Int): Bitmap {
        val px = IntArray(w * h) { val v = g[it].toInt() and 0xFF; (0xFF shl 24) or (v shl 16) or (v shl 8) or v }
        return Bitmap.createBitmap(px, w, h, Bitmap.Config.ARGB_8888)
    }
}
