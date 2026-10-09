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

// Calibration run at the start of a Bluetooth mouse session (new ceiling): at least CAL_MIN_S and CAL_TARGET_STROKES
// strokes, at most CAL_MAX_S; fewer than CAL_MIN_STROKES means the scale is not trustworthy.
private const val CAL_MIN_S = 10.0
private const val CAL_MAX_S = 20.0
private const val CAL_TARGET_STROKES = 6
private const val CAL_MIN_STROKES = 3

enum class Phase { Idle, Countdown, Metering, Recording, Saving, WaitingForPc, Streaming, BtMouse }

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
    var btStatus by mutableStateOf("Bluetooth mouse off")
    var calibrateFirst by mutableStateOf(true)
    var calibrating by mutableStateOf(false)

    /** Set by the activity once Bluetooth permissions are granted. */
    var bt: BtMouse? = null
        set(v) {
            field = v
            v?.listener = object : BtMouse.Listener { override fun onBtState(text: String) { btStatus = text } }
        }

    /** Bluetooth check without tracking: the cursor draws two circles (T1.1). */
    fun btTestCircle() {
        val b = bt ?: return
        if (!b.connected) { btStatus = "not connected to a computer"; return }
        Thread {
            val r = 150.0
            val steps = 240 // 2 circles at 120 Hz = 2 s
            var px = r; var py = 0.0
            for (i in 1..steps) {
                val a = 2 * Math.PI * 2 * i / steps
                val x = r * Math.cos(a); val y = r * Math.sin(a)
                b.move(Math.round(x - px).toInt(), Math.round(y - py).toInt())
                px += Math.round(x - px); py += Math.round(y - py)
                Thread.sleep(8)
            }
            b.click(1)
            btStatus = "test done: two circles + one left click"
        }.start()
    }

    @Volatile private var rec: Recording? = null
    @Volatile private var streamer: StreamServer? = null
    @Volatile private var mouse: MouseSession? = null
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
                .put("facing", if (cfg.choice.front) "front" else "back")
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

    /** Bluetooth mouse: tracking runs on the phone (C++ core), reports go straight to the paired computer. */
    fun btMouse() {
        if (phase != Phase.Idle || choice == null) return
        val b = bt
        if (b == null || !b.connected) { btStatus = "connect a computer first (Bluetooth mouse section)"; return }
        stopRequested = false
        worker.execute { runBtMouse(b) }
    }

    private fun runBtMouse(b: BtMouse) {
        val cfg = config()
        var m: MouseSession? = null
        val dir = root.parentFile ?: root
        lastSummary = ""
        try {
            if (!cam.isOpenWith(cfg)) cam.open(cfg)
            phase = Phase.Countdown
            for (i in 3 downTo 1) { remaining = i; beeper.tick(); Thread.sleep(1000) }
            phase = Phase.Metering
            cam.unlockExposure()
            cam.resetMetering()
            Thread.sleep(1500)
            cam.lockExposure()
            cam.awaitExposureApplied()
            val (K, dist) = NativeEngine.intrinsics(cam.chars, cfg.width)
            val (params, ratio) = MouseSession.loadParams(dir)
            Log.i(TAG, "bt mouse: ${params.size} tuned params, gauge ratio $ratio, front=${cfg.choice.front}")
            m = MouseSession(NativeEngine(K, dist, cfg.width, cfg.height, params, ratio), b, cfg.width, cfg.height)
            imu.onSample = { t, k, x, y, z -> m.onImu(t, k, x, y, z) }
            mouse = m
            phase = Phase.BtMouse
            beeper.start()
            if (calibrateFirst) calibrate(m, b)
            val t0 = SystemClock.elapsedRealtime()
            while (!stopRequested && b.connected) {
                remaining = ((SystemClock.elapsedRealtime() - t0) / 1000).toInt()
                val s = m.state
                val mode = when { s[1] == 1.0 -> "lifted"; s[2] == 1.0 -> "scroll"; s[0] == 1.0 -> "still"; else -> "moving" }
                status = (if (cfg.choice.front) "FRONT camera: engine assumes face down, directions wrong. " else "") + "BT mouse: %d frames, %d skipped, engine %.1f ms (max %.1f), %s, scale %s, calibrations %.0f, taps %.0f, inliers %.0f".format(
                    m.processed, m.skipped, m.msMean, m.msMax, mode, if (s[3].isNaN()) "-" else "%.4f".format(s[3]), s[6], s[5], s[8])
                Thread.sleep(200)
            }
            if (!b.connected) status = "Bluetooth disconnected"
        } catch (e: Exception) {
            Log.e(TAG, "bt mouse failed", e); beeper.error(); status = "BT mouse error: ${e.message}"
        } finally {
            imu.onSample = null
            mouse = null
            m?.let { MouseSession.saveRatio(dir, it.state[9]); it.stop() }
            beeper.stop()
            cam.unlockExposure()
            calibrating = false
            lastSummary = (if (lastSummary.startsWith("calibrat")) "$lastSummary; " else "") +
                "BT mouse ended: ${m?.processed ?: 0} frames, ${m?.skipped ?: 0} skipped, engine %.1f ms/frame, knocks %.0f".format(m?.msMean ?: 0.0, m?.state?.get(12) ?: 0.0)
            Log.i(TAG, "$lastSummary; state ${m?.state?.joinToString()}")
            phase = Phase.Idle
        }
    }

    /**
     * Calibration for the current ceiling: the user slides the phone in separate 10–20 cm strokes with short pauses;
     * each stroke compares camera and accelerometer distance, and the scale becomes their median. The phone lies face
     * down, so progress is audible: a tick per measured stroke, the start tone when done, the error tone if too few.
     */
    private fun calibrate(m: MouseSession, b: BtMouse) {
        calibrating = true
        m.beginCalibration()
        val t0 = SystemClock.elapsedRealtime()
        var seen = 0
        while (!stopRequested && b.connected) {
            val s = m.state
            val n = if (s[10] == 1.0 && !s[11].isNaN()) s[11].toInt() else 0
            if (n > seen) { beeper.tick(); seen = n }
            val el = (SystemClock.elapsedRealtime() - t0) / 1000.0
            remaining = el.toInt()
            status = "Calibrating: slide the phone 10–20 cm, pause, repeat · $n strokes"
            if ((el >= CAL_MIN_S && n >= CAL_TARGET_STROKES) || el >= CAL_MAX_S) break
            Thread.sleep(100)
        }
        val done = CountDownLatch(1)
        var n = 0
        m.endCalibration { n = it; done.countDown() }
        done.await(1, java.util.concurrent.TimeUnit.SECONDS)
        calibrating = false
        val ok = n >= CAL_MIN_STROKES
        if (ok) beeper.start() else beeper.error()
        lastSummary = if (ok) "calibrated on $n strokes" else "calibration: only $n strokes, speed may be off (use separate strokes with pauses)"
        Log.i(TAG, "calibration: $n strokes, scale ${m.state[3]}")
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
        if (phase == Phase.Streaming || phase == Phase.WaitingForPc || phase == Phase.BtMouse) {
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
        mouse?.let { ms ->
            val res = cam.lastResult
            val skew = res?.get(android.hardware.camera2.CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW) ?: 0L
            ms.onFrame(y, tNs, cam.appliedExposureNs, skew)
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
