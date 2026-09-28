package eu.leongorecki.deskmouse.recorder

import android.annotation.SuppressLint
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.TotalCaptureResult
import android.hardware.camera2.params.OutputConfiguration
import android.hardware.camera2.params.SessionConfiguration
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Range
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executor
import java.util.concurrent.TimeUnit

private const val TAG = "DeskCam"

/** A way to get a rear camera stream: open [openId], optionally at a zoom ratio < 1 (logical → ultra-wide). */
data class CamChoice(val label: String, val openId: String, val zoom: Float)

data class CaptureConfig(
    val choice: CamChoice,
    val width: Int = 640,
    val height: Int = 480,
    val fps: Int = 60,
    val exposureNs: Long = 2_000_000,
    val maxExposureNs: Long = 4_000_000,
    val focusDistanceM: Float = 1.9f,
)

/** Per-frame values from CaptureResult, joined to frames by SENSOR_TIMESTAMP. */
data class FrameMeta(
    val exposureNs: Long, val skewNs: Long, val iso: Int, val frameDurationNs: Long,
    val focusDiopters: Float, val activePhysicalId: String,
)

fun listCamChoices(cm: CameraManager): List<CamChoice> {
    val out = mutableListOf<CamChoice>()
    for (id in cm.cameraIdList) {
        val ch = cm.getCameraCharacteristics(id)
        if (ch.get(CameraCharacteristics.LENS_FACING) != CameraCharacteristics.LENS_FACING_BACK) continue
        val focal = ch.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull() ?: 0f
        out += CamChoice("id $id · %.1f mm · 1×".format(focal), id, 1f)
        val zr = ch.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE)
        if (zr != null && zr.lower < 0.99f) out += CamChoice("id $id · %.1f× (ultra-wide)".format(zr.lower), id, zr.lower)
    }
    return out
}

class CameraCapture(private val cm: CameraManager) {
    interface FrameListener {
        /** Called on the camera thread with the image's Y plane packed to width*height. */
        fun onFrame(y: ByteArray, tNs: Long)
    }

    private val thread = HandlerThread("camera").apply { start() }
    val handler = Handler(thread.looper)
    private val executor = Executor { handler.post(it) }

    private var device: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var reader: ImageReader? = null
    private var builder: CaptureRequest.Builder? = null
    lateinit var config: CaptureConfig
        private set
    lateinit var chars: CameraCharacteristics
        private set

    @Volatile var lastResult: TotalCaptureResult? = null
    /** Brightest auto-exposure result (smallest exposure·ISO) since [resetMetering]; a hand over the lens only darkens. */
    @Volatile private var brightest: Pair<Long, Int>? = null
    fun resetMetering() { brightest = null }
    /** Filled only while [collectMeta] is true. */
    val frameMeta = ConcurrentHashMap<Long, FrameMeta>()
    @Volatile var collectMeta = false
    var listener: FrameListener? = null
    private var yBuf = ByteArray(0)

    /** What the metering pass decided, for meta.json. */
    var exposureMode = "auto"
        private set
    var appliedExposureNs = 0L
        private set
    var appliedIso = 0
        private set
    var meteredExposureNs = 0L
        private set
    var meteredIso = 0
        private set

    fun isOpenWith(cfg: CaptureConfig) = device != null && ::config.isInitialized && config == cfg

    val manualSensor: Boolean
        get() = chars.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES)
            ?.contains(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR) == true

    /** Opens the camera and starts streaming with auto exposure. Blocks until streaming. */
    @SuppressLint("MissingPermission")
    fun open(cfg: CaptureConfig) {
        close()
        config = cfg
        chars = cm.getCameraCharacteristics(cfg.choice.openId)
        yBuf = ByteArray(cfg.width * cfg.height)

        val opened = CountDownLatch(1)
        var err: String? = null
        cm.openCamera(cfg.choice.openId, executor, object : CameraDevice.StateCallback() {
            override fun onOpened(d: CameraDevice) { device = d; opened.countDown() }
            override fun onDisconnected(d: CameraDevice) { d.close(); device = null; err = "disconnected"; opened.countDown() }
            override fun onError(d: CameraDevice, e: Int) { d.close(); device = null; err = "error $e"; opened.countDown() }
        })
        check(opened.await(5, TimeUnit.SECONDS) && device != null) { "camera open failed: $err" }

        val r = ImageReader.newInstance(cfg.width, cfg.height, ImageFormat.YUV_420_888, 6)
        r.setOnImageAvailableListener({ onImage(it) }, handler)
        reader = r

        val configured = CountDownLatch(1)
        val sc = SessionConfiguration(
            SessionConfiguration.SESSION_REGULAR, listOf(OutputConfiguration(r.surface)), executor,
            object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(s: CameraCaptureSession) { session = s; configured.countDown() }
                override fun onConfigureFailed(s: CameraCaptureSession) { configured.countDown() }
            })
        device!!.createCaptureSession(sc)
        check(configured.await(5, TimeUnit.SECONDS) && session != null) { "session configure failed" }

        builder = device!!.createCaptureRequest(CameraDevice.TEMPLATE_RECORD).apply {
            addTarget(r.surface)
            applyCommon(this, cfg)
            set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON)
        }
        repeat()
    }

    private fun applyCommon(b: CaptureRequest.Builder, cfg: CaptureConfig) {
        b.set(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO)
        b.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, Range(cfg.fps, cfg.fps))
        b.set(CaptureRequest.CONTROL_ZOOM_RATIO, cfg.choice.zoom)
        b.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF)
        b.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
        b.set(CaptureRequest.EDGE_MODE, CaptureRequest.EDGE_MODE_OFF)
        val afModes = chars.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES) ?: intArrayOf()
        val minFocus = chars.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE) ?: 0f
        if (afModes.contains(CaptureRequest.CONTROL_AF_MODE_OFF)) {
            b.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF)
            if (minFocus > 0f) b.set(CaptureRequest.LENS_FOCUS_DISTANCE, 1f / cfg.focusDistanceM)
        }
    }

    private fun repeat() {
        session!!.setRepeatingRequest(builder!!.build(), object : CameraCaptureSession.CaptureCallback() {
            override fun onCaptureCompleted(s: CameraCaptureSession, req: CaptureRequest, res: TotalCaptureResult) {
                lastResult = res
                if (exposureMode == "auto") {
                    val e = res.get(CaptureResult.SENSOR_EXPOSURE_TIME)
                    val iso = res.get(CaptureResult.SENSOR_SENSITIVITY)
                    val b = brightest
                    if (e != null && iso != null && (b == null || e.toDouble() * iso < b.first.toDouble() * b.second)) brightest = e to iso
                }
                if (!collectMeta) return
                val t = res.get(CaptureResult.SENSOR_TIMESTAMP) ?: return
                frameMeta[t] = FrameMeta(
                    res.get(CaptureResult.SENSOR_EXPOSURE_TIME) ?: -1,
                    res.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW) ?: -1,
                    res.get(CaptureResult.SENSOR_SENSITIVITY) ?: -1,
                    res.get(CaptureResult.SENSOR_FRAME_DURATION) ?: -1,
                    res.get(CaptureResult.LENS_FOCUS_DISTANCE) ?: -1f,
                    res.get(CaptureResult.LOGICAL_MULTI_CAMERA_ACTIVE_PHYSICAL_ID) ?: "",
                )
            }
        }, handler)
    }

    /**
     * Locks exposure from the auto-exposure result: target exposure time, ISO scaled to keep
     * brightness; if ISO hits its max, exposure grows up to [CaptureConfig.maxExposureNs].
     * Falls back to AE lock when manual sensor control isn't available.
     */
    fun lockExposure() {
        val res = lastResult ?: error("no capture result yet")
        val b = builder!!
        val bright = brightest
        meteredExposureNs = bright?.first ?: res.get(CaptureResult.SENSOR_EXPOSURE_TIME) ?: config.exposureNs
        meteredIso = bright?.second ?: res.get(CaptureResult.SENSOR_SENSITIVITY) ?: 100
        if (!manualSensor) {
            b.set(CaptureRequest.CONTROL_AE_LOCK, true)
            exposureMode = "ae_lock"
            appliedExposureNs = meteredExposureNs; appliedIso = meteredIso
        } else {
            val isoRange = chars.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE)!!
            val expRange = chars.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE)!!
            val light = meteredExposureNs.toDouble() * meteredIso // exposure·gain to keep
            var exp = config.exposureNs
            var iso = (light / exp).toInt()
            if (iso > isoRange.upper) {
                iso = isoRange.upper
                exp = (light / iso).toLong().coerceAtMost(config.maxExposureNs)
            }
            iso = iso.coerceIn(isoRange.lower, isoRange.upper)
            exp = exp.coerceIn(expRange.lower, expRange.upper)
            b.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF)
            b.set(CaptureRequest.SENSOR_EXPOSURE_TIME, exp)
            b.set(CaptureRequest.SENSOR_SENSITIVITY, iso)
            b.set(CaptureRequest.SENSOR_FRAME_DURATION, 1_000_000_000L / config.fps)
            exposureMode = "manual"
            appliedExposureNs = exp; appliedIso = iso
        }
        repeat()
        Log.i(TAG, "exposure $exposureMode: metered ${meteredExposureNs}ns ISO $meteredIso -> ${appliedExposureNs}ns ISO $appliedIso")
    }

    /** Back to auto exposure (after a recording, for live monitoring). */
    fun unlockExposure() {
        val b = builder ?: return
        b.set(CaptureRequest.CONTROL_AE_LOCK, false)
        b.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON)
        exposureMode = "auto"
        repeat()
    }

    /** Waits until capture results reflect the requested manual exposure (or timeout). */
    fun awaitExposureApplied(timeoutMs: Long = 1000): Boolean {
        val deadline = System.currentTimeMillis() + timeoutMs
        while (System.currentTimeMillis() < deadline) {
            val e = lastResult?.get(CaptureResult.SENSOR_EXPOSURE_TIME)
            if (exposureMode != "manual" || e == appliedExposureNs) return true
            Thread.sleep(20)
        }
        return false
    }

    /** Snapshot of what the camera is actually doing, from the last result (for meta.json). */
    fun appliedState(): Map<String, Any?> {
        val r = lastResult ?: return emptyMap()
        return mapOf(
            "ae_mode" to r.get(CaptureResult.CONTROL_AE_MODE),
            "af_mode" to r.get(CaptureResult.CONTROL_AF_MODE),
            "ois_mode" to r.get(CaptureResult.LENS_OPTICAL_STABILIZATION_MODE),
            "video_stabilization_mode" to r.get(CaptureResult.CONTROL_VIDEO_STABILIZATION_MODE),
            "edge_mode" to r.get(CaptureResult.EDGE_MODE),
            "noise_reduction_mode" to r.get(CaptureResult.NOISE_REDUCTION_MODE),
            "zoom_ratio" to r.get(CaptureResult.CONTROL_ZOOM_RATIO),
            "focus_distance_diopters" to r.get(CaptureResult.LENS_FOCUS_DISTANCE),
            "exposure_ns" to r.get(CaptureResult.SENSOR_EXPOSURE_TIME),
            "iso" to r.get(CaptureResult.SENSOR_SENSITIVITY),
            "frame_duration_ns" to r.get(CaptureResult.SENSOR_FRAME_DURATION),
            "active_physical_id" to r.get(CaptureResult.LOGICAL_MULTI_CAMERA_ACTIVE_PHYSICAL_ID),
        )
    }

    private fun onImage(r: ImageReader) {
        val img = r.acquireNextImage() ?: return
        try {
            val w = config.width
            val h = config.height
            val p = img.planes[0]
            val buf = p.buffer
            val rs = p.rowStride
            if (p.pixelStride == 1) {
                for (row in 0 until h) { buf.position(row * rs); buf.get(yBuf, row * w, w) }
            } else {
                for (row in 0 until h) for (c in 0 until w) yBuf[row * w + c] = buf.get(row * rs + c * p.pixelStride)
            }
            listener?.onFrame(yBuf, img.timestamp)
        } finally {
            img.close()
        }
    }

    fun close() {
        try { session?.stopRepeating() } catch (_: Exception) {}
        session?.close(); session = null
        device?.close(); device = null
        reader?.close(); reader = null
        lastResult = null
    }

    fun quit() { close(); thread.quitSafely() }
}
