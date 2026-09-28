package eu.leongorecki.deskmouse.recorder

import android.hardware.SensorManager
import android.hardware.camera2.CameraManager
import android.os.SystemClock
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * One take on disk (DESIGN.md §5.3): rec_YYYYMMDD_HHMMSS_<tag>/ with meta.json, imu.csv,
 * frames.csv, video.mp4 or frames.y8, labels.csv.
 */
class Recording(root: File, val protocol: Protocol, val location: String, val cfg: CaptureConfig, val raw: Boolean) {
    val dir = File(root, "rec_" + SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date()) + "_" + protocol.tag)
        .apply { mkdirs() }

    val sink: FrameSink = if (raw) RawSink(File(dir, "frames.y8"))
    else H264Sink(File(dir, "video.mp4"), cfg.width, cfg.height, cfg.fps, bitrateFor(cfg))

    /** SENSOR_TIMESTAMP of each stored frame, in storage order. */
    val frameTs = ArrayList<Long>(protocol.seconds * cfg.fps + 64)
    var droppedFrames = 0
    private val labels = StringBuilder("t_ns,label\n")
    var startNs = 0L
    var endNs = 0L

    fun label(name: String) = synchronized(labels) {
        labels.append(SystemClock.elapsedRealtimeNanos()).append(',').append(name).append('\n')
    }

    fun finish(cam: CameraCapture, cm: CameraManager, sm: SensorManager, imu: ImuLogger, appVersion: String, notes: List<String>) {
        sink.close()
        File(dir, "labels.csv").writeText(synchronized(labels) { labels.toString() })

        var missingMeta = 0
        File(dir, "frames.csv").bufferedWriter().use { w ->
            w.write("idx,t_ns,exposure_ns,rolling_shutter_skew_ns,iso,frame_duration_ns,focus_diopters,active_physical_id\n")
            for ((i, t) in frameTs.withIndex()) {
                val m = cam.frameMeta[t]
                if (m == null) { missingMeta++; w.write("$i,$t,,,,,,\n"); continue }
                w.write("$i,$t,${m.exposureNs},${m.skewNs},${m.iso},${m.frameDurationNs},${m.focusDiopters},${m.activePhysicalId}\n")
            }
        }

        val durS = (endNs - startNs) / 1e9
        val physIds = cam.frameMeta.values.map { it.activePhysicalId }.filter { it.isNotEmpty() }.toSet()
        val meta = JSONObject()
            .put("format_version", 1)
            .put("app_version", appVersion)
            .put("protocol", protocol.tag)
            .put("location", location)
            .put("device", DeviceCaps.device())
            .put("start_elapsed_realtime_ns", startNs)
            .put("duration_s", durS)
            .put("storage", if (raw) "frames.y8" else "video.mp4")
            .put("capture", JSONObject()
                .put("camera_choice", cfg.choice.label).put("camera_id", cfg.choice.openId).put("zoom_ratio", cfg.choice.zoom.toDouble())
                .put("width", cfg.width).put("height", cfg.height).put("requested_fps", cfg.fps)
                .put("requested_exposure_ns", cfg.exposureNs).put("max_exposure_ns", cfg.maxExposureNs)
                .put("focus_distance_m", cfg.focusDistanceM.toDouble())
                .put("exposure_mode", cam.exposureMode)
                .put("metered_exposure_ns", cam.meteredExposureNs).put("metered_iso", cam.meteredIso)
                .put("applied_exposure_ns", cam.appliedExposureNs).put("applied_iso", cam.appliedIso)
                .put("applied_state", JSONObject(cam.appliedState().mapValues { DeviceCaps.toJson(it.value) }))
                .put("active_physical_ids", JSONArray(physIds.toList()))
                .put("video_bitrate", if (raw) JSONObject.NULL else bitrateFor(cfg)))
            .put("camera_characteristics", DeviceCaps.camera(cm, cfg.choice.openId))
            .put("physical_camera_characteristics", JSONArray(physIds.map { DeviceCaps.camera(cm, it) }))
            .put("measured", JSONObject()
                .put("frames", frameTs.size).put("dropped_frames", droppedFrames).put("frames_missing_meta", missingMeta)
                .put("fps", if (frameTs.size > 1) (frameTs.size - 1) * 1e9 / (frameTs.last() - frameTs.first()) else 0.0)
                .put("imu_rates_hz", JSONObject().apply {
                    for (k in ImuLogger.Kind.entries) put(k.label, imu.rateHz(k))
                }))
            .put("imu_sensors", JSONObject().apply {
                for ((k, s) in imu.availableSensors()) put(k.label, s?.let { "${it.name} (${it.vendor})" } ?: JSONObject.NULL)
            })
            .put("notes", JSONArray(notes))
        File(dir, "meta.json").writeText(meta.toString(1))
    }

    companion object {
        /** ≥ 20 Mbit/s at 640×480, scaled with pixel count. */
        fun bitrateFor(cfg: CaptureConfig): Int = (20_000_000L * cfg.width * cfg.height / (640 * 480)).toInt().coerceAtLeast(20_000_000)
    }
}
