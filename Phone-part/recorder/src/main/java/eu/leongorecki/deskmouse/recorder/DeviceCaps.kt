package eu.leongorecki.deskmouse.recorder

import android.graphics.ImageFormat
import android.hardware.Sensor
import android.hardware.SensorManager
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.os.Build
import org.json.JSONArray
import org.json.JSONObject

/** Everything we might need to know about the cameras and IMU, as JSON (A0 capabilities dump). */
object DeviceCaps {
    fun device(): JSONObject = JSONObject()
        .put("manufacturer", Build.MANUFACTURER).put("model", Build.MODEL).put("device", Build.DEVICE)
        .put("soc", if (Build.VERSION.SDK_INT >= 31) Build.SOC_MODEL else "")
        .put("android_release", Build.VERSION.RELEASE).put("sdk_int", Build.VERSION.SDK_INT)
        .put("build", Build.DISPLAY)

    fun camera(cm: CameraManager, id: String): JSONObject {
        val c = cm.getCameraCharacteristics(id)
        val o = JSONObject().put("id", id)
        fun put(name: String, v: Any?) { o.put(name, toJson(v)) }
        put("facing", c.get(CameraCharacteristics.LENS_FACING))
        put("hardware_level", c.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL))
        put("capabilities", c.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES))
        put("physical_ids", c.physicalCameraIds.toList())
        put("timestamp_source", c.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE))
        put("sensor_orientation", c.get(CameraCharacteristics.SENSOR_ORIENTATION))
        put("focal_lengths", c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS))
        put("min_focus_distance", c.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE))
        put("focus_distance_calibration", c.get(CameraCharacteristics.LENS_INFO_FOCUS_DISTANCE_CALIBRATION))
        put("af_modes", c.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES))
        put("ois_modes", c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION))
        put("video_stabilization_modes", c.get(CameraCharacteristics.CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES))
        put("ae_fps_ranges", c.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)?.map { "${it.lower}-${it.upper}" })
        put("zoom_ratio_range", c.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE)?.let { listOf(it.lower, it.upper) })
        put("intrinsics", c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION))
        put("distortion", c.get(CameraCharacteristics.LENS_DISTORTION))
        put("pose_translation", c.get(CameraCharacteristics.LENS_POSE_TRANSLATION))
        put("pose_rotation", c.get(CameraCharacteristics.LENS_POSE_ROTATION))
        put("pose_reference", c.get(CameraCharacteristics.LENS_POSE_REFERENCE))
        put("active_array", c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)?.flattenToString())
        put("pre_correction_active_array", c.get(CameraCharacteristics.SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE)?.flattenToString())
        put("pixel_array", c.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)?.toString())
        put("physical_size_mm", c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)?.let { listOf(it.width, it.height) })
        put("exposure_range_ns", c.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE)?.let { listOf(it.lower, it.upper) })
        put("iso_range", c.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE)?.let { listOf(it.lower, it.upper) })
        put("max_frame_duration_ns", c.get(CameraCharacteristics.SENSOR_INFO_MAX_FRAME_DURATION))
        c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)?.let { map ->
            val sizes = JSONArray()
            for (s in map.getOutputSizes(ImageFormat.YUV_420_888) ?: emptyArray()) {
                val d = map.getOutputMinFrameDuration(ImageFormat.YUV_420_888, s)
                sizes.put(JSONObject().put("size", s.toString()).put("min_frame_duration_ns", d)
                    .put("max_fps", if (d > 0) 1e9 / d else JSONObject.NULL))
            }
            o.put("yuv_sizes", sizes)
        }
        return o
    }

    fun cameras(cm: CameraManager): JSONArray {
        val arr = JSONArray()
        val seen = mutableSetOf<String>()
        for (id in cm.cameraIdList) {
            arr.put(camera(cm, id)); seen += id
            for (pid in cm.getCameraCharacteristics(id).physicalCameraIds) {
                if (seen.add(pid)) arr.put(camera(cm, pid).put("physical_of", id))
            }
        }
        return arr
    }

    fun sensors(sm: SensorManager): JSONArray {
        val arr = JSONArray()
        for (s in sm.getSensorList(Sensor.TYPE_ALL)) {
            arr.put(JSONObject().put("name", s.name).put("vendor", s.vendor).put("type", s.type)
                .put("string_type", s.stringType).put("min_delay_us", s.minDelay).put("max_delay_us", s.maxDelay)
                .put("resolution", s.resolution.toDouble()).put("max_range", s.maximumRange.toDouble())
                .put("fifo_max", s.fifoMaxEventCount).put("wakeup", s.isWakeUpSensor))
        }
        return arr
    }

    fun all(cm: CameraManager, sm: SensorManager): JSONObject =
        JSONObject().put("device", device()).put("cameras", cameras(cm)).put("sensors", sensors(sm))

    fun toJson(v: Any?): Any = when (v) {
        null -> JSONObject.NULL
        is IntArray -> JSONArray(v.toList())
        is FloatArray -> JSONArray(v.map { it.toDouble() })
        is LongArray -> JSONArray(v.toList())
        is Collection<*> -> JSONArray(v.map { toJson(it) })
        is Float -> v.toDouble()
        else -> v
    }
}
