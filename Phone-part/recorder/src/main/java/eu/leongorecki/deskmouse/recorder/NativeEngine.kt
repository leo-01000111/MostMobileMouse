package eu.leongorecki.deskmouse.recorder

import android.hardware.camera2.CameraCharacteristics

/** Thin wrapper around the C++ engine (PC-part/core) via JNI. Not thread-safe: use from one thread. */
class NativeEngine(K: DoubleArray, dist: DoubleArray, width: Int, height: Int, params: Map<String, Double>, gaugeRatio: Double) {
    enum class Kind { MOVE, LEFT, RIGHT, WHEEL, LIFT, LAND }  // same order as deskmouse::EventKind
    data class Ev(val kind: Kind, val dx: Int, val dy: Int, val wheel: Int)

    private var h = nCreate(K, dist, width, height, params.keys.toTypedArray(), params.values.toDoubleArray(), gaugeRatio)

    fun imu(t: LongArray, kinds: ByteArray, xyz: FloatArray, n: Int) { if (n > 0) nImu(h, t, kinds, xyz, n) }
    fun frame(y: ByteArray, tNs: Long, exposureNs: Long, skewNs: Long) = nFrame(h, y, tNs, exposureNs, skewNs)

    fun drain(): List<Ev> {
        val a = nDrain(h)
        return List(a.size / 4) { Ev(Kind.entries[a[4 * it]], a[4 * it + 1], a[4 * it + 2], a[4 * it + 3]) }
    }

    /** [still, lifted, scrolling, scale, frames, taps, scale_updates, lifts, inliers, gauge_ratio]; NaN = none. */
    fun state(): DoubleArray = nState(h)

    fun close() { if (h != 0L) { nDestroy(h); h = 0 } }

    companion object {
        init { System.loadLibrary("deskmouse_jni") }

        @JvmStatic private external fun nCreate(
            K: DoubleArray, dist: DoubleArray, w: Int, h: Int, names: Array<String>, values: DoubleArray, gaugeRatio: Double,
        ): Long
        @JvmStatic private external fun nDestroy(h: Long)
        @JvmStatic private external fun nImu(h: Long, t: LongArray, kinds: ByteArray, xyz: FloatArray, n: Int)
        @JvmStatic private external fun nFrame(h: Long, y: ByteArray, t: Long, exposure: Long, skew: Long)
        @JvmStatic private external fun nDrain(h: Long): IntArray
        @JvmStatic private external fun nState(h: Long): DoubleArray

        /**
         * K (row-major 3×3) and Camera2 distortion at stream width. Same as io.py intrinsics_from: cameras with
         * placeholder intrinsics (S24 front) get K from focal length and sensor size, no distortion.
         */
        fun intrinsics(ch: CameraCharacteristics, width: Int): Pair<DoubleArray, DoubleArray> {
            val arr = ch.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)!!
            var (fx, fy, cx, cy) = (ch.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION) ?: FloatArray(5)).map { it.toDouble() }
            var dist = (ch.get(CameraCharacteristics.LENS_DISTORTION) ?: FloatArray(5)).map { it.toDouble() }.toDoubleArray()
            if (fx < 10) {
                val f = ch.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)!!.first().toDouble()
                val ps = ch.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)!!
                fx = f / ps.width * arr.width(); fy = f / ps.height * arr.height()
                cx = (arr.left + arr.right) / 2.0; cy = (arr.top + arr.bottom) / 2.0
                dist = DoubleArray(5)
            }
            val s = width.toDouble() / arr.width()
            val K = doubleArrayOf(fx * s, 0.0, (cx - arr.left) * s, 0.0, fy * s, (cy - arr.top) * s, 0.0, 0.0, 1.0)
            return K to dist
        }
    }
}
