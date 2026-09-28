package eu.leongorecki.deskmouse.recorder

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import java.io.BufferedWriter
import java.io.File
import java.util.concurrent.atomic.AtomicLongArray

/**
 * Logs IMU samples to imu.csv: t_ns,type,x,y,z,a,b,c,d
 *  - gyro/accel: a..d empty
 *  - gyro_unc/accel_unc: a,b,c = bias estimate
 *  - grv: a = w (cos(θ/2)), b = heading accuracy if present
 * t_ns is SensorEvent.timestamp (CLOCK_BOOTTIME, same base as REALTIME camera timestamps).
 */
class ImuLogger(private val sm: SensorManager) : SensorEventListener {
    enum class Kind(val type: Int, val label: String) {
        GYRO(Sensor.TYPE_GYROSCOPE, "gyro"),
        ACCEL(Sensor.TYPE_ACCELEROMETER, "accel"),
        GYRO_UNC(Sensor.TYPE_GYROSCOPE_UNCALIBRATED, "gyro_unc"),
        ACCEL_UNC(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED, "accel_unc"),
        GRV(Sensor.TYPE_GAME_ROTATION_VECTOR, "grv"),
    }

    private val thread = HandlerThread("imu").apply { start() }
    private val handler = Handler(thread.looper)
    private var out: BufferedWriter? = null
    private val sensors = Kind.entries.associateWith { sm.getDefaultSensor(it.type) }
    private val byType = Kind.entries.associateBy { it.type }

    /** Sample counts and first/last timestamps per kind, for live and final rates. */
    val counts = AtomicLongArray(Kind.entries.size)
    val firstT = LongArray(Kind.entries.size) { -1 }
    val lastT = LongArray(Kind.entries.size) { -1 }

    fun availableSensors(): Map<Kind, Sensor?> = sensors

    /** Start listening. If [file] is null, samples are only counted (live rate display). */
    fun start(file: File?) {
        for (i in 0 until counts.length()) { counts.set(i, 0); firstT[i] = -1; lastT[i] = -1 }
        out = file?.bufferedWriter(bufferSize = 1 shl 20)?.also { it.write("t_ns,type,x,y,z,a,b,c,d\n") }
        for ((_, s) in sensors) if (s != null) sm.registerListener(this, s, SensorManager.SENSOR_DELAY_FASTEST, handler)
    }

    fun stop() {
        sm.unregisterListener(this)
        // Flush on the sensor thread so no callback races with close().
        val done = java.util.concurrent.CountDownLatch(1)
        handler.post { out?.flush(); out?.close(); out = null; done.countDown() }
        done.await()
    }

    fun quit() = thread.quitSafely()

    fun rateHz(kind: Kind): Double {
        val i = kind.ordinal
        val n = counts.get(i)
        if (n < 2 || firstT[i] < 0) return 0.0
        return (n - 1) * 1e9 / (lastT[i] - firstT[i]).coerceAtLeast(1)
    }

    override fun onSensorChanged(e: SensorEvent) {
        val kind = byType[e.sensor.type] ?: return
        val i = kind.ordinal
        counts.incrementAndGet(i)
        if (firstT[i] < 0) firstT[i] = e.timestamp
        lastT[i] = e.timestamp
        val w = out ?: return
        val v = e.values
        val sb = StringBuilder(96).append(e.timestamp).append(',').append(kind.label)
        for (k in 0 until 3) sb.append(',').append(v[k])
        for (k in 3 until 7) { sb.append(','); if (k < v.size) sb.append(v[k]) }
        sb.append('\n')
        w.write(sb.toString())
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}
}
