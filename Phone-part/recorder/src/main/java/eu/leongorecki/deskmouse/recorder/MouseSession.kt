package eu.leongorecki.deskmouse.recorder

import android.os.SystemClock
import android.util.Log
import org.json.JSONObject
import java.io.File
import java.util.concurrent.ConcurrentLinkedQueue

private const val TAG = "DeskMouse"

/**
 * On-phone mouse: camera frames + IMU → C++ engine → Bluetooth HID reports. No PC software.
 * The camera thread only hands over the newest frame; the engine runs on its own thread and skips frames
 * when it falls behind, like mvp_live.py.
 */
class MouseSession(
    private val engine: NativeEngine,
    private val bt: BtMouse,
    private val width: Int,
    private val height: Int,
) {
    private class Imu(val t: Long, val kind: Byte, val x: Float, val y: Float, val z: Float)

    private val imuQ = ConcurrentLinkedQueue<Imu>()
    private val lock = Object()
    private var slot = ByteArray(width * height)
    private var work = ByteArray(width * height)
    private var slotT = 0L; private var slotExp = 0L; private var slotSkew = 0L
    private var slotFull = false
    @Volatile private var running = true
    private var tArr = LongArray(256); private var kArr = ByteArray(256); private var vArr = FloatArray(768)

    // stats (read by the UI thread)
    @Volatile var processed = 0; @Volatile var skipped = 0; @Volatile var msMean = 0.0; @Volatile var msMax = 0.0
    @Volatile var state = DoubleArray(10) { Double.NaN }

    private val thread = Thread({ loop() }, "engine").apply { priority = Thread.MAX_PRIORITY; start() }

    fun onImu(t: Long, kind: Int, x: Float, y: Float, z: Float) { imuQ.add(Imu(t, kind.toByte(), x, y, z)) }

    /** Camera thread: copy the frame into the slot; an unprocessed older frame is dropped. */
    fun onFrame(y: ByteArray, t: Long, exposure: Long, skew: Long) {
        synchronized(lock) {
            if (slotFull) skipped++
            System.arraycopy(y, 0, slot, 0, width * height)
            slotT = t; slotExp = exposure; slotSkew = skew; slotFull = true
            lock.notify()
        }
    }

    fun stop() {
        running = false
        synchronized(lock) { lock.notify() }
        thread.join(1000)
        engine.close()
    }

    private fun feedImu() {
        var n = 0
        while (true) {
            val s = imuQ.poll() ?: break
            if (n == tArr.size) { tArr = tArr.copyOf(2 * n); kArr = kArr.copyOf(2 * n); vArr = vArr.copyOf(6 * n) }
            tArr[n] = s.t; kArr[n] = s.kind; vArr[3 * n] = s.x; vArr[3 * n + 1] = s.y; vArr[3 * n + 2] = s.z
            n++
        }
        engine.imu(tArr, kArr, vArr, n)
    }

    private fun loop() {
        var sum = 0.0; var cnt = 0; var mx = 0.0; var tWin = SystemClock.elapsedRealtime()
        while (running) {
            var t: Long; var exp: Long; var skew: Long
            synchronized(lock) {
                while (running && !slotFull) lock.wait(20)
                if (!running) return
                val tmp = work; work = slot; slot = tmp
                t = slotT; exp = slotExp; skew = slotSkew; slotFull = false
            }
            // IMU that arrived before the frame, in arrival order (as on the PC)
            feedImu()
            val t0 = SystemClock.elapsedRealtimeNanos()
            engine.frame(work, t, exp, skew)
            val ms = (SystemClock.elapsedRealtimeNanos() - t0) / 1e6
            sum += ms; cnt++; if (ms > mx) mx = ms
            processed++
            send(engine.drain())
            if (SystemClock.elapsedRealtime() - tWin > 1000) {
                msMean = sum / cnt; msMax = mx; sum = 0.0; cnt = 0; mx = 0.0; tWin = SystemClock.elapsedRealtime()
                state = engine.state()
            }
        }
    }

    private fun send(ev: List<NativeEngine.Ev>) {
        var dx = 0; var dy = 0; var wheel = 0
        for (e in ev) when (e.kind) {
            NativeEngine.Kind.MOVE -> { dx += e.dx; dy += e.dy }
            NativeEngine.Kind.WHEEL -> wheel += e.wheel
            NativeEngine.Kind.LEFT -> { flush(dx, dy, wheel); dx = 0; dy = 0; wheel = 0; bt.click(1) }
            NativeEngine.Kind.RIGHT -> { flush(dx, dy, wheel); dx = 0; dy = 0; wheel = 0; bt.click(2) }
            else -> {}
        }
        flush(dx, dy, wheel)
    }

    private fun flush(dx: Int, dy: Int, wheel: Int) { if (dx != 0 || dy != 0 || wheel != 0) bt.move(dx, dy, wheel) }

    companion object {
        /**
         * Engine parameters: PC-part/game/phone_params.json pushed to the app's files dir (aim-lab tuning), numeric and
         * boolean fields only; everything else keeps the C++ defaults. Returns (params, remembered gauge ratio).
         */
        fun loadParams(dir: File): Pair<Map<String, Double>, Double> {
            val out = mutableMapOf<String, Double>()
            var ratio = 0.0
            val f = File(dir, "phone_params.json")
            if (f.exists()) try {
                val j = JSONObject(f.readText())
                for (k in j.keys()) when (val v = j.get(k)) {
                    is Boolean -> out[k] = if (v) 1.0 else 0.0
                    is Number -> out[k] = v.toDouble()
                }
                ratio = out.remove("gauge_ratio") ?: 0.0
            } catch (e: Exception) { Log.w(TAG, "bad phone_params.json", e) }
            val st = File(dir, "mouse_state.json")
            if (st.exists()) try { ratio = JSONObject(st.readText()).getDouble("gauge_ratio") } catch (_: Exception) {}
            return out to ratio
        }

        fun saveRatio(dir: File, ratio: Double) {
            if (!ratio.isNaN() && ratio > 0) File(dir, "mouse_state.json").writeText(JSONObject().put("gauge_ratio", ratio).toString())
        }
    }
}
