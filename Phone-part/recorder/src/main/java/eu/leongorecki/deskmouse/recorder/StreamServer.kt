package eu.leongorecki.deskmouse.recorder

import android.util.Log
import java.io.BufferedOutputStream
import java.net.ServerSocket
import java.net.Socket
import java.net.SocketTimeoutException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.atomic.AtomicInteger

/**
 * MVP live stream to the PC over TCP (reach it through USB with `adb forward tcp:47475 tcp:47475`).
 * Little-endian messages:
 *   1 hello: u32 len, UTF-8 JSON (camera characteristics, size, fps)
 *   2 imu:   i64 t_ns, u8 kind (0 gyro, 1 accel), f32 x, y, z
 *   3 frame: i64 t_ns, i32 exposure_ns, i32 skew_ns, u16 w, u16 h, w*h bytes (Y plane)
 * Frames are dropped (never queued) when more than [maxPendingFrames] are waiting.
 */
class StreamServer(val port: Int = 47475, private val maxPendingFrames: Int = 2) {
    private val server = ServerSocket(port).apply { soTimeout = 500 }
    private var socket: Socket? = null
    private val queue = LinkedBlockingQueue<ByteArray>()
    private val pendingFrames = AtomicInteger(0)
    @Volatile var connected = false
        private set
    val sentFrames = AtomicInteger(0)
    val droppedFrames = AtomicInteger(0)
    private var writer: Thread? = null

    /** Waits for the PC to connect; returns false when [keepWaiting] turns false. */
    fun awaitClient(keepWaiting: () -> Boolean): Boolean {
        while (keepWaiting()) {
            try {
                val s = server.accept()
                s.tcpNoDelay = true
                s.sendBufferSize = 4 shl 20
                socket = s
                connected = true
                writer = Thread({ writeLoop(s) }, "stream-writer").apply { start() }
                return true
            } catch (_: SocketTimeoutException) {
            }
        }
        return false
    }

    private fun writeLoop(s: Socket) {
        try {
            val out = BufferedOutputStream(s.getOutputStream(), 1 shl 20)
            while (connected) {
                var msg = queue.take()
                while (true) {
                    if (msg.isEmpty()) { out.flush(); return } // poison pill
                    out.write(msg)
                    if (msg[0].toInt() == 3) pendingFrames.decrementAndGet()
                    msg = queue.poll() ?: break
                }
                out.flush()
            }
        } catch (e: Exception) {
            Log.i("DeskStream", "client gone: ${e.message}")
        } finally {
            connected = false
        }
    }

    fun hello(json: String) {
        val b = json.toByteArray()
        queue.put(ByteBuffer.allocate(5 + b.size).order(ByteOrder.LITTLE_ENDIAN).put(1).putInt(b.size).put(b).array())
    }

    fun imu(tNs: Long, kind: Int, x: Float, y: Float, z: Float) {
        if (!connected) return
        queue.put(ByteBuffer.allocate(22).order(ByteOrder.LITTLE_ENDIAN)
            .put(2).putLong(tNs).put(kind.toByte()).putFloat(x).putFloat(y).putFloat(z).array())
    }

    fun frame(y: ByteArray, w: Int, h: Int, tNs: Long, exposureNs: Int, skewNs: Int) {
        if (!connected) return
        if (pendingFrames.get() >= maxPendingFrames) { droppedFrames.incrementAndGet(); return }
        val bb = ByteBuffer.allocate(21 + w * h).order(ByteOrder.LITTLE_ENDIAN)
            .put(3).putLong(tNs).putInt(exposureNs).putInt(skewNs).putShort(w.toShort()).putShort(h.toShort())
        bb.put(y, 0, w * h)
        pendingFrames.incrementAndGet()
        sentFrames.incrementAndGet()
        queue.put(bb.array())
    }

    fun close() {
        connected = false
        queue.put(ByteArray(0))
        try { socket?.close() } catch (_: Exception) {}
        try { server.close() } catch (_: Exception) {}
        writer?.join(1000)
    }
}
