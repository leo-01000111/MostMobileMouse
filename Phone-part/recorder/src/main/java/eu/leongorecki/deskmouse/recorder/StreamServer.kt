package eu.leongorecki.deskmouse.recorder

import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.util.Log
import java.io.BufferedOutputStream
import java.io.ByteArrayOutputStream
import java.net.ServerSocket
import java.net.Socket
import java.net.SocketTimeoutException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.atomic.AtomicInteger

/**
 * MVP live stream to the PC over TCP: through USB with `adb forward tcp:47475 tcp:47475`, or directly over Wi-Fi.
 * Little-endian messages:
 *   1 hello: u32 len, UTF-8 JSON (camera characteristics, size, fps)
 *   2 imu:   i64 t_ns, u8 kind (0 gyro, 1 accel), f32 x, y, z
 *   3 frame: i64 t_ns, i32 exposure_ns, i32 skew_ns, u16 w, u16 h, w*h bytes (Y plane)
 *   4 jpeg:  i64 t_ns, i32 exposure_ns, i32 skew_ns, u16 w, u16 h, u32 len, len bytes (JPEG of the Y plane)
 * Frames go as JPEG ([jpegQuality]) when the client is not on loopback (Wi-Fi carries ~60 Mbit/s here, raw
 * 640×480 at 60 fps needs ~150); over USB (adb forward = loopback) they go raw.
 * Frames are dropped (never queued) when more than [maxPendingFrames] are waiting.
 */
class StreamServer(val port: Int = 47475, private val maxPendingFrames: Int = 2, private val jpegQuality: Int = 90) {
    private val server = ServerSocket(port).apply { soTimeout = 500 }
    private var socket: Socket? = null
    private val queue = LinkedBlockingQueue<ByteArray>()
    private val pendingFrames = AtomicInteger(0)
    @Volatile var connected = false
        private set
    val sentFrames = AtomicInteger(0)
    val droppedFrames = AtomicInteger(0)
    private var writer: Thread? = null
    @Volatile var jpeg = false
        private set
    private var nv21: ByteArray? = null
    private val jpegOut = ByteArrayOutputStream(1 shl 17)

    /** Waits for the PC to connect; returns false when [keepWaiting] turns false. */
    fun awaitClient(keepWaiting: () -> Boolean): Boolean {
        while (keepWaiting()) {
            try {
                val s = server.accept()
                s.tcpNoDelay = true
                s.sendBufferSize = 4 shl 20
                socket = s
                jpeg = !s.inetAddress.isLoopbackAddress
                Log.i("DeskStream", "client ${s.inetAddress}, jpeg=$jpeg")
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
                    if (msg[0].toInt() == 3 || msg[0].toInt() == 4) pendingFrames.decrementAndGet()
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
        if (jpeg) { jpegFrame(y, w, h, tNs, exposureNs, skewNs); return }
        val bb = ByteBuffer.allocate(21 + w * h).order(ByteOrder.LITTLE_ENDIAN)
            .put(3).putLong(tNs).putInt(exposureNs).putInt(skewNs).putShort(w.toShort()).putShort(h.toShort())
        bb.put(y, 0, w * h)
        pendingFrames.incrementAndGet()
        sentFrames.incrementAndGet()
        queue.put(bb.array())
    }

    /** Y plane + flat chroma (128) as NV21, so the JPEG's luma is the camera's Y. */
    private fun jpegFrame(y: ByteArray, w: Int, h: Int, tNs: Long, exposureNs: Int, skewNs: Int) {
        val n = w * h
        val buf = nv21?.takeIf { it.size == n * 3 / 2 } ?: ByteArray(n * 3 / 2).also { it.fill(128.toByte(), n); nv21 = it }
        System.arraycopy(y, 0, buf, 0, n)
        jpegOut.reset()
        YuvImage(buf, ImageFormat.NV21, w, h, null).compressToJpeg(Rect(0, 0, w, h), jpegQuality, jpegOut)
        val len = jpegOut.size()
        val bb = ByteBuffer.allocate(25 + len).order(ByteOrder.LITTLE_ENDIAN)
            .put(4).putLong(tNs).putInt(exposureNs).putInt(skewNs).putShort(w.toShort()).putShort(h.toShort()).putInt(len)
        bb.put(jpegOut.toByteArray())
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
