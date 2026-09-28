package eu.leongorecki.deskmouse.recorder

import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.media.MediaMuxer
import java.io.BufferedOutputStream
import java.io.File
import java.io.FileOutputStream

/** Receives compact Y planes (width*height bytes) in capture order. */
interface FrameSink {
    /** Returns false if the frame could not be stored (it must then be left out of frames.csv). */
    fun write(y: ByteArray, tNs: Long): Boolean
    fun close()
}

/** Raw mode: frames.y8, concatenated 8-bit Y planes. */
class RawSink(file: File) : FrameSink {
    private val out = BufferedOutputStream(FileOutputStream(file), 8 shl 20)
    override fun write(y: ByteArray, tNs: Long): Boolean { out.write(y); return true }
    override fun close() { out.flush(); out.close() }
}

/**
 * H.264 of the Y plane (chroma fixed at 128), one video frame per captured frame, no B-frames.
 * Presentation time = sensor timestamp relative to the first frame.
 */
class H264Sink(file: File, private val width: Int, private val height: Int, fps: Int, bitrate: Int) : FrameSink {
    private val codec = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC)
    private val muxer = MediaMuxer(file.absolutePath, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)
    private var track = -1
    private var muxing = false
    private var t0 = -1L
    private val info = MediaCodec.BufferInfo()

    init {
        val fmt = MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, width, height).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible)
            setInteger(MediaFormat.KEY_BIT_RATE, bitrate)
            setInteger(MediaFormat.KEY_BITRATE_MODE, MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR)
            setInteger(MediaFormat.KEY_FRAME_RATE, fps)
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)
            setInteger(MediaFormat.KEY_MAX_B_FRAMES, 0)
        }
        codec.configure(fmt, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
        codec.start()
    }

    override fun write(y: ByteArray, tNs: Long): Boolean {
        if (t0 < 0) t0 = tNs
        val idx = codec.dequeueInputBuffer(30_000)
        if (idx < 0) { drain(false); return false }
        val img = codec.getInputImage(idx)!!
        val yp = img.planes[0]
        val yb = yp.buffer
        val rs = yp.rowStride
        if (yp.pixelStride == 1) {
            for (r in 0 until height) { yb.position(r * rs); yb.put(y, r * width, width) }
        } else {
            for (r in 0 until height) for (c in 0 until width) yb.put(r * rs + c * yp.pixelStride, y[r * width + c])
        }
        for (p in 1..2) {
            val b = img.planes[p].buffer
            b.position(0)
            while (b.hasRemaining()) b.put(128.toByte())
        }
        codec.queueInputBuffer(idx, 0, width * height * 3 / 2, (tNs - t0) / 1000, 0)
        drain(false)
        return true
    }

    private fun drain(eos: Boolean) {
        var waits = 0
        while (true) {
            val idx = codec.dequeueOutputBuffer(info, if (eos) 10_000 else 0)
            when {
                idx == MediaCodec.INFO_TRY_AGAIN_LATER -> if (!eos || ++waits > 200) return
                idx == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> {
                    track = muxer.addTrack(codec.outputFormat); muxer.start(); muxing = true
                }
                idx >= 0 -> {
                    val buf = codec.getOutputBuffer(idx)!!
                    if (info.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG == 0 && info.size > 0 && muxing) {
                        buf.position(info.offset); buf.limit(info.offset + info.size)
                        muxer.writeSampleData(track, buf, info)
                    }
                    codec.releaseOutputBuffer(idx, false)
                    if (info.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0) return
                }
            }
        }
    }

    override fun close() {
        val idx = codec.dequeueInputBuffer(500_000)
        if (idx >= 0) codec.queueInputBuffer(idx, 0, 0, 0, MediaCodec.BUFFER_FLAG_END_OF_STREAM)
        drain(true)
        codec.stop(); codec.release()
        if (muxing) muxer.stop()
        muxer.release()
    }
}
