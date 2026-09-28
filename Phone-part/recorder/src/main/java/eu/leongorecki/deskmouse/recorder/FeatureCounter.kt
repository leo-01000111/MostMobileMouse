package eu.leongorecki.deskmouse.recorder

import kotlin.math.max
import kotlin.math.sqrt

/**
 * Quick ceiling-quality indicator: Shi-Tomasi corner count on a 2× (or more) downsampled
 * Y plane, quality 0.01 of the strongest corner, 3×3 non-max suppression. Also the fraction of
 * saturated pixels. Rough by design: it's for judging a ceiling on the spot, not for tracking.
 */
object FeatureCounter {
    data class Result(val corners: Int, val saturatedFrac: Float, val w: Int, val h: Int, val small: ByteArray)

    fun analyse(y: ByteArray, width: Int, height: Int): Result {
        val f = max(1, width / 320)
        val w = width / f
        val h = height / f
        val img = FloatArray(w * h)
        val small = ByteArray(w * h)
        var sat = 0
        for (yy in 0 until h) for (xx in 0 until w) {
            var s = 0
            for (dy in 0 until f) for (dx in 0 until f) {
                val v = y[(yy * f + dy) * width + xx * f + dx].toInt() and 0xFF
                s += v
                if (v >= 250) sat++
            }
            val m = s / (f * f)
            img[yy * w + xx] = m.toFloat()
            small[yy * w + xx] = m.toByte()
        }
        // Structure tensor min eigenvalue with 3×3 window.
        val ix = FloatArray(w * h)
        val iy = FloatArray(w * h)
        for (yy in 1 until h - 1) for (xx in 1 until w - 1) {
            val i = yy * w + xx
            ix[i] = (img[i + 1] - img[i - 1]) * 0.5f
            iy[i] = (img[i + w] - img[i - w]) * 0.5f
        }
        val lam = FloatArray(w * h)
        var maxLam = 0f
        for (yy in 2 until h - 2) for (xx in 2 until w - 2) {
            var a = 0f; var b = 0f; var c = 0f
            for (dy in -1..1) for (dx in -1..1) {
                val j = (yy + dy) * w + xx + dx
                a += ix[j] * ix[j]; b += ix[j] * iy[j]; c += iy[j] * iy[j]
            }
            val l = (a + c) / 2f - sqrt(((a - c) / 2f) * ((a - c) / 2f) + b * b)
            lam[yy * w + xx] = l
            if (l > maxLam) maxLam = l
        }
        val thr = maxLam * 0.01f
        var n = 0
        if (maxLam > 0f) for (yy in 3 until h - 3) for (xx in 3 until w - 3) {
            val i = yy * w + xx
            val l = lam[i]
            if (l <= thr) continue
            var isMax = true
            loop@ for (dy in -1..1) for (dx in -1..1) {
                if ((dy != 0 || dx != 0) && lam[i + dy * w + dx] >= l) { isMax = false; break@loop }
            }
            if (isMax) n++
        }
        return Result(n, sat.toFloat() / (w * h * f * f).coerceAtLeast(1), w, h, small)
    }
}
