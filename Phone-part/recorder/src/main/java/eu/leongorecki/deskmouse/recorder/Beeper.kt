package eu.leongorecki.deskmouse.recorder

import android.media.AudioManager
import android.media.ToneGenerator

/** Audible cues for face-down use. No vibration: it would corrupt the IMU data. */
class Beeper {
    private val tone = ToneGenerator(AudioManager.STREAM_MUSIC, 100)

    fun tick() = tone.startTone(ToneGenerator.TONE_PROP_BEEP, 80)
    fun start() = tone.startTone(ToneGenerator.TONE_CDMA_HIGH_L, 300)
    fun stop() = tone.startTone(ToneGenerator.TONE_PROP_BEEP2, 400)
    fun error() = tone.startTone(ToneGenerator.TONE_SUP_ERROR, 800)
    fun release() = tone.release()
}
