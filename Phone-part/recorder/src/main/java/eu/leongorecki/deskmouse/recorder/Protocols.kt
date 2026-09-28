package eu.leongorecki.deskmouse.recorder

/** Recording protocols from DESIGN.md §5.2 / plans/RECORDINGS_NEEDED.md. Durations are auto-stop defaults. */
data class Protocol(val tag: String, val seconds: Int, val howTo: String)

val PROTOCOLS = listOf(
    Protocol("still", 60, "Don't touch the phone or the desk."),
    Protocol("ruler_x", 60, "Slide 20 cm right along the ruler, pause 2 s, back, pause 2 s. ×5."),
    Protocol("ruler_y", 60, "Slide 20 cm forward along the ruler, pause 2 s, back, pause 2 s. ×5."),
    Protocol("square", 75, "Trace the 15 cm square, 3 laps, 1 s pause at corners, top edge forward."),
    Protocol("square_rot", 75, "Square, but with the phone rotated ~30° the whole time."),
    Protocol("twist", 45, "Rotate in place ±45°, 8–10 times, 1 s pause between."),
    Protocol("lift", 75, "Slide, lift 3–5 cm, move, put down, slide. Hold Vol-Up while in the air. ~8 lifts."),
    Protocol("taps", 100, "~40 taps. Vol-Up right after a left tap, Vol-Down right after a right tap."),
    Protocol("free", 125, "Use it like a mouse. No labels."),
    Protocol("dark", 75, "Square protocol, lights dimmed."),
    Protocol("lights", 75, "Square protocol, lamp directly in view."),
    Protocol("test", 15, "Quick test take."),
)
