# Recordings I need from you

Two tiers. **Tier 0** is optional, uses the stock camera, and can be done today. **Tier 1** uses the recorder app I build in M0 and is what the go/no-go decision rests on.

Nothing here blocks me from starting to write code.

---

## 1. Tier 0: ceiling check (before M0, optional, ~10 minutes)

Goal: find out early whether your ceiling has enough texture to track. A plain white ceiling is the biggest risk in the whole project, and this answers it without any custom code.

Since it has to work on **any ceiling**, do this at **3–5 different places**. Include your own desk and **the plainest, most boring ceiling you can find** (a uniform white one is the one we most need to see). Other good candidates: a kitchen or dining table, a room with a very bright ceiling lamp, somewhere with a high ceiling.

At each place:

1. **Note** roughly how high the ceiling is above the table (a tape measure if handy, otherwise a guess), what the table surface is, and what's up there: plain paint, textured plaster, beams, lamps, a fan, shelves or walls in view?
2. **Photos (2):** stock Camera app, rear camera, lay the phone **face-down** and shoot with the 3 s timer. One photo at **1× (main camera)** and one at **0.6× (ultra-wide)**.
3. **Video (1), ~20 s**, main camera 1×: Camera settings → turn **video stabilisation / Super steady off**, 1080p at 60 fps if offered. Start recording, lay the phone face-down, wait 3 s, slide it slowly ~20 cm left and back, forward and back, then a slow twist. Stop.
4. Tell me which photos/video belong to which place (e.g. "place 1 = my desk, place 2 = kitchen"). I'll pull the files over USB myself from the phone's camera folder, so you don't need to copy anything.

What I'll do with it: count trackable features per frame for each place and camera (main vs ultra-wide), check exposure and motion blur, and tell you which ceilings look fine, which are borderline, and what that means for the design (e.g. switching to the ultra-wide camera, or an IMU-only fallback for blank ceilings).

---

## 2. Tier 1: full protocol set (after M0; needed for M1 validation and M2 go/no-go)

### 2.1 What you'll need on the desk

- The phone, **in a case** (screen protection), charged > 60 %.
- A **30 cm ruler or straightedge**, taped down so it can't move. The phone slides along it.
- A **15 cm square** marked with masking tape on the desk (corners clearly marked).
- A tape measure (ceiling height, measured once).
- A way to note phone dimensions: I'll ask for the model and use its spec sheet for the lens offset. If you have calipers, measuring the camera lens centre from the phone's top-left corner (face-down) helps.
- Do-not-disturb on the phone, so notifications don't vibrate it.

### 2.2 General rules for every take

- Phone always starts and ends **flat on the desk, top edge pointing away from you**.
- The app counts down 3 s and beeps. Keep hands off during the first **2 s after the beep** and the last 2 s before stopping (gives the filter stillness to initialise bias and tilt).
- Don't cover the camera with your hand. Hold the phone by its sides.
- Volume keys are the label buttons (face-down). Only press them where the protocol says to.
- If something goes wrong mid-take (knocked the desk, someone turned a light on), just do it again. Retakes are cheap; bad ground truth is expensive.

### 2.3 The recording list

| # | Tag | Takes | Length | Exactly what to do | Ground truth |
|---|-----|-------|--------|--------------------|--------------|
| 1 | `still` | 2 | 60 s each | Don't touch the phone or the desk. Take 1 in daylight / normal light, take 2 in evening light | zero motion |
| 2 | `ruler_x` | 3 | ~45 s | Phone's **left or right edge** against the ruler. Slide 20 cm to the right, pause 2 s, slide back to the start, pause 2 s. Repeat ×5. Take 1 slow (~5 cm/s), take 2 normal, take 3 fast (flick-like) | ±20 cm in X, 0 in Y |
| 3 | `ruler_y` | 3 | ~45 s | Same, but the phone's **side edge** runs along a ruler pointing away from you; slide forward 20 cm and back. Same three speeds | ±20 cm in Y, 0 in X |
| 4 | `square` | 3 | ~60 s | Trace the taped 15 cm square, 3 laps, pausing ~1 s at each corner. Keep the phone's top edge pointing forward the whole time. Take 1 and 2 clockwise, take 3 counter-clockwise | closed loop, side 15 cm |
| 5 | `square` (rotated) | 1 | ~60 s | Same as #4 but with the phone **rotated ~30°** on the desk the whole time | tests world-aligned output |
| 6 | `twist` | 2 | ~40 s | Rotate the phone in place ±45° around its centre, 8–10 times, at varying speed. Pause 1 s between each | zero translation |
| 7 | `lift` | 2 | ~60 s | Slide, then **lift** the phone ~3–5 cm, move it around in the air, put it down somewhere else, slide again. Do ~8 lifts. **Hold Vol-Up while the phone is in the air** (press at lift-off, release on landing) | lift intervals |
| 8 | `taps` | 2 | ~90 s | Tap the back with one finger like a mouse button, ~40 taps total. Press **Vol-Up right after a "left" tap** and **Vol-Down right after a "right" tap**. Mix in: some double taps, some taps on the left half vs right half of the back, and in take 2 about 10 taps **while sliding** | tap times + intent |
| 9 | `free` | 2 | 2 min | Use it like a real mouse: move, stop, small corrections, a few twists, a few taps, one or two lifts. No labels | none (false-tap rate, feel) |
| 10 | `dark` | 1 | ~60 s | `square` protocol with the room lights dimmed (e.g. just a monitor or a small lamp away from the desk) | robustness |
| 11 | `lights` | 1 | ~60 s | `square` protocol positioned so a lamp/ceiling light is **directly** in view | robustness |
| 12 | `ruler_x` **raw mode** | 1 | ~15 s | Toggle "raw frames" in the app, then one ruler_x round trip at normal speed | checks video compression isn't hurting tracking |

**Totals:** 23 takes, roughly 25–30 minutes of recording, plus setup. About 3–4 GB on the phone (the 2-minute `free` takes are the big ones). Do the full set at **your main desk**.

### 2.4 Variety set (other ceilings and desks, ~10 min per place)

Because it has to work anywhere, repeat a short subset at **each other place from Tier 0** (at least 3 places total, including the plainest ceiling), and on **at least 3 desk surfaces** overall: a hard one (wood/laminate), glass or very smooth, and a soft one (mousepad, desk mat, tablecloth).

| Tag | Takes | Notes |
|-----|-------|-------|
| `still` | 1 | 30 s is enough |
| `ruler_x` | 1 | normal speed |
| `square` | 1 | if you can't tape a square there, trace any known rectangle (e.g. a sheet of A4 paper: 29.7 × 21 cm; say which in the notes) |
| `taps` | 1 | ~20 taps; this is where the desk surface matters most |
| `free` | 1 | 1 min |

The recorder will have a "location" text field so each take is tagged with the place and desk surface.

### 2.5 Handing them over

Either:
- plug the phone in with USB debugging on and tell me; I'll run the pull script into `recordings/`, or
- copy the `rec_YYYYMMDD_HHMMSS_<tag>/` folders from `Android/data/<app package>/files/recordings/` into `recordings/` yourself.

Also write down, once (or put it in `notes/DEVICE.md`):
- ceiling height above the desk (cm),
- desk surface (wood, laminate, mousepad, glass…),
- anything odd about any take ("take 2 of ruler_y, the ruler slipped").

I'll validate each recording (frame counts, IMU rate, fps, timestamps) and ask for specific retakes if needed.

---

## 3. Possible later recordings (only if M2 says we need them)

- More `taps` for the left/right tap-location classifier (M7).
- `free` sessions with the real-time app in M5 for false-tap tuning.
