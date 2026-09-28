# MVP: phone as a mouse over USB (2026-09-28)

Built at the user's request from the session-1 recordings, **skipping the M2 go/no-go gate** (user's decision,
notes/DECISIONS.md). Not the final architecture: the algorithm runs in Python on the PC; the phone only streams.

## How it works
Phone (recorder app, "Mouse mode") → TCP over USB (`adb forward tcp:47475`) → `PC-part/receiver/mvp_live.py`
→ `deskmouse.engine.Engine` → Windows `SendInput`.
- Motion: multi-depth front end (M1) → translation, rotated by gyro yaw (world-aligned), × scale.
- Scale: rough ceiling guess at first; after each stroke, IMU-only stroke length (ZUPT at both ends) recalibrates it.
  The scale is saved to `PC-part/receiver/state.json` and reused next time.
- Still → no output. Tap (accel-z jerk > 0.4) → left click after 250 ms; double tap → right click.
  Twist (|ω| > 0.3 rad/s) → scroll, 4° per notch. Tilt > 4° or vision scale change → lifted, output frozen.
- Replay on recordings: `python PC-part/proto/scripts/mvp_replay.py recordings/<rec> --plot`.

## Offline check on recordings
still: 0 cursor motion. square: 3 recognisable laps, scale calibrated after 16 strokes. twist: scroll back and forth
(net 8–11 of 380–450 notches), 250–300 px cursor leakage. taps: all taps click (volume presses also count).
Live link: 60 fps processed, 0 skipped, front end ~9 ms/frame.

## Known limitations
- USB cable required; Windows only; the PC does the work.
- Latency = camera pipeline + transfer (not yet measured), no IMU prediction yet.
- Scale is off until the first few strokes; cursor directions assume the phone's top edge points away from you
  (use `--mount-yaw 90 / -90 / 180` otherwise).
- Lens offset is a spec estimate, so twisting leaks some cursor motion.
- No drag, no left/right by tap position, no Bluetooth.

## Aim-lab game + auto-tuning (PC-part/game/README.md)
Same levels with mouse (baseline) and phone; phone sessions log IMU + front-end results so the tuner can adjust
direction, vertical gain, tap and scroll thresholds and stillness. Verified with a distorted test bot (converges in
~3 sessions). Measured live latency: camera frame reaches the PC ~56 ms after mid-exposure (p95 68 ms), IMU ~22 ms.
In the game, phone frames were processed at ~45 fps in a short test (vs 60 in mvp_live): the status line shows fps.
