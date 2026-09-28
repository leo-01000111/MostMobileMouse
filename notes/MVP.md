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

### First real sessions (2026-09-28)
Mouse vs phone: Targets 0.80 vs 3.44 s/target; Trace 92 % vs 29 % inside; Precision 1.6 vs 3.9 s; Scroll 1.7 vs 5.4 s/zone;
Rhythm 5/20 vs 0/20 (level too hard → made easier). Phone first moves cover 52 % of the distance (mouse 71 %).
Tuner v1 had bugs (tap threshold learned from noise, scroll from extremes, a meaningless lag number); fixed and re-tuned
from defaults: mount_yaw −3.0°, aspect 1.13, dpi 800 → 1000, tap 0.40 (kept: rhythm taps too weak to separate from
bumps), scroll 0.30 → 0.51, stillness unchanged.

### Progress over phone rounds (aim lab)
| Round | Change before it | Targets s | Trace % | Precision s | Rhythm |
|---|---|---|---|---|---|
| 1 (19:21) | defaults | 3.44 | 29 | 3.9 | 0/20 |
| 3 (19:33) | tuned v2, scroll off | 2.98 | 21 | 4.0 | 9/16 |
| 4 (19:44) | body-aligned output, scale carry-over fix | **1.13** | **69** | **2.7** | 11/16 |
| mouse | | 0.80 | 92 | 1.6 | 5/20 (old harder rhythm) |
Round 4: no direction drift over time (−7° → −3°), first-move amplitude 0.69 (mouse 0.71). Game processed 42 fps
(fixed: cheaper skip check, 120 Hz redraw). Camera→PC latency p50 51 ms. Next: latency compensation.

### Latency compensation (2026-09-28)
The cursor leads by velocity × 55 ms (camera velocity, refined with accelerometer samples newer than the frame),
computed in output counts after the gain so the lead always returns exactly to zero. Offline on round 4
(`PC-part/game/replay_session.py`), target = uncompensated path 56 ms later: RMS error 55.6 → 25.7 px, p95 108 → 53 px,
~7 px overshoot at stops. First attempt added the lead in metres before the speed-dependent gain and drifted
(250–450 px); fixed. Default on (`latency_comp_ms = 55`).
Camera in use: id 0 at 1× → physical 5 = main wide 50 MP (middle lens of the three).
