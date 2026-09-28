# Tier 1 results (recorder app)

Raw data in `recordings/` (gitignored). All takes 640×480 @ 60 fps, main camera (id 0 → physical 5), manual exposure, OIS/EIS off.

## Place 1, session 2026-09-28 (desk with large cloth mouse pad, ceiling 1.90 m)

| Tag | Takes | Notes |
|---|---|---|
| still | 2 × 60 s | image drift ≤ 0.11 / 0.03 px over 60 s; gyro noise σ ≈ 0.0013 rad/s, accel σ ≈ 0.006 m/s² at 500 Hz; desk tilt ≈ 0.6°; take 1 has 4 camera-skipped frames at ~55 s |
| ruler_x | 3 × 60 s | slow / normal / fast; ~10–25 strokes; motion almost purely along image x (cross-axis a few px) |
| ruler_y | 3 × 15 s + 1 × 60 s | 15 s takes end mid-stroke (duration field was 15); the 60 s take is the clean one; motion along image y |
| square | 3 × ~30 s | 3 laps each, stopped early with both volume keys; 1 take lost (app reinstalled mid-take) |
| square_rot | 3 × ~24 s | square edges ~35° in image; yaw steady within ±8° |
| twist | 2 × 27–38 s | ±84° / ±68° |

**Axis mapping (first look):** sliding along body X → image x; body Y → image y; phone yaw → image rotation = +gyro_z (no sign flip).

**Camera–IMU time sync (twist, cross-correlation of camera rotation rate vs gyro z):** correlation 0.9997 / 0.998, offset **+2.0 / +2.5 ms** (camera timestamp = SENSOR_TIMESTAMP + exposure/2 + skew/2), rotation gain 1.03 / 0.98. REALTIME clock confirmed; a fixed ~2 ms offset is enough.

**Crude vision-only (median flow, no depth handling):** strokes and squares clearly visible; scale ~3× what the ceiling alone predicts (20 cm → ~130 px vs ~45 px), and laps shrink/drift: nearby objects dominate → the approved multi-depth front end is needed.

**Exposure:** metered light is bimodal across takes (≈4 000 vs ≈10 000–13 000 µs·ISO), so something (probably the user leaning over the desk) shades the camera during metering even with the brightest-of-1.5 s rule. Several takes ran at 3–4 ms / ISO 3200 instead of 2 ms. Usable, but planned fix: an option to reuse one exposure for the whole session.

Still to record: lift, taps, free, dark, lights, raw-mode ruler_x.
