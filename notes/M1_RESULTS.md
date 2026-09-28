# M1 results: vision front end + vision-only odometry

Code: `PC-part/proto/deskmouse/{frontend,vo,sim,metrics,config}.py`, `scripts/replay.py`, `scripts/simulate.py`.

## Front end (multi-depth, notes/DECISIONS.md)
Per frame: LK with gyro-predicted initial flow → forward-backward check → undistort → gyro de-rotation →
robust fit of d_i = ρ_i·w + δψ·J·n_i + σ·n_i (RANSAC over tracks with known depth, LS refine) →
per-track inverse depth ρ_i by least squares over the track's history → drop tracks that stay inconsistent.
Scale is relative (one unknown factor for all ρ_i); vision-only mode fixes it by assuming the 15th percentile
of ρ is the ceiling (1.9 m). Fusion (M2) will get scale from the IMU instead.

## Synthetic (exact ground truth), ruler_x 5 round trips of 20 cm

| Scene | stroke length | stroke std | cross-axis | final error |
|---|---|---|---|---|
| textured ceiling | 20.0 cm | 0.01 % | 0.03 % | 0.0 mm |
| ceiling + shelf (0.45 m) + monitor (0.30 m) | 19.9 cm | 0.2 % | 0.03 % | 1.0 mm |
| + moving distractor | 19.8 cm | 1.1 % | 0.65 % | 6.4 mm |

→ M1 synthetic targets met, including multi-depth and a moving distractor.

## Real (place 1)
- Tracks ~80, inliers median 55–69 (target ≥ 30); fast strokes dip to 5–20 inliers briefly.
- Direction: cross-axis 2.8–2.9 % on ruler_x / ruler_y (target < 3 %).
- **Vision-only scale is fragile here**: most tracks are on the shelf/monitor, so the "farthest cluster = ceiling" gauge moves by up to 2× as the view changes. Expected: scale must come from the IMU (M2).
- With the gauge frozen after 6 s: ruler_y strokes std 3.7 %, round-trip closure 3.2 mm; ruler_x take 1 std 14.5 %.
- **IMU-only per-stroke (ZUPT at both ends of each stroke):** ruler_y **20.7 cm ± 3.3 %** (nominal "about 20"), i.e. the accelerometer alone is very good over single strokes. Vision vs IMU stroke lengths: corr 0.84, 2.0 % residual after one scale factor.
- Slow ruler_x (take 1, ~5 cm/s): IMU-only 14.8 cm ± 17 %, vision and IMU disagree (corr 0.13). Slow strokes are the IMU's weak case (little acceleration to integrate, bias dominates); to resolve in M2.
- Pause detection needs work on the fast take and on ruler_x take 2 (metrics code, not tracking).

## Next (M2)
EKF fusing IMU (prediction, ZUPT) with the front end's w (measurement, per-track depths with one scale state).
The IMU fixes scale; vision fixes IMU drift on slow strokes.
