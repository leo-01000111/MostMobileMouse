# Desk Mouse — Camera + IMU Visual-Inertial Odometry on a Phone

## Design Document

Turn a Samsung Galaxy phone lying **face-down on a desk** into a mouse. Sliding the phone moves the cursor, tapping its back clicks, and twisting it scrolls. Motion is estimated by fusing:

- the **rear camera**, which faces the ceiling and tracks ceiling features (drift-free, but delayed and texture-dependent), and
- the **IMU** (gyro + accelerometer, ~400 Hz: low latency, but drifts when integrated),

in an **extended Kalman filter** with zero-velocity updates.

This document is written to be handed to a coding agent. Development is deliberately **data-first**:
1. record real sensor data,
2. prototype the algorithm offline in Python,
3. make a go/no-go decision on measured accuracy,
4. only then build the real-time Android app.

Follow the milestone order in §11. Each milestone must meet its "done when" criteria before the next starts.

---

## 1. Goals and non-goals

**Goals**
- Relative cursor motion from sliding the phone on a desk, at mouse-like precision (sub-millimetre jitter, low drift, feels immediate).
- Lifting the phone pauses tracking, like lifting a real mouse.
- Back-tap clicks (single = left, double = right) and twist-to-scroll.
- Screen-free operation: the phone lies face-down with the display off; all interaction is physical.
- Output over Wi-Fi (UDP + small PC receiver) or as a native Bluetooth HID mouse.
- A recorder app and offline evaluation tools that measure accuracy against simple ground truth.

**Non-goals**
- Full 6-DOF visual-inertial odometry. Motion is assumed planar (§2).
- Working on arbitrary surfaces without a ceiling in view (e.g. under a shelf).
- Absolute positioning. Output is relative mouse deltas.
- iOS.

## 2. Physical setup, assumptions, frames

**Setup:** phone face-down on a flat desk, top edge pointing away from the user. The rear camera faces the ceiling, roughly 1.2–2.2 m away. A case is recommended to protect the screen.

**Assumptions** (the algorithm relies on these; detect violations):
- The desk is a plane. The phone moves with 3 DOF: translation X, Y and yaw ψ.
- The ceiling region in view is approximately a plane parallel to the desk, at unknown but constant height h above the camera.
- Tilt from the desk is small and constant while the phone lies on it. Lifting is detected (§6.5).

**Frames**
- **Device frame (Android):** x right, y toward top edge, z out of screen.
- **Body frame B (face-down):** X_B = −x_dev (user's right when face-down), Y_B = +y_dev (forward), Z_B = −z_dev (up).
- **World frame W:** desk plane, fixed at session start. Z up; ψ = yaw of B relative to W.
- **Image frame:** undistorted **normalized** coordinates (pixels ÷ focal length, principal point at origin). The 2×2 mapping `M_ci` from body XY to image xy (a rotation by the sensor orientation plus a possible mirror) is determined by the axis calibration in §6.1.
- **Reference point:** mouse motion is reported for a configurable point on the phone (default: geometric centre), not the camera lens. The lens offset `r_cam` (in B, metres) is a setting with per-model defaults, so twisting the phone doesn't create spurious translation.

## 3. Tech stack

- **Android app:** Kotlin + Jetpack Compose for UI and services. **Camera2 API** (needs manual control of focus, exposure and frame rate). C++ via the NDK for the real-time core. minSdk 31 (Android 12), target latest stable.
- **Real-time core:** C++17 with OpenCV (core, imgproc, video) and Eigen. Exposed to Android via JNI and to Python via **pybind11**, so the same code runs in the offline evaluation harness.
- **Offline prototype and tools:** Python 3.11+, NumPy, SciPy, OpenCV-Python, pandas, matplotlib.
- **PC receiver:** Python 3.11+, `pynput` (optional Linux backend via `python-evdev` / uinput).

## 4. Repository layout

```
desk-mouse/
├── DESIGN.md
├── README.md                     # setup, recording protocol, manual test checklist
├── android/
│   ├── recorder/                 # M0: data recorder app (separate Gradle module)
│   └── app/                      # M4+: real-time mouse app
│       └── src/main/
│           ├── java/.../deskmouse/
│           │   ├── MainActivity.kt          # settings, calibration, status
│           │   ├── MouseService.kt          # foreground service (camera + sensors)
│           │   ├── camera/CameraSource.kt   # Camera2 setup, ImageReader → JNI
│           │   ├── sensors/ImuSource.kt     # SensorManager → JNI
│           │   ├── transport/{MouseTransport,UdpTransport,BtHidTransport}.kt
│           │   └── settings/Settings.kt
│           └── cpp/ → symlink or CMake reference to /core
├── core/                         # C++ real-time core (shared)
│   ├── include/deskmouse/
│   ├── src/
│   │   ├── frontend.cpp           # feature tracking, de-rotation, translation estimate
│   │   ├── ekf.cpp                # filter, delayed updates
│   │   ├── detectors.cpp          # stillness, lift, taps, twist-scroll
│   │   ├── output.cpp             # velocity → mouse counts, acceleration curve
│   │   └── pipeline.cpp           # wires everything, timestamp-ordered event queue
│   ├── python/bindings.cpp        # pybind11 module `deskmouse_core`
│   └── tests/
├── proto/                        # M1–M2: pure-Python reference implementation
│   ├── deskmouse/{io,frontend,ekf,detectors,output,pipeline}.py
│   ├── scripts/replay.py          # run pipeline on a recording, plots + metrics
│   ├── scripts/evaluate.py        # batch metrics over protocol recordings
│   └── tests/
├── receiver/receiver.py
└── recordings/                   # small sample recordings for tests (git-lfs)
```

## 5. Recorder app and recording format (M0)

Minimal app: a big Start/Stop button, a protocol picker (§5.2), and a live indicator of IMU rate, camera fps and feature count (a quick Shi-Tomasi count on a downsampled frame, so the ceiling can be judged on the spot).

### 5.1 Capture settings
- Rear **main** (wide) camera, **640×480** YUV_420_888 via `ImageReader`, **60 fps** (`CONTROL_AE_TARGET_FPS_RANGE = [60,60]`). Fall back to 30 fps if 60 isn't available, and record which was used.
- **Focus:** manual (`CONTROL_AF_MODE_OFF`), `LENS_FOCUS_DISTANCE` = 1 / 1.8 m (settable).
- **Exposure:** manual when supported: exposure time ≤ 4 ms (default 2 ms) and ISO adjusted for brightness, set once by a short auto-exposure metering pass at start, then locked. Otherwise use AE lock after metering.
- Disable OIS and video stabilization and any EIS (`LENS_OPTICAL_STABILIZATION_MODE_OFF`, `CONTROL_VIDEO_STABILIZATION_MODE_OFF`). Stabilization moves the image and corrupts odometry.
- IMU: `TYPE_GYROSCOPE`, `TYPE_ACCELEROMETER`, **uncalibrated** variants too if available, at `SENSOR_DELAY_FASTEST`, with `HIGH_SAMPLING_RATE_SENSORS` declared. Also `TYPE_GAME_ROTATION_VECTOR` for reference.
- Recording starts after a 3 s countdown (so the user can place the phone) and plays a beep at start and stop.

### 5.2 Recording protocols (ground truth)
Each recording carries a protocol tag. The README describes how to perform each one:

| Tag          | Procedure                                                                  | Ground truth                          |
|--------------|----------------------------------------------------------------------------|---------------------------------------|
| `still`      | Phone untouched for 60 s                                                   | Zero motion                           |
| `ruler_x/y`  | Slide the phone along a ruler/straightedge 20 cm, pause, back; ×5          | ±20 cm along one axis, 0 across       |
| `square`     | Trace a 15 cm square taped on the desk, 3 laps                             | Closed loop, known side length        |
| `twist`      | Rotate in place ±45°, several times                                        | Zero translation at the reference point |
| `lift`       | Slide, lift, move in the air, put down, slide                              | Lift intervals (user presses volume key while lifted) |
| `taps`       | Tap the back, labelled via volume keys (up = left-intent, down = right-intent) | Tap timestamps + intent            |
| `free`       | Normal mouse-like use for 2 min                                            | None (feel / sanity)                  |
| `dark`, `lights` | Same as `square` under dim light / directly under a lamp               | Robustness                            |

The volume keys serve as label buttons during recording (the screen is face-down).

### 5.3 File format
One directory per recording: `rec_YYYYMMDD_HHMMSS_<tag>/`
- `meta.json`: device model, Android version, protocol tag, camera id, resolution, fps, `SENSOR_INFO_TIMESTAMP_SOURCE`, `SENSOR_ORIENTATION`, `LENS_INTRINSIC_CALIBRATION`, `LENS_DISTORTION`, `LENS_POSE_TRANSLATION` / `LENS_POSE_ROTATION` (if available), focus distance, exposure time, ISO, the actual measured sensor rates, and app version.
- `imu.csv`: `t_ns, type, x, y, z` (type ∈ gyro, accel, gyro_unc, accel_unc, grv; for uncalibrated, add bias columns).
- `frames.csv`: `idx, t_ns (SENSOR_TIMESTAMP), exposure_ns, rolling_shutter_skew_ns, iso`.
- `video.mp4`: H.264 of the Y plane at high bitrate (≥ 20 Mbit/s), one video frame per `frames.csv` row, in the same order. Option **raw mode**: `frames.y8`, concatenated raw 8-bit Y planes (for short clips; ~18 MB/s).
- `labels.csv`: `t_ns, label` (volume key presses).

If the timestamp source is not `REALTIME`, record this prominently. Camera and IMU clocks then need an offset estimate (§6.6).

## 6. Algorithm

The Python reference (`proto/`) and the C++ core (`core/`) implement exactly this. All numeric values are **defaults**, exposed as config parameters in both.

### 6.1 Calibration and preprocessing
- **Intrinsics / distortion:** from `meta.json` (Camera2 values). If absent, fall back to a nominal FOV from the settings, and provide an optional chessboard calibration script in `proto/scripts/`.
- **Axis mapping `M_ci`:** computed from `SENSOR_ORIENTATION` and verified automatically on `ruler_x`/`ruler_y` recordings (the fitted image-motion direction must match within 5°). The app has a one-step "slide forward 10 cm" calibration that estimates `M_ci` (rotation angle + mirror flag) directly from data.
- **Gyro bias:** EMA update during detected stillness (§6.4), τ = 3 s, starting from zero, or from `gyro_unc` bias if provided.
- **Desk tilt / gravity:** during stillness, average the accelerometer to obtain gravity direction `ĝ_B`. Compute the rotation `R_tilt` that aligns it with −Z_B, and apply it to all IMU samples. The horizontal specific force is `a_h = [R_tilt·a]_{X,Y}`.
- **Yaw ψ:** integrate the bias-corrected gyro Z_B component. Correct slowly with the camera-derived rotation (§6.2 step 5) through a complementary filter (camera weight 0.02 per frame), so yaw doesn't drift over long sessions.

### 6.2 Vision front end (per frame)
1. Take the Y plane (640×480). Optionally apply CLAHE when the median intensity is low.
2. **Track** existing features from the previous frame with pyramidal Lucas–Kanade (window 21×21, 3 levels, 30 iterations / ε 0.01). Use the gyro-predicted rotation as the initial flow guess.
3. **Forward–backward check:** re-track back to the previous frame, and drop tracks with an error > 0.5 px.
4. **Undistort** tracked points to normalized coordinates.
5. **De-rotate:** rotate the previous frame's points about the principal point by the gyro yaw increment between the two frame timestamps, mapped into image axes via `M_ci`. Also estimate the residual rotation from the tracks (a least-squares 2D rigid fit), and report it as a camera yaw measurement for §6.1.
6. **Translation:** residual displacements `d_i`. Estimate `d̂` by 2-DOF RANSAC (inlier threshold 0.3 px equivalent, ≥ 50 iterations), refined by the mean of the inliers. Covariance `R_img` = sample covariance of the inliers ÷ n_inliers, floored at (0.02 px)² equivalent.
7. **Scale check:** a similarity fit on the inliers. If |scale − 1| > 1.5 %, flag `scale_anomaly` (suggests lifting or tilting).
8. **Replenish** features: if tracks < 80, detect Shi-Tomasi corners (quality 0.01, min distance 12 px) in grid cells (4×4) lacking tracks, up to 150 total, masking out saturated pixels (≥ 250) and a 3 px margin around them.
9. **Frame quality** `q ∈ [0,1]`: based on the inlier count (≥ 30 → 1, ≤ 8 → 0), mean LK error and the saturation fraction. If q < 0.2, skip the update (the filter runs IMU-only).

Output per frame: `{t_ns (mid-exposure), d̂ (normalized units), R_img, dψ_cam, n_inliers, q, scale_anomaly}`. Timestamp = `SENSOR_TIMESTAMP` + exposure/2 (+ half the rolling-shutter skew, since the features are spread over the frame).

### 6.3 EKF

**State (world frame, metres):**
```
x = [p_x, p_y, v_x, v_y, b_ax, b_ay, ρ]
```
p is the reference-point position, v its velocity, b the horizontal accelerometer bias (body frame), and ρ = 1/h the inverse ceiling height (m⁻¹). Initial ρ = 1/1.8, σ_ρ = 0.15. A "ceiling height" setting may override the initial value.

**Prediction (every IMU accel sample, ~400 Hz)**, with ψ from §6.1 as a known input:
```
a_W   = R(ψ) · (a_h − b_a)
p    += v·Δt + ½·a_W·Δt²
v    += a_W·Δt
b_a  : random walk, σ = 0.005 m/s²/√s
ρ    : random walk, σ = 0.002 m⁻¹/√s
accel noise σ_a = 0.05 m/s² (tune from `still` recordings)
```

**Camera update (per frame with q ≥ 0.2)**, as a velocity measurement over the frame interval Δt_f:
```
v_cam_W  = v − ω_z × R(ψ)·r_cam                # camera-lens velocity from reference-point velocity
z        = d̂ / Δt_f                            # normalized image units per second
h(x)     = −ρ · M_ci · R(ψ)ᵀ · v_cam_W
H        = [0₂ₓ₂, ∂h/∂v, 0₂ₓ₂, ∂h/∂ρ]  with ∂h/∂v = −ρ·M_ci·R(ψ)ᵀ, ∂h/∂ρ = −M_ci·R(ψ)ᵀ·v_cam_W
R_meas   = R_img / Δt_f² ÷ max(q, 0.2)
```
Apply a Mahalanobis gate (χ², 2 DOF, 99.9 %). Rejected updates are counted and exposed in debug stats.

**ZUPT (while stationary, §6.4):** pseudo-measurement v = 0 with σ = 0.002 m/s, at 50 Hz.

**Delayed updates:** camera measurements arrive 30–80 ms late. Keep a ring buffer (≥ 300 ms) of IMU samples and filter snapshots. On a camera measurement with timestamp t_m: restore the snapshot at or just before t_m, propagate to t_m, apply the update, then re-propagate through buffered IMU samples to "now". Measurements are processed in timestamp order. A measurement older than the buffer is dropped and counted.

**Clock offset (only if timestamp source ≠ REALTIME):** estimated offline by cross-correlating gyro yaw rate with camera `dψ_cam/Δt` on `twist` recordings. Online, it is a fixed setting.

### 6.4 Stillness detector
Stationary when all hold over the last 60 ms:
- |ω| < 0.03 rad/s (bias-corrected),
- horizontal accel std < 0.08 m/s²,
- the last camera frame (if q ≥ 0.2) has |d̂| < 0.05 px equivalent.

Hysteresis: enter after 60 ms of stillness, exit immediately on violation. Stillness drives the ZUPT, the gyro-bias and gravity/tilt updates, and the output deadband.

### 6.5 Lift detector
Lifted if any of:
- the gravity direction deviates > 4° from the resting `ĝ_B` (tracked with a short-horizon gyro attitude integration),
- vertical specific force deviates > 1.5 m/s² from g for > 30 ms,
- `scale_anomaly` on two consecutive frames.

While lifted: output is frozen and camera updates are ignored. On landing (stillness + tilt back within 2° for 100 ms): set v = 0 and inflate the v covariance, keep p, re-estimate tilt.

### 6.6 Taps (clicks)
The back now faces up, so tapping it is like pressing a mouse button.
- Jerk proxy `j = |a_Z[n] − a_Z[n−1]|` (body Z, raw accel).
- Adaptive threshold `thr = max(thr_min, median(j) + 8·MAD(j))` over a rolling 1 s window, excluding recent tap windows. `thr_min = 1.5 m/s²` per sample at ~400 Hz, scaled with the actual rate.
- Refractory period: 100 ms.
- **Motion suppression:** a tap pushes the phone into the desk and can jolt it. For the interval [t0 − 15 ms, t0 + 80 ms], skip camera updates and replace the accel input with the pre-tap value. Output deltas are delayed by `output_delay_ms` (default 25 ms) so the click can be ordered **before** any residual motion.
- **Click logic** (setting):
  - default "single = left, double = right" (double window 250 ms),
  - "instant left": left fires immediately, right = triple tap.
- **Drag:** tap-and-slide within 300 ms after a tap = held left button (press on the tap, release when the motion stops for > 200 ms, or on the next tap). Setting to disable.
- *Experimental (M7):* classify left/right from tap location (left vs right half of the back) using the roll-rate / accel-X signature, trained on `taps` recordings.

### 6.7 Twist-to-scroll
- Scroll mode is active only when the reference-point speed < 0.01 m/s (translation near zero) **and** |ω_z| > 0.3 rad/s for > 40 ms. This avoids the small rotations that happen naturally while moving.
- Wheel counts = accumulated Δψ / `scroll_step` (default 4° per notch), with sub-notch carry. Sign is configurable.
- While scroll mode is active, translation output is suppressed. It exits 150 ms after |ω_z| < 0.1 rad/s.

### 6.8 Output mapping
- Motion at the reference point, `Δp` in metres, is converted to counts like a real mouse: `counts = Δp · DPI / 0.0254`. Default DPI = 800, configurable.
- Acceleration curve: `gain = 1 + (g_max − 1)·smoothstep(v_lo, v_hi, |v|)`, with g_max = 2.5, v_lo = 0.05 m/s, v_hi = 0.5 m/s. Option: off (flat).
- Axes: +X_W → cursor right, +Y_W (forward) → cursor up (mouse dy negative). Settings: invert X/Y, and **world-aligned** (default; direction independent of phone yaw) vs **body-aligned** (like a real mouse sensor; motion rotates with the phone).
- Output deadband: when stationary, emit nothing. Sub-count carry per axis.
- Emit deltas at 125–250 Hz, coalesced.

## 7. Offline prototype and evaluation (M1–M2)

- `proto/scripts/replay.py <recording> [--config cfg.yaml] [--plot] [--only-vision] [--only-imu]` runs the pipeline and saves `out/<rec>/trajectory.csv`, `events.csv` (clicks, scroll, lifts), `stats.json`, and plots (trajectory XY, velocity vs time, feature count / q, EKF innovations, ρ and bias convergence, tap detections vs labels).
- `proto/scripts/evaluate.py recordings/ --config cfg.yaml` produces a metrics table across all protocol recordings:

| Metric                                            | Target (go)          |
|---------------------------------------------------|----------------------|
| `still`: drift over 60 s                          | < 0.5 mm             |
| `still`: output jitter                            | 0 counts (deadband)  |
| `ruler`: travel length error (after ρ converges)  | < 5 %                |
| `ruler`: cross-axis error                         | < 3 % of travel      |
| `square`: loop closure error per lap              | < 5 mm               |
| `twist`: reference-point translation              | < 3 mm per ±45°      |
| `lift`: lift detection recall / false lifts       | ≥ 98 % / 0           |
| `taps`: recall / false taps per min of `free`     | ≥ 95 % / ≤ 1         |
| Median inliers per frame (your ceiling)           | ≥ 30                 |
| Camera-to-output latency (IMU makes it lower)     | reported             |

- Also report the **IMU-only** (`--only-imu`: ZUPT without camera) and **vision-only** results, to show what fusion buys.
- **Go/no-go after M2:** if the targets aren't met on the user's own desk and ceiling, stop and report which component limits accuracy before starting Android real-time work.

## 8. Real-time Android app (M4+)

**Threads**
- Camera `ImageReader` callback thread: passes the Y plane pointer + timestamp to the core via JNI (zero-copy where possible; `ImageReader` maxImages = 4, drop frames if the core is busy — never queue them).
- IMU thread (`HandlerThread`): batches samples to the core.
- Core: the timestamp-ordered event queue runs on its own native thread. The front end must take < 10 ms per frame on the target phone (measure and display this).
- Transport thread: sends coalesced mouse events.

**Service**
- `MouseService` is a foreground service with `foregroundServiceType="camera|connectedDevice"`, started from the visible activity (required for background camera access). It holds a partial wake lock, so it keeps running with the screen off.
- Request the battery-optimization exemption (Samsung puts apps to sleep aggressively). The README includes One UI steps to set the app to "Unrestricted".
- Start/stop: from the app, and from a notification action.
- The status notification shows the connection state, fps, and a feature-quality indicator.
- Volume keys (while the service runs): long-press Vol-Down for 2 s to pause/resume tracking. Otherwise the keys pass through.

**Settings UI** (face-up use only): DPI, acceleration curve, axis options, ceiling-height override, tap/click mode, scroll step and sign, the calibration wizard (axis mapping + tap threshold check), transport selection, and a debug overlay with live plots of features, q, ρ and the stillness/lift state.

**Power:** show estimated battery drain. Setting: 30 fps "eco" mode.

## 9. Transports

### 9.1 UDP + receiver
Binary little-endian packets, sent at ≤ 250 Hz (coalesced), port 47474:

| Offset | Type  | Field                                    |
|-------:|-------|------------------------------------------|
| 0      | u16   | magic `0xD35C`                           |
| 2      | u8    | version = 1                              |
| 3      | u8    | buttons state bitmask (L=1, R=2, M=4)    |
| 4      | u32   | sequence number                          |
| 8      | i16   | dx (counts)                              |
| 10     | i16   | dy (counts)                              |
| 12     | i8    | wheel                                    |
| 13     | u8    | flags (bit0 = tracking paused/lifted)    |
| 14     | u8[2] | reserved                                 |

- Buttons are sent as **state**. The receiver diffs against the previous state, so a lost packet can't leave a button stuck. A heartbeat is sent every 200 ms. The receiver drops out-of-order packets and releases all buttons after 1 s of silence.
- Discovery: UDP broadcast `"DESKMOUSE?"` → reply `"DESKMOUSE!" + hostname`. Manual IP entry is also possible.
- `receiver.py --port --backend {pynput,uinput} --verbose` (prints packet rate, loss, and inter-arrival jitter).

### 9.2 Bluetooth HID
- `BluetoothHidDevice` with `SUBCLASS1_MOUSE`, and a standard mouse report descriptor: 3 buttons, relative X/Y int8, wheel int8. Split large deltas across several reports; ≤ 200 reports/s.
- Check support at startup via `getProfileProxy(HID_DEVICE)`. If it's unavailable, show a clear message and fall back to UDP. **Run this check on the target phone as the first task of M6.**
- Pairing: request discoverability, list bonded hosts, connect; remember the last host and auto-reconnect. Runtime permissions: `BLUETOOTH_CONNECT`, `BLUETOOTH_SCAN`, `BLUETOOTH_ADVERTISE`.

## 10. Testing

- **Python unit tests (`proto/tests`):** the EKF on synthetic 1D/2D trajectories (constant velocity, strokes with stops, with known bias and ρ) must converge; the de-rotation on synthetic rotated point sets; RANSAC with outliers; the stillness, lift, tap and scroll detectors on synthetic signals; the output mapping (DPI, sub-count carry).
- **Synthetic end-to-end:** a generator renders a textured ceiling plane (random texture image) through a pinhole model for a given planar trajectory, plus simulated IMU with noise and bias. The pipeline must recover the trajectory within the §7 targets. This lets the algorithm be developed before any recording exists.
- **Parity (M3):** the C++ core run through pybind11 on every sample recording must match the Python reference: trajectory within 1 % of path length, identical click/scroll events (±5 ms).
- **C++ unit tests:** GoogleTest in `core/tests`.
- **Receiver tests:** packet parsing, out-of-order drop, button-state diffing, timeout release.
- **Manual checklist** in the README per milestone: hitting 20×20 px targets, drift when idle for 5 min, lift-and-reposition, clicks during motion, twist-scroll without cursor movement, dim room, under a lamp, Wi-Fi drop recovery.

## 11. Milestones

**M0 — Recorder app**
- The §5 recorder with all capture settings, protocols, labels, the live feature count, and the file format. Export via the share sheet or a USB-accessible folder.
- ✅ Done when: recordings of every protocol load in Python, frame count = `frames.csv` rows, IMU ≥ 400 Hz, camera ≥ 55 fps (or 30 fps documented), and the timestamp source is recorded.

**M1 — Offline vision front end + synthetic simulator**
- `proto/`: I/O, the synthetic ceiling/IMU generator, §6.2 front end, vision-only odometry (fixed ρ from the height setting), plots.
- ✅ Done when: synthetic tests pass, and on real `ruler`/`square` recordings the vision-only trajectory is visibly correct with the median inlier count reported.

**M2 — Offline EKF fusion (go/no-go)**
- §6.1, §6.3–§6.5 in Python, plus taps (§6.6) and scroll (§6.7) and output mapping (§6.8). `evaluate.py` produces the metrics table.
- ✅ Done when: the §7 targets are met on the user's recordings, **or** a written report explains the limiting factor. Decide go/no-go with the user before M3.

**M3 — C++ core + parity**
- Port §6 to `core/` (OpenCV + Eigen), with pybind11 bindings and parity tests against the Python reference.
- ✅ Done when: parity tests pass on all sample recordings, and the front end takes < 10 ms per frame on a desktop CPU (the phone is measured in M4).

**M4 — Real-time Android app + UDP**
- JNI integration, `MouseService`, Camera2 + IMU sources, settings UI and debug overlay, `UdpTransport`, `receiver.py` with discovery.
- ✅ Done when: the cursor is usable end-to-end, 20×20 px targets can be hit, no drift when idle, front end < 10 ms per frame on the phone, and it runs 30 min with the screen off without being killed.

**M5 — Clicks and scroll in real time**
- Tap detection, click modes, tap-drag and twist-scroll running live, plus the calibration wizard.
- ✅ Done when: the manual checklist passes, and false taps ≤ 1 per minute of normal use.

**M6 — Bluetooth HID**
- HID support check, registration, pairing, reconnect.
- ✅ Done when: the phone pairs with Windows/Linux/macOS as a mouse with no PC software, and behaviour matches M5.

**M7 — Optional**
- Tap-location left/right classifier, rolling-shutter correction, an air-mouse mode (phone in hand, gyro → cursor; reuses the transports, taps and output mapping), and multi-plane or non-parallel ceiling robustness.

## 12. Risks and open questions

- **Ceiling texture:** a uniform white ceiling may give too few features. The recorder's live feature count and the M2 metrics decide. Mitigations: lights and fixtures as features, CLAHE, and an IMU+ZUPT fallback (degraded).
- **Lighting extremes:** saturated lamps (masked out; used as blob features if needed) and dark rooms (noise; longer exposure trades off against motion blur).
- **Samsung camera specifics:** whether manual exposure/focus, 60 fps at 640×480, and `REALTIME` timestamps are available on the target model. M0 answers this.
- **Latency:** Android camera pipeline delay. The IMU bridges it; M2 and M4 measure the real end-to-end figure.
- **Power/thermal:** continuous camera + processing. Measure in M4; eco mode as mitigation.
- **Target phone model:** to be confirmed (affects lens offset `r_cam`, HID support, camera capabilities).
