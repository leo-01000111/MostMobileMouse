# Device and desk facts

## Phone
- **Samsung Galaxy S24, SM-S921B** (EU), SoC **Exynos 2400** (`s5e9945`), adb serial RZCY2161X6X.
- One UI 8.5, Android 16 (API 36).
- Case (yes/no, type): _ask user_
- Camera lens centre offset from phone centre, face-down (mm): _measure or take from spec drawings_

## Camera2 (read via `adb shell dumpsys media.camera`, 2026-09-28)

| | Main wide (id 0, logical) | Ultra-wide (id 2) |
|---|---|---|
| Hardware level | FULL | LIMITED |
| MANUAL_SENSOR | yes | yes |
| MANUAL_POST_PROCESSING | yes | no |
| Timestamp source | **REALTIME** | **REALTIME** |
| AE fps ranges | [15,15] [15,20] [20,20] [24,24] [15,30] [30,30] [15,60] **[60,60]** | same |
| OIS modes | OFF, ON (can be turned off) | OFF only (no OIS) |
| Video stabilization modes | OFF, ON, PREVIEW_STABILIZATION | same |
| AF modes | OFF, AUTO, MACRO, CONT_VIDEO, CONT_PICTURE | **OFF only (fixed focus)** |
| Min focus distance | 10 D (10 cm), focus calibration CALIBRATED | 0 (fixed), UNCALIBRATED |
| Focal length | 5.4 mm | 2.2 mm |
| Intrinsics fx, fy, cx, cy (full array) | 2728.5, 2725.0, 2035.6, 1514.5 | 1615.3, 1612.7, 1975.8, 1493.6 |
| Distortion k1..k3 | 0.0945, −0.1353, 0.0588 | 0.0015, 0.0197, −0.0143 |
| FOV (H × V, from intrinsics) | ≈ 74° × 59° | ≈ 101° × 85° |
| fx at 640×480 | ≈ 428 px | ≈ 262 px |
| Ceiling footprint at 1.8 m | ≈ 2.7 × 2.0 m | ≈ 4.4 × 3.3 m |
| Image shift per 1 mm phone slide at 1.8 m | ≈ 0.24 px | ≈ 0.15 px |
| Exposure range | 85 µs – 100 ms | 60 µs – 100 ms |
| ISO range | 25–3200 | 48–3200 |
| Sensor orientation | 90° | 90° |
| Physical sensor size | 8.16 × 6.12 mm | 5.6 × 4.2 mm |

Notes:
- Camera 0 is a **logical multi-camera** (physical ids 5, 6, 2). Pin zoom ratio 1.0 (or open the physical wide sensor) so it can't switch lenses mid-recording.
- The ultra-wide has no OIS and fixed focus, both good for this use; it sees ~2.7× more ceiling area. Main has better resolution per mm. Compare both in A4.
- Front cameras (ids 1, 3, 4) are irrelevant (they face the desk).

## IMU
- STMicro **LSM6DSVTR** accel + gyro, both **max 500 Hz** (min 6.25 Hz), calibrated and uncalibrated variants available.
- Also Game Rotation Vector, Linear Acceleration (Samsung fusion).

## Bluetooth
- `bluetooth.profile.hid.device.enabled = true`: the HID device profile (phone acting as a mouse) is enabled, so M6 looks feasible. Still confirm at runtime via `getProfileProxy(HID_DEVICE)`.

## Desk and room
Target: any ceiling, any smooth desk. Per-location notes go with the recordings (Tier 0 / variety set).
- Place 1 (Tier 0): _to fill in_
