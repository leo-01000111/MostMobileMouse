# Status

**Current phase:** M0 recorder working on the S24 (test take passes all checks). User can start Tier 1. Next for me: M1 simulator (multi-depth) + front end.

| Milestone | State |
|-----------|-------|
| A0 device probe | done (notes/DEVICE.md, notes/device_caps_S24.json) |
| M0 recorder | working; acceptance passes on a 15 s test take; full check on Tier 1 set |
| M1 front end + simulator | done on synthetic (targets met); real data: tracking/direction OK, vision-only scale fragile → IMU needed (notes/M1_RESULTS.md) |
| Tier 0 ceiling check (user) | place 1 done (notes/TIER0_RESULTS.md) |
| Tier 1 recordings (user) | first-priority set done (still, ruler_x/y, square, square_rot, twist); remaining: lift, taps, free, dark, lights, raw |
| M2 fusion + go/no-go | skipped for now by user's choice; MVP engine (stroke-calibrated scale) instead |
| MVP (USB stream + PC engine) | working end to end, awaiting user's hands-on test (notes/MVP.md) |
| Aim-lab game + auto-tune | built, self-tested with bot; awaiting user's mouse baseline + phone sessions |
| On-phone engine + Bluetooth mouse | C++ core (MVP engine port) runs on the phone, ~4.3 ms/frame; parity PASS on 7 recordings; Bluetooth HID built, awaiting user's pairing + hands-on test |
| M3–M7 (DESIGN EKF path) | gated on M2 GO; user chose the MVP engine on the phone instead (DECISIONS 2026-10-05) |

## Log
- 2026-09-28: Wrote plans/PLAN.md and plans/RECORDINGS_NEEDED.md; set up folders and CLAUDE.md.
- 2026-09-28: Device probe via adb: SM-S921B (Exynos 2400), REALTIME timestamps, 60 fps, manual sensor, OIS off-able, IMU 500 Hz, HID device enabled.
- 2026-09-28: Installed toolchain (JDK 21, NDK 29, CMake 3.31.6, platform 36). Scope widened to any ceiling/desk. Git initialised, remote = GitHub MostMobileMouse.
- 2026-09-28: Tier 0 place 1 analysed: texture OK; multi-depth scene breaks single-plane assumption (45% rigid inliers while moving) -> proposed per-track depth front end.
- 2026-09-28: M0 recorder built and installed (AGP 9.4.1, Gradle 9.8, Kotlin 2.4.20, compileSdk 37). Test take: 908 frames @ 60.0 fps, 0 dropped, IMU 500 Hz, REALTIME, OIS/EIS off, manual 2 ms / ISO 1534 indoors. Python loader + validator + pull script added.
- 2026-09-28: Tier 1 first-priority set recorded at place 1 (notes/TIER1_RESULTS.md). Camera–gyro sync verified: corr 0.9997, offset ~2 ms.
- 2026-09-28: M1: multi-depth front end + vision-only VO + simulator. Synthetic 20 cm strokes within 0.2–1 %. Real: inliers 55–69, cross-axis <3 %, IMU-only strokes 20.7 cm ±3.3 % on ruler_y.
- 2026-09-28: Recording session ended by user (tired). Still missing for M2: lift, free, dark, lights, raw ruler_x (plus other places later). Phone app free to reinstall now.
- 2026-09-28: MVP built: phone Mouse mode (TCP stream), PC engine + SendInput runner. Live link 60 fps, 0 skipped.
- 2026-09-28: Aim-lab game (6 levels) + tuner; bot self-test converges (15° → 0.2°). Live latency ~56 ms camera→PC.
- 2026-10-05: Recorder: front-camera option for face-up tests (meta.json `capture.facing` / `phone_orientation`); io.py estimates K for cameras with placeholder intrinsics. Face-up layout decided (DECISIONS.md). Face-up takes listed in RECORDINGS_NEEDED §2.6.
- 2026-10-05: Moved tracking onto the phone (user's call, past the M2 gate): C++ port of the MVP engine in PC-part/core (OpenCV 4.13 AAR), JNI in the recorder app, "Bluetooth mouse" mode sends HID reports with no PC software. Engine 4.0–4.5 ms/frame on the S24 (max 12–31 ms). Parity on the phone (scripts/parity.py, first 15 s of still, ruler_x, ruler_y, square, square_rot, twist, taps): engine exact on all 7; closed-loop path error 0–1.7 %, within Python's own 1-grey-level noise floor; clicks and scroll identical. RANSAC now uses a shared SplitMix64 sampler.
