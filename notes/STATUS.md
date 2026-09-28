# Status

**Current phase:** M0 recorder working on the S24 (test take passes all checks). User can start Tier 1. Next for me: M1 simulator (multi-depth) + front end.

| Milestone | State |
|-----------|-------|
| A0 device probe | done (notes/DEVICE.md, notes/device_caps_S24.json) |
| M0 recorder | working; acceptance passes on a 15 s test take; full check on Tier 1 set |
| M1 front end + simulator | not started |
| Tier 0 ceiling check (user) | place 1 done (notes/TIER0_RESULTS.md) |
| Tier 1 recordings (user) | first-priority set done (still, ruler_x/y, square, square_rot, twist); remaining: lift, taps, free, dark, lights, raw |
| M2 fusion + go/no-go | not started |
| M3–M7 | gated on M2 GO |

## Log
- 2026-09-28: Wrote plans/PLAN.md and plans/RECORDINGS_NEEDED.md; set up folders and CLAUDE.md.
- 2026-09-28: Device probe via adb: SM-S921B (Exynos 2400), REALTIME timestamps, 60 fps, manual sensor, OIS off-able, IMU 500 Hz, HID device enabled.
- 2026-09-28: Installed toolchain (JDK 21, NDK 29, CMake 3.31.6, platform 36). Scope widened to any ceiling/desk. Git initialised, remote = GitHub MostMobileMouse.
- 2026-09-28: Tier 0 place 1 analysed: texture OK; multi-depth scene breaks single-plane assumption (45% rigid inliers while moving) -> proposed per-track depth front end.
- 2026-09-28: M0 recorder built and installed (AGP 9.4.1, Gradle 9.8, Kotlin 2.4.20, compileSdk 37). Test take: 908 frames @ 60.0 fps, 0 dropped, IMU 500 Hz, REALTIME, OIS/EIS off, manual 2 ms / ISO 1534 indoors. Python loader + validator + pull script added.
- 2026-09-28: Tier 1 first-priority set recorded at place 1 (notes/TIER1_RESULTS.md). Camera–gyro sync verified: corr 0.9997, offset ~2 ms.
