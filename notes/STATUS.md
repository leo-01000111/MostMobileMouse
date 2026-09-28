# Status

**Current phase:** toolchain ready; user doing Tier 0 at several places; next step for me is A0/M0 recorder + M1 simulator.

| Milestone | State |
|-----------|-------|
| A0 device probe | mostly done via adb (notes/DEVICE.md) |
| M0 recorder | not started |
| M1 front end + simulator | not started |
| Tier 0 ceiling check (user) | in progress, 1 place for now |
| Tier 1 recordings (user) | blocked on M0 |
| M2 fusion + go/no-go | not started |
| M3–M7 | gated on M2 GO |

## Log
- 2026-09-28: Wrote plans/PLAN.md and plans/RECORDINGS_NEEDED.md; set up folders and CLAUDE.md.
- 2026-09-28: Device probe via adb: SM-S921B (Exynos 2400), REALTIME timestamps, 60 fps, manual sensor, OIS off-able, IMU 500 Hz, HID device enabled.
- 2026-09-28: Installed toolchain (JDK 21, NDK 29, CMake 3.31.6, platform 36). Scope widened to any ceiling/desk. Git initialised, remote = GitHub MostMobileMouse.
