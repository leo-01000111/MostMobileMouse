# Decisions

Record decisions that deviate from or refine DESIGN.md, with date and reason.

- 2026-09-28: Project split into `Phone-part/` and `PC-part/` instead of DESIGN.md §4's single `desk-mouse/` tree. C++ core lives in `PC-part/core/` and is referenced from the Android build. Reason: user's existing folder structure.
- 2026-09-28: **Scope widened: must work on any ceiling and any desk** smooth enough not to scratch the phone (user requirement). DESIGN.md §1 lists "arbitrary surfaces without a ceiling in view" as a non-goal; this pushes against it. Consequences:
  - Recordings at several locations, including the plainest ceiling available (plans/RECORDINGS_NEEDED.md).
  - Texture-poor ceilings become a first-class case, not a risk footnote. Levers to evaluate in M1/M2: ultra-wide camera (more of the room in view), higher capture resolution / centre crop, longer exposure + CLAHE, lamps and room edges as features, and a quality-graded IMU+ZUPT fallback.
  - Physical limit, stated up front: a perfectly uniform, evenly lit ceiling with nothing else in view gives the camera nothing to track. Then only the IMU+ZUPT mode is available, and M2 measures how usable it is.
  - Desk surface matters mainly for taps (hard vs soft), stick-slip sliding, and flatness, so test on hard, glass and soft (mousepad/cloth) surfaces.
- 2026-09-28: Toolchain: Temurin JDK 21, Android cmdline-tools (new `android` CLI), platform android-36, build-tools 36.1.0, NDK 29.0.14206865, CMake 3.31.6 (SDK's), platform-tools 37.0.1. ANDROID_HOME set, adb on user PATH.
- 2026-09-28: GitHub repo https://github.com/leo-01000111/MostMobileMouse (public). Recordings and tier0 photos are gitignored (they show the user's home).
