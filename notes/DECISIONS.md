# Decisions

Record decisions that deviate from or refine DESIGN.md, with date and reason.

- 2026-09-28: Project split into `Phone-part/` and `PC-part/` instead of DESIGN.md §4's single `desk-mouse/` tree. C++ core lives in `PC-part/core/` and is referenced from the Android build. Reason: user's existing folder structure.
- 2026-09-28: **Scope widened: must work on any ceiling and any desk** smooth enough not to scratch the phone (user requirement). DESIGN.md §1 lists "arbitrary surfaces without a ceiling in view" as a non-goal; this pushes against it. Consequences:
  - Recordings at several locations, including the plainest ceiling available (plans/RECORDINGS_NEEDED.md).
  - Texture-poor ceilings become a first-class case, not a risk footnote. Levers to evaluate in M1/M2: ultra-wide camera (more of the room in view), higher capture resolution / centre crop, longer exposure + CLAHE, lamps and room edges as features, and a quality-graded IMU+ZUPT fallback.
  - Physical limit, stated up front: a perfectly uniform, evenly lit ceiling with nothing else in view gives the camera nothing to track. Then only the IMU+ZUPT mode is available, and M2 measures how usable it is.
  - Desk surface matters mainly for taps (hard vs soft), stick-slip sliding, and flatness, so test on hard, glass and soft (mousepad/cloth) surfaces.
- 2026-09-28: **Multi-depth front end — APPROVED by user 2026-09-28** (from Tier 0 place 1, notes/TIER0_RESULTS.md). Real desks have shelves, monitors and hutches in view at other depths than the ceiling; only ~45 % of points fit one motion. Change to DESIGN.md §6.2 step 6 and §6.3:
  - Key property: the phone moves in a plane, so after gyro de-rotation every static point's image flow is **parallel** (direction set by the phone's velocity) and only its **magnitude** scales with that point's inverse depth ρ_i. The direction is estimated from all points; the speed needs depth.
  - Give each persistent track its own inverse depth ρ_i, estimated from how its flow scales against IMU-predicted velocity. Measurement = all tracks' flows, each with its own ρ_i. The single ρ state becomes a reference ρ plus per-track ratios (or a small set of depth clusters), keeping the EKF small.
  - Outliers (moving head, monitor content) = tracks whose flow isn't parallel to the consensus or whose ratio isn't stable over time.
  - The synthetic simulator (M1) must render multi-depth scenes (ceiling + boxes/shelves at 0.3–1.5 m) and moving distractors, not just one textured plane.
- 2026-09-28: Toolchain: Temurin JDK 21, Android cmdline-tools (new `android` CLI), platform android-36, build-tools 36.1.0, NDK 29.0.14206865, CMake 3.31.6 (SDK's), platform-tools 37.0.1. ANDROID_HOME set, adb on user PATH.
- 2026-09-28: GitHub repo https://github.com/leo-01000111/MostMobileMouse (public). Recordings and tier0 photos are gitignored (they show the user's home).
- 2026-09-28: User allows pushing to origin/main without asking.
