# CLAUDE.md: Desk Mouse (17_MostMobileMouse)

A Samsung Galaxy phone lying face-down on a desk becomes a mouse: rear camera tracks the ceiling, IMU fills the gaps, an EKF fuses them. Back-tap = click, twist = scroll.

## Read first
- **[DESIGN.md](DESIGN.md)**: the spec. Algorithm, parameters, file formats, milestones. It wins over anything else unless the user says otherwise.
- **[plans/PLAN.md](plans/PLAN.md)**: order of work, phases, who does what.
- **[plans/RECORDINGS_NEEDED.md](plans/RECORDINGS_NEEDED.md)**: what the user records and how.
- **[notes/STATUS.md](notes/STATUS.md)**: current milestone and progress log. Update it when a milestone step finishes.

## Layout
```
DESIGN.md                spec (user-authored)
plans/                   implementation plan, recording requests
notes/                   STATUS.md (progress), DEVICE.md (phone/desk facts), DECISIONS.md, M2_REPORT.md later
recordings/              user recordings, rec_YYYYMMDD_HHMMSS_<tag>/ (large; never commit; tier0/ for stock-camera checks)
Phone-part/recorder/     M0 recorder app (Kotlin, Compose, Camera2)
Phone-part/app/          M4+ real-time mouse app
PC-part/proto/           M1–M2 Python reference implementation + replay/evaluate scripts
PC-part/core/            M3 C++17 core (OpenCV + Eigen, pybind11); Android CMake references it by relative path
PC-part/receiver/        UDP receiver (pynput)
```
This split replaces DESIGN.md §4's `desk-mouse/` layout (mapping in plans/PLAN.md §5).

## Rules of the road
- **Data-first, milestone-gated.** Follow DESIGN.md §11 order. Don't start a milestone until the previous "done when" is met.
- **Hard stop after M2:** write `notes/M2_REPORT.md`, recommend GO/NO-GO, and wait for the user's explicit decision before any C++ core or real-time Android work.
- Every numeric value in DESIGN.md §6 is a **default config parameter** in both Python and C++. No hard-coded magic numbers.
- The synthetic simulator writes the **same directory format** as the recorder, so all tools run on both.
- Python and C++ must implement the same algorithm; parity is tested in M3.
- Never modify files in `recordings/`; derived outputs go to `PC-part/proto/out/`.

## Commands
- Build/install recorder: `cd Phone-part && gradlew.bat :recorder:installDebug` (JAVA_HOME = Temurin 21). compileSdk 37 (Compose needs it), targetSdk 36.
- Pull + validate recordings: `python PC-part/proto/scripts/pull_recordings.py`
- Validate: `python PC-part/proto/scripts/validate_recording.py recordings/`
- Tier 0 stock-camera check: `python PC-part/proto/scripts/tier0_check.py recordings/tier0/<place>`
- MVP live mouse: `python PC-part/receiver/mvp_live.py [--dry-run] [--mount-yaw 90]`, then "Mouse mode" on the phone (notes/MVP.md)
- MVP replay on a recording: `python PC-part/proto/scripts/mvp_replay.py recordings/<rec> --plot`
- Aim-lab game: `python PC-part/game/aimlab.py --input mouse|phone|bot` (PC-part/game/README.md); tuner writes PC-part/game/phone_params.json
- In Git Bash, prefix adb commands with `MSYS_NO_PATHCONV=1` or phone paths get mangled.

## Target
- Samsung Galaxy S24, One UI 8.5, Android 16 (API 36). Details in notes/DEVICE.md.
- **Must work on any ceiling and any smooth desk** (user requirement, widens DESIGN.md; see notes/DECISIONS.md). Texture-poor ceilings are a main case, not an edge case.

## Environment (set up 2026-09-28)
- Windows 11, PowerShell primary, Git Bash available. Python 3.12.5, CMake 4.2, git, gh. VS 2022 folder present (MSVC not yet verified, needed for M3).
- JDK: Temurin 21 (`JAVA_HOME` set machine-wide). Android SDK: `ANDROID_HOME=%LOCALAPPDATA%\Android\Sdk`, platform android-36, build-tools 36.1.0, NDK 29.0.14206865, CMake 3.31.6, platform-tools on user PATH (new shells only; in old shells use the full path to adb.exe).
- SDK package manager is the new `android` CLI (`cmdline-tools\latest\bin\android.exe --no-metrics sdk install <pkg/ver>`); `sdkmanager` is deprecated. Kill the adb server before updating platform-tools.
- Git remote: https://github.com/leo-01000111/MostMobileMouse (**public**). Never commit recordings or photos of the user's rooms.

## Unknowns to fill in (see notes/DEVICE.md)
Case, lens offset, per-location ceiling notes. (Camera2, IMU and HID facts are in notes/DEVICE.md.)
