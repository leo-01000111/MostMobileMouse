# Desk Mouse: Implementation Plan

Source of truth for the algorithm and specs: [../DESIGN.md](../DESIGN.md). This file covers **order of work, who does what, and what gets handed over when**. Section numbers (§) refer to DESIGN.md.

Written 2026-09-28. Status: nothing implemented yet.

---

## 0. The short answer: what I need from you before I start

**No recordings block me from starting.** The real recordings come from the recorder app (M0), which I write first. In parallel I build the synthetic simulator (M1), which needs no data at all.

What I *do* need before I start:

| # | Item | Status |
|---|------|--------|
| 1 | Phone model and OS | ✅ Galaxy S24, One UI 8.5, Android 16 |
| 2 | Toolchain (JDK, NDK, CMake, adb on PATH) | ✅ installed 2026-09-28 |
| 3 | USB debugging on + computer authorized | ✅ enabled; authorization prompt to accept on the phone |
| 4 | Tier 0 ceiling check, now at **3–5 places** ([RECORDINGS_NEEDED.md §1](RECORDINGS_NEEDED.md)) | ⏳ do it now |
| 5 | Desk facts | Superseded: target is **any ceiling, any smooth desk**. Per-place notes go with Tier 0 |

**Scope change (2026-09-28):** it must work on any ceiling and any desk smooth enough not to scratch the phone. So texture-poor ceilings get their own workstream (A4 below), and recordings cover several places and desk surfaces. See notes/DECISIONS.md.

After M0 is installed on your phone, I need the **Tier 1 recording session** (about 30-40 minutes, see [RECORDINGS_NEEDED.md §2](RECORDINGS_NEEDED.md#2-tier-1-full-protocol-set-after-m0-needed-for-m1-validation-and-m2-gono-go)). That's the one that feeds the go/no-go decision.

---

## 1. Phase overview

```
 Phase A  (no data needed)          Phase B (your recordings)      Phase C (only after GO)
 ─────────────────────────          ─────────────────────────      ──────────────────────
 A1 recorder app (M0)  ─────────▶  B1 Tier-1 session  ──┐
 A2 python I/O + simulator (M1a)                        ├──▶ B3 M2 fusion + go/no-go ──▶ C1 C++ core (M3)
 A3 vision front end on synth (M1b) ─▶ B2 front end on real data ┘                       C2 Android app + UDP (M4)
                                                                                         C3 clicks/scroll live (M5)
                                                                                         C4 Bluetooth HID (M6)
```

A1 and A2/A3 run in parallel. The only hard dependency on you is the Tier 1 session between A1 and B2.

---

## 2. Phase A: build without data

### A0. Device probe (half a day, folded into M0)
- First screen of the recorder app: a **capabilities dump** that writes `device_caps.json`: camera IDs, hardware level, `MANUAL_SENSOR` support, available fps ranges at 640×480, `SENSOR_INFO_TIMESTAMP_SOURCE`, OIS/EIS modes, intrinsics/distortion, IMU sensor list with max rates, `BluetoothHidDevice` availability.
- You run it once and send me the JSON (or I pull it with adb). This answers the Samsung-specific open questions in §12 before any tuning.

### A1. M0: recorder app (`Phone-part/recorder/`)
- Kotlin + Compose, Camera2, single activity. Implements §5 fully: capture settings, protocol picker, 3 s countdown + beeps, volume keys as labels, live IMU rate / fps / feature count.
- Writes the §5.3 directory format to app-specific external storage (visible over USB as `Android/data/<pkg>/files/recordings/`), plus a share-sheet export.
- A small `pull_recordings` script (`adb pull`) into `recordings/`.
- **Done when** (§11 M0): every protocol loads in Python, frame count = `frames.csv` rows, IMU ≥ 400 Hz, camera ≥ 55 fps (or 30 fps documented), timestamp source recorded.
- Risk I'll handle in code: Samsung may refuse 60 fps at 640×480 with manual exposure. Fallback order: 60 fps manual → 60 fps AE-locked → 30 fps manual → 30 fps AE-locked, recorded in `meta.json`.

### A2. M1a: Python I/O + synthetic simulator (`PC-part/proto/`)
- `deskmouse/io.py`: loader for the §5.3 format (video via OpenCV, `frames.y8` raw mode, CSVs via pandas), timestamp alignment helpers.
- `deskmouse/sim.py`: renders a textured ceiling plane through a pinhole camera for a given planar trajectory (translation + yaw, optional small tilt and lift), plus simulated IMU with noise, bias and 400 Hz sampling. Produces **the same directory format** as the recorder, so every tool works on both.
- Trajectory presets that mirror the protocols: `still`, `ruler_x/y`, `square`, `twist`, `lift`, `taps`.
- Unit tests (§10).

### A3. M1b: vision front end on synthetic data
- §6.2 in full: LK tracking with gyro-predicted initial flow, forward-backward check, undistortion, de-rotation, RANSAC translation, scale check, replenishment, frame quality.
- Vision-only odometry with fixed ρ, `replay.py --only-vision --plot`.
- **Done when**: synthetic tests pass the §7 accuracy targets in vision-only mode (with known ρ).

### A4. Texture-poor ceilings (runs across M0–M2)
The recorder must be able to capture with the **ultra-wide** camera and at **higher resolution** (e.g. 1280×960 or a centre crop), not only main 640×480, so M1/M2 can compare on the same ceiling. Levers to evaluate on the plainest Tier 0/Tier 1 ceiling:
1. Ultra-wide vs main camera (more of the room in view: lamps, walls, cornices, window frames).
2. Resolution / crop vs frame time.
3. Exposure length + CLAHE vs motion blur.
4. Bright blobs (lamps) as features.
5. IMU+ZUPT-only fallback: how usable are short mouse strokes without vision? The camera then only corrects scale when texture comes back.

The M2 report gets a per-location table, so "works everywhere" is measured, not assumed.

## 3. Phase B: real data (needs your Tier 1 session)

### B1. Tier 1 recording session (you)
See [RECORDINGS_NEEDED.md](RECORDINGS_NEEDED.md). I check every recording against the M0 acceptance criteria as soon as it lands and tell you if anything needs a retake.

### B2. M1 on real data
- Run the front end on `ruler_*` and `square`. Verify axis mapping `M_ci` (§6.1) from `ruler_x`/`ruler_y` within 5°.
- Report median inlier count for your ceiling. If < 30, try CLAHE / lamp-blob features before moving on.
- If `SENSOR_INFO_TIMESTAMP_SOURCE ≠ REALTIME`: estimate camera-IMU clock offset from `twist` recordings (§6.3).

### B3. M2: EKF fusion + detectors + go/no-go
- §6.1 (bias, tilt, yaw), §6.3 EKF with delayed updates and ZUPT, §6.4 stillness, §6.5 lift, §6.6 taps, §6.7 scroll, §6.8 output mapping.
- `evaluate.py` produces the §7 metrics table, including IMU-only and vision-only columns.
- **Deliverable to you:** `notes/M2_REPORT.md` with the metrics table, plots, and a clear GO / NO-GO recommendation. If any target fails, the report names the limiting component (ceiling texture, latency, IMU noise, timestamp sync, …) and what would fix it.
- **We decide together before C1.** I won't start Android real-time work without your explicit go.

## 4. Phase C: real-time (only after GO)

| Step | Milestone | Folder | Needs from you |
|------|-----------|--------|----------------|
| C1 | M3: C++ core + pybind11 + parity tests | `PC-part/core/` | Nothing (desktop only). Needs a C++ toolchain: MSVC Build Tools or MinGW; I'll check what's installed then |
| C2 | M4: Android app, `MouseService`, JNI, UDP, `receiver.py` | `Phone-part/app/`, `PC-part/receiver/` | Allow the receiver through Windows Firewall (UDP 47474); set app battery to "Unrestricted"; hands-on test with the manual checklist |
| C3 | M5: taps, click modes, drag, twist-scroll live | same | Hands-on test; possibly a second `taps` + `free` recording with the real app |
| C4 | M6: Bluetooth HID | `Phone-part/app/` | Pair with your PC; the HID check from A0 already tells us if this is possible |
| C5 | M7: optional extras | — | Your call |

## 5. Deviations from DESIGN.md layout

DESIGN.md §4 describes a single `desk-mouse/` repo. This project is split into `Phone-part/` and `PC-part/` instead:

| DESIGN.md | Here |
|-----------|------|
| `android/recorder/` | `Phone-part/recorder/` |
| `android/app/` | `Phone-part/app/` |
| `proto/` | `PC-part/proto/` |
| `core/` | `PC-part/core/` (developed and parity-tested on the PC; the Android CMake build references it by relative path) |
| `receiver/` | `PC-part/receiver/` |
| `recordings/` | `recordings/` (top level; large, not versioned) |

Git: https://github.com/leo-01000111/MostMobileMouse (public). `recordings/` is gitignored. Small synthetic sample recordings for tests can go to git-lfs later; real recordings of the user's rooms stay local.

## 6. Open risks and when they get answered

| Risk (§12) | Answered by | When |
|------------|-------------|------|
| Ceiling texture too weak | Tier 0 photo/video, then M1 inlier counts | Day 1 (Tier 0), B2 |
| Samsung camera: manual exposure/focus, 60 fps, REALTIME timestamps | ✅ answered via adb 2026-09-28: all yes on main and ultra-wide (notes/DEVICE.md). Still verify 60 fps at 640×480 with OIS off in M0 | done / M0 |
| Bluetooth HID supported on the phone | ✅ HID device profile enabled (system property); confirm at runtime in M6 | done / M6 |
| IMU rate ≥ 400 Hz | ✅ LSM6DSV, 500 Hz max | done |
| Latency | M2 estimate, M4 measurement | B3, C2 |
| Power / thermal | M4 measurement | C2 |
