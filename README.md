# MostMobileMouse

A phone lying face-down on a desk, used as a mouse. The rear camera looks at the ceiling and tracks features on it. The gyroscope and accelerometer provide the scale and detect stillness, taps and lifts. The Android app is Kotlin; the PC side is Python.

Status: work in progress. It is a Windows-only prototype that runs on one phone model.

## What works today

- A recorder app for the Samsung Galaxy S24 (the only phone tested) that logs camera frames and IMU data for offline development.
- A "Mouse mode" in the same app that streams frames and IMU data to a PC.
- A Python engine on the PC that tracks the ceiling, turns the motion into cursor movement and moves the Windows cursor.
- Clicks: a tap on the back of the phone is a left click, a double tap is a right click.
- An aim-lab game that compares the phone with a real mouse and auto-tunes some engine settings.

In the first aim-lab sessions the phone was still clearly behind a real mouse. Time per target was 3.44 s in round 1 and 1.13 s in round 4, against 0.80 s with a mouse. Trace accuracy (% of time inside the target) went 29, 69 and 92 for the same three runs. Median latency from mid-exposure to the frame arriving on the PC is about 51 ms. Details are in `notes/MVP.md`, which also says how each number was measured.

## How it works

The phone only captures. It streams over TCP to the PC, which does all the processing and drives the Windows cursor.

Phone (Kotlin, Camera2): 640x480 grey frames (the Y plane) at 60 fps with a 2 ms manual exposure and OIS/EIS off, plus gyro and accelerometer at 500 Hz. In Mouse mode the app runs a TCP server on port 47475 that sends frames and IMU samples.

Transport:

- USB, via `adb forward tcp:47475 tcp:47475`. Frames are sent raw.
- Wi-Fi. The PC connects to the IP the app shows. Frames are sent as JPEG (quality 90), because the raw stream needs about 150 Mbit/s and Wi-Fi gave about 63 Mbit/s here.

Bluetooth is not used for streaming; it is far too slow for this.

PC engine (`PC-part/proto/deskmouse`):

- Vision front end: Lucas-Kanade tracks with a gyro-predicted start, a forward-backward check and gyro de-rotation, then a RANSAC fit of the phone's translation. Each track gets its own inverse depth, because a real desk has shelves and monitors in view as well as the ceiling.
- Scale: after each stroke, the stroke length from the IMU alone (with zero-velocity updates at both ends) recalibrates the scale. The scale is saved to `PC-part/receiver/state.json` and reused on the next start.
- IMU events: stillness, lift (tilt above 4 degrees or a sudden scale change) and taps (accelerometer jerk).
- Output: body-aligned like a normal mouse, with speed-dependent gain and 1-euro smoothing, sent through Windows `SendInput`.

The vision and IMU data are not fused in a filter yet. An error-state EKF that does this is the next planned step.

## Repository layout

```
Phone-part/             Gradle project (opens in Android Studio)
  recorder/             Kotlin app: recorder plus "Mouse mode" streaming (StreamServer.kt)
PC-part/
  proto/deskmouse/      Python engine: front end, visual odometry, engine, simulator, metrics, config
  proto/scripts/        pull and validate recordings, replay, simulate, offline MVP replay
  receiver/mvp_live.py  live receiver: stream -> engine -> Windows cursor
  game/                 aim-lab game and auto-tuner
  core/                 empty, reserved for a C++ core
notes/, plans/          status, device facts, results, decisions, plans
DESIGN.md               design spec
recordings/             raw recordings (not committed, see recordings/README.md)
```

## Requirements

Phone:

- An Android phone with Camera2 manual sensor control. Only the Samsung Galaxy S24 (Android 16) has been tested, and the camera geometry in `PC-part/proto/deskmouse/engine.py` was measured on it.
- minSdk 31, and USB debugging enabled for the USB path.

Build machine:

- JDK 21 (`JAVA_HOME`), the Android SDK with platforms 36 and 37 (compileSdk 37, targetSdk 36), and `local.properties` pointing at the SDK. The Gradle wrapper (Gradle 9.8) fetches the rest.

PC:

- Windows, since cursor output uses `SendInput`. Developed on Python 3.12; other versions are untested.
- There is no requirements file yet. The code imports numpy, opencv-python, scipy, pandas, pillow and, for the game only, pygame.
- `adb` (Android platform-tools) on PATH for the USB path.

## Build and run

Android app:

```
cd Phone-part
gradlew.bat :recorder:installDebug
```

Live mouse:

1. Put the phone face-down on the desk with its top edge pointing away from you, and open the app.
2. Tap "Mouse mode: stream to PC (USB or Wi-Fi)".
3. On the PC, run one of:

```
python PC-part/receiver/mvp_live.py                  # USB
python PC-part/receiver/mvp_live.py --host PHONE_IP  # Wi-Fi, same network
```

Options: `--dry-run` prints clicks without moving the cursor, `--mount-yaw DEG` corrects for a rotated phone, `--dpi` sets the resolution (default 800), `--world-aligned` switches the output frame, `--scroll` turns on twist-to-scroll (off by default), and `--save FILE` stores the raw stream. Stop with Ctrl+C or by pressing both volume keys on the phone.

Aim-lab game:

```
python PC-part/game/aimlab.py --input mouse   # baseline with a real mouse
python PC-part/game/aimlab.py --input phone [--host PHONE_IP]
```

After a phone session the tuner updates `PC-part/game/phone_params.json`. See `PC-part/game/README.md`.

Recording data for offline work (the recorder, not Mouse mode):

```
python PC-part/proto/scripts/pull_recordings.py
python PC-part/proto/scripts/validate_recording.py recordings/
python PC-part/proto/scripts/mvp_replay.py recordings/<rec> --plot
```

`Phone-part/recorder/README.md` explains how to use the recorder.

## Known limitations

- Windows and one phone model only, and the PC does all the processing.
- The scale is off until the first few strokes have been made.
- Cursor directions assume the phone's top edge points away from you; use `--mount-yaw` otherwise.
- The lens offset is an estimate from the spec, so twisting the phone leaks some cursor motion. This is why twist-to-scroll is off by default.
- No drag, and no left or right click by tap position.
- Latency prediction is off because it made the cursor jittery.
- Some recordings are still missing (lift, free movement, dark room, lights on and off); see `plans/RECORDINGS_NEEDED.md`.

## Next steps

- An EKF that fuses the IMU with the per-track vision measurements.
- A C++ core (OpenCV, Eigen) that runs on the phone, so only cursor events cross the link.
- Bluetooth HID, so the phone pairs as an ordinary mouse with no PC software.
