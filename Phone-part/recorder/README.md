# DeskMouse Recorder (M0)

Records camera + IMU for offline development (DESIGN.md §5). What to record: [plans/RECORDINGS_NEEDED.md](../../plans/RECORDINGS_NEEDED.md).

## Build and install
```
cd Phone-part
gradlew.bat :recorder:installDebug
```
(JDK 21 via `JAVA_HOME`, SDK path in `local.properties`, not committed.)

## Using it
1. Open the app face-up. The green line shows live camera fps, IMU rates, a ceiling feature count and saturation; the preview shows what the camera sees.
2. Pick the **protocol**, check the duration, and type the **place / desk surface** (e.g. `desk1 mousepad`).
3. Camera: default **id 0 · 1×** (main). `id 2` = ultra-wide opened directly; `id 0 · 0.6×` = ultra-wide via the logical camera. 640×480 @ 60 fps, 2 ms exposure unless told otherwise.
4. Tap **Record**: 3 beeps (put the phone face-down now), 1 s silent exposure metering (hands off), a long beep = recording. Stops automatically after the duration with a double beep.
5. During a take the screen is black and ignores touches. **Vol-Up / Vol-Down** are labels (pressed and released are both logged). **Both together** = stop early.

Files: `Android/data/eu.leongorecki.deskmouse.recorder/files/recordings/rec_YYYYMMDD_HHMMSS_<tag>/` (meta.json, imu.csv, frames.csv, video.mp4 or frames.y8, labels.csv). Pull + validate on the PC:
```
python PC-part/proto/scripts/pull_recordings.py
```
"Save device caps" writes `device_caps.json` (all camera/sensor characteristics).
