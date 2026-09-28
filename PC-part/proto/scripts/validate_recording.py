"""Check recordings against the M0 acceptance criteria (DESIGN.md §11 M0).

usage: python scripts/validate_recording.py <rec_dir> [<rec_dir> ...]
       python scripts/validate_recording.py recordings/          (all rec_* inside)
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from deskmouse.io import load  # noqa: E402


def check(path: Path) -> bool:
    r = load(path)
    m = r.meta
    problems, info = [], []
    n_csv = len(r.frames)
    n_vid = r.video_frame_count()
    if n_vid != n_csv:
        problems.append(f"video has {n_vid} frames, frames.csv has {n_csv}")
    t = r.frames["t_ns"].to_numpy()
    if np.any(np.diff(t) <= 0):
        problems.append("frame timestamps not strictly increasing")
    dt = np.diff(t) / 1e6
    fps = 1e3 / np.median(dt) if len(dt) else 0
    gaps = int(np.sum(dt > 1.5 * np.median(dt))) if len(dt) else 0
    info.append(f"{n_csv} frames, {fps:.1f} fps (median), {gaps} gaps >1.5 frame, dropped={m['measured']['dropped_frames']}")
    if fps < 55 and m["capture"]["requested_fps"] >= 60:
        problems.append(f"camera fps {fps:.1f} < 55")
    for kind in ("gyro", "accel"):
        s = r.imu_of(kind)["t_ns"].to_numpy()
        rate = (len(s) - 1) * 1e9 / (s[-1] - s[0]) if len(s) > 1 else 0
        info.append(f"{kind} {rate:.0f} Hz")
        if rate < 400:
            problems.append(f"{kind} rate {rate:.0f} Hz < 400")
    ts_src = m["camera_characteristics"].get("timestamp_source")
    info.append(f"timestamp_source={ts_src} (1=REALTIME)")
    if ts_src != 1:
        problems.append("camera timestamp source is not REALTIME: needs clock offset estimation")
    st = m["capture"]["applied_state"]
    if st.get("ois_mode") not in (0, None) or st.get("video_stabilization_mode") not in (0, None):
        problems.append(f"stabilisation active: ois={st.get('ois_mode')} vstab={st.get('video_stabilization_mode')}")
    if m["capture"]["exposure_mode"] != "manual":
        problems.append(f"exposure mode {m['capture']['exposure_mode']}")
    imu_t = r.imu["t_ns"]
    overlap = (min(t[-1], imu_t.max()) - max(t[0], imu_t.min())) / 1e9
    info.append(f"camera/IMU overlap {overlap:.1f} s of {m['duration_s']:.1f} s; exposure {m['capture']['applied_exposure_ns']/1e6:.2f} ms ISO {m['capture']['applied_iso']}")
    print(("OK   " if not problems else "FAIL ") + path.name)
    for s in info:
        print("     " + s)
    for s in problems:
        print("  !! " + s)
    return not problems


if __name__ == "__main__":
    paths = []
    for a in sys.argv[1:]:
        p = Path(a)
        paths += sorted(p.glob("rec_*")) if not (p / "meta.json").exists() else [p]
    ok = all([check(p) for p in paths])
    sys.exit(0 if ok else 1)
