"""Run the MVP engine on a recording in arrival order and report cursor path, clicks and scrolls.

usage: python scripts/mvp_replay.py <recording> [--plot] [--latency-ms 45] [--mount-yaw 0] [--tuned]

Labels cal_begin / cal_end (Bluetooth mouse session recordings) start and end a calibration run, as on the phone.
"""
import argparse
import heapq
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from deskmouse.engine import Engine, EngineConfig  # noqa: E402
from deskmouse.io import load  # noqa: E402


def run(rec, cfg: EngineConfig, latency_ms: float = 45.0, gauge_ratio: float | None = None):
    K, dist = rec.intrinsics()
    eng = Engine(K, dist, (rec.width, rec.height), cfg, gauge_ratio=gauge_ratio)
    L = rec.labels
    cal = sorted((int(t), lab) for t, lab in zip(L.t_ns, L.label) if lab in ("cal_begin", "cal_end")) if len(L) else []
    imu = rec.imu[rec.imu["type"].isin(["gyro", "accel"])].sort_values("t_ns")
    f = rec.frames
    frames = iter(rec.frames_y())
    # arrival time of frame k = sensor time + exposure/2 + latency
    arr = (f.t_ns + f.exposure_ns.fillna(0) // 2 + latency_ms * 1e6).to_numpy()
    k = 0
    events = []
    for t, kind, x, y, z in imu[["t_ns", "type", "x", "y", "z"]].itertuples(index=False):
        while k < len(arr) and arr[k] <= t:
            _, img = next(frames)
            row = f.iloc[k]
            eng.on_frame(img, int(row.t_ns), int(row.exposure_ns or 0), int(row.rolling_shutter_skew_ns or 0))
            k += 1
        while cal and cal[0][0] <= t:
            eng.begin_calibration() if cal.pop(0)[1] == "cal_begin" else eng.end_calibration()
        eng.on_imu(int(t), kind, x, y, z)
        eng.poll(int(t))
        events += eng.drain()
    return eng, events


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--latency-ms", type=float, default=45.0)
    ap.add_argument("--mount-yaw", type=float, default=0.0)
    ap.add_argument("--tuned", action="store_true", help="use PC-part/game/phone_params.json like the live mouse")
    a = ap.parse_args()
    rec = load(a.recording)
    tuned = json.loads((Path(__file__).resolve().parents[2] / "game/phone_params.json").read_text()) if a.tuned else {}
    kw = {k: v for k, v in tuned.items() if k in EngineConfig.__dataclass_fields__}
    kw.setdefault("mount_yaw_deg", a.mount_yaw)
    cfg = EngineConfig(**kw)
    eng, ev = run(rec, cfg, a.latency_ms, tuned.get("gauge_ratio"))
    moves = np.array([(e.t_ns, e.dx, e.dy) for e in ev if e.kind == "move"]) if any(e.kind == "move" for e in ev) else np.zeros((0, 3))
    clicks = [(e.t_ns, e.kind) for e in ev if e.kind in ("left", "right")]
    wheel = sum(e.wheel for e in ev if e.kind == "wheel")
    wheel_abs = sum(abs(e.wheel) for e in ev if e.kind == "wheel")
    L = rec.labels
    labels = L[L.label.str.endswith("_down")] if len(L) else L
    path = np.cumsum(moves[:, 1:], 0) if len(moves) else np.zeros((1, 2))
    out = {"recording": rec.path.name, "frames": eng.stats["frames"], "taps": eng.stats["taps"],
           "left": sum(1 for c in clicks if c[1] == "left"), "right": sum(1 for c in clicks if c[1] == "right"),
           "labels_down": len(labels), "wheel_net": wheel, "wheel_abs": wheel_abs, "lifts": eng.stats["lifts"],
           "scale_updates": eng.stats["scale_updates"], "lam": eng.lam,
           "cursor_extent_px": (path.max(0) - path.min(0)).tolist(), "move_events": len(moves),
           "cursor_final_px": path[-1].tolist()}
    print(json.dumps(out, indent=1))
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(12, 5))
        ax[0].plot(path[:, 0], -path[:, 1], lw=0.7)
        ax[0].set_aspect("equal"); ax[0].set_title(f"{rec.path.name}: cursor path (px, up = forward)", fontsize=9)
        t0 = rec.frames.t_ns.iloc[0]
        if len(moves):
            ax[1].plot((moves[:, 0] - t0) / 1e9, path[:, 0], lw=0.7, label="cursor x")
            ax[1].plot((moves[:, 0] - t0) / 1e9, -path[:, 1], lw=0.7, label="cursor up")
        for tc, kind in clicks:
            ax[1].axvline((tc - t0) / 1e9, color="g" if kind == "left" else "r", lw=0.8)
        for tl in labels.t_ns if len(labels) else []:
            ax[1].axvline((tl - t0) / 1e9, color="k", lw=0.4, ls=":")
        ax[1].legend(fontsize=8); ax[1].set_xlabel("s  (green/red = left/right clicks, dotted = volume labels)")
        plt.tight_layout()
        out_dir = Path(__file__).resolve().parents[1] / "out" / "mvp"
        out_dir.mkdir(parents=True, exist_ok=True)
        plt.savefig(out_dir / f"{rec.path.name}.png", dpi=75)


if __name__ == "__main__":
    main()
