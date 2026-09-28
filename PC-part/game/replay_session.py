"""Replay a phone aim-lab session through the engine with different settings (offline evaluation).

usage: python PC-part/game/replay_session.py [SESSION_DIR]    → latency-compensation evaluation
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "proto"))
from deskmouse.engine import Engine, EngineConfig  # noqa: E402
from deskmouse.io import intrinsics_from  # noqa: E402


def replay(d: Path, overrides: dict) -> np.ndarray:
    """Cursor path (arrival_pc_ns, x, y) in counts, driven by logged IMU + front-end results in arrival order."""
    imu = np.genfromtxt(d / "phone_imu.csv", delimiter=",", names=True)
    pf = np.genfromtxt(d / "phone_frames.csv", delimiter=",", names=True)
    h = json.loads((d / "phone_hello.json").read_text())
    params = json.loads((d / "session.json").read_text()).get("params") or {}
    kw = {k: v for k, v in {**params, **overrides}.items() if k in EngineConfig.__dataclass_fields__}
    K, dist = intrinsics_from(h["camera_characteristics"], h["width"])
    eng = Engine(K, dist, (h["width"], h["height"]), EngineConfig(instant_left=True, **kw), gauge_ratio=params.get("gauge_ratio"))
    items = [(r["arrival_pc_ns"], 0, r) for r in imu] + [(r["arrival_pc_ns"], 1, r) for r in pf]
    items.sort(key=lambda x: (x[0], x[1]))
    pos = np.zeros(2)
    out = []
    for arr, k, r in items:
        if k == 0:
            eng.on_imu(int(r["t_phone_ns"]), "gyro" if r["kind"] == 0 else "accel", r["x"], r["y"], r["z"])
            eng.poll(int(r["t_phone_ns"]))
        else:
            eng.on_result(int(r["t_ns"]), r["psi_prev"], r["psi"], r["wx"], r["wy"], r["sigma"], bool(r["valid"]),
                          int(r["n_rhos"]), r["rho15"], r["rho_med"], int(r["n_inliers"]))
        for e in eng.drain():
            if e.kind == "move":
                pos += (e.dx, e.dy)
        if k == 1:
            out.append((arr, pos[0], pos[1]))
    return np.array(out)


def evaluate(d: Path, latency_ms: float):
    base = replay(d, {"latency_comp_ms": 0.0})
    t = base[:, 0]
    fut = np.stack([np.interp(t + latency_ms * 1e6, t, base[:, i]) for i in (1, 2)], 1)  # where the phone "is now"
    moving = np.linalg.norm(np.gradient(base[:, 1:], axis=0), axis=1) > 0.5
    print(f"session {d.name}: {len(t)} frames, moving {moving.mean() * 100:.0f} %  (target = uncompensated path {latency_ms:.0f} ms later)")
    print(f"{'setting':32s} {'rms err px':>10s} {'p95 px':>8s} {'overshoot px at stops':>22s}")
    for label, ov in [("no compensation", {"latency_comp_ms": 0.0}),
                      ("lead 30 ms, camera only", {"latency_comp_ms": 30.0, "lead_use_imu": False}),
                      ("lead 55 ms, camera only", {"latency_comp_ms": 55.0, "lead_use_imu": False}),
                      ("lead 30 ms, camera + IMU", {"latency_comp_ms": 30.0}),
                      ("lead 55 ms, camera + IMU", {"latency_comp_ms": 55.0}),
                      ("lead 75 ms, camera + IMU", {"latency_comp_ms": 75.0})]:
        p = replay(d, ov)
        e = np.linalg.norm(p[:, 1:] - fut, axis=1)[moving]
        # overshoot: at the end of movements, how far past the final resting point the cursor went
        stop = np.where(moving[:-1] & ~moving[1:])[0]
        ov_px = [np.max(np.linalg.norm(p[max(0, i - 6):i + 1, 1:] - p[min(len(p) - 1, i + 30), 1:], axis=1)) -
                 np.max(np.linalg.norm(base[max(0, i - 6):i + 1, 1:] - base[min(len(p) - 1, i + 30), 1:], axis=1)) for i in stop]
        print(f"{label:32s} {np.sqrt(np.mean(e ** 2)):10.1f} {np.percentile(e, 95):8.1f} {np.median(ov_px) if ov_px else 0:22.1f}")


if __name__ == "__main__":
    d = Path(sys.argv[1]) if len(sys.argv) > 1 else sorted((HERE / "sessions").glob("*_phone"))[-1]
    evaluate(d, float(sys.argv[2]) if len(sys.argv) > 2 else 56.0)
