"""Python ↔ C++ parity (M3): run the same recording through the Python engine and the C++ core and compare.

The C++ side is PC-part/core/tools/replay.cpp built for the phone (arm64) by the recorder's Gradle build; it runs
on the phone over adb, so no desktop C++ toolchain is needed. Inputs are fed to both in the same arrival order as
mvp_replay.py, with decoded frames written to the dump so both see identical pixels.

usage: python scripts/parity.py recordings/<rec> [more recs] [--seconds 20] [--tuned]

Pass criteria (notes/DECISIONS.md 2026-10-05; DESIGN.md §10 asks for 1 % of path length):
  1. engine exact: the C++ engine fed the Python front-end results emits identical events;
  2. closed loop: cursor path error within max(1 %, 1.5 × the noise floor), where the noise floor is Python vs
     Python on frames with 2 % of pixels changed by one grey level (the MVP's scale bootstrap amplifies tiny
     differences; ARM vs x86 OpenCV LK differs by ~1e-3 px);
  3. identical clicks (±5 ms).
"""
import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile
from dataclasses import fields
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from deskmouse.config import FrontEndConfig  # noqa: E402
from deskmouse.engine import Engine, EngineConfig  # noqa: E402
from deskmouse.io import load  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
ADB = str(Path(os.environ.get("LOCALAPPDATA", "")) / "Android/Sdk/platform-tools/adb.exe")
PHONE_DIR = "/data/local/tmp/deskmouse"
ENV = dict(os.environ, MSYS_NO_PATHCONV="1")


def find_build() -> tuple[Path, Path, Path]:
    """deskmouse_replay, libc++_shared.so and libopencv_java4.so from the recorder's native build."""
    rec = ROOT / "Phone-part/recorder/build"
    exe = next(rec.glob("intermediates/cxx/*/*/obj/arm64-v8a/deskmouse_replay"))
    libs = rec / "intermediates/merged_native_libs/debug/mergeDebugNativeLibs/out/lib/arm64-v8a"
    if not libs.exists():
        libs = next(rec.glob("intermediates/merged_native_libs/**/arm64-v8a"))
    return exe, libs / "libc++_shared.so", libs / "libopencv_java4.so"


def engine_params(cfg: EngineConfig) -> dict[str, float]:
    """Engine config as name → number for the dump (C++ set_param names)."""
    out = {}
    for f in fields(cfg):
        v = getattr(cfg, f.name)
        if f.name == "M_ci":
            out.update({f"M_ci.{i}": float(x) for i, x in enumerate(np.ravel(v))})
        elif f.name == "r_cam":
            out.update({f"r_cam.{i}": float(x) for i, x in enumerate(v)})
        else:
            out[f.name] = float(v)
    for f in fields(FrontEndConfig):
        if f.name not in ("rot_sign", "clock_offset_ns"):
            out["fe." + f.name] = float(getattr(FrontEndConfig(), f.name))
    return out


def run(rec, cfg: EngineConfig, ratio: float, seconds: float, dump_path: Path, latency_ms: float = 45.0,
        results_path: Path | None = None):
    """Feeds the Python engine and writes the dump in the same order. Returns the Python log lines.
    results_path: also write a dump with the Python front-end results in place of frames (exact engine check)."""
    K, dist = rec.intrinsics()
    eng = Engine(K, dist, (rec.width, rec.height), cfg, gauge_ratio=ratio or None)
    eng.frame_log = []
    imu = rec.imu[rec.imu["type"].isin(["gyro", "accel"])].sort_values("t_ns")
    t_end = imu.t_ns.iloc[0] + seconds * 1e9
    imu = imu[imu.t_ns <= t_end]
    f = rec.frames
    frames = iter(rec.frames_y())
    arr = (f.t_ns + f.exposure_ns.fillna(0) // 2 + latency_ms * 1e6).to_numpy()
    log = []

    def events():
        for e in eng.drain():
            log.append(f"E {e.t_ns} {e.kind} {e.dx} {e.dy} {e.wheel}")

    dist5 = (list(dist) + [0] * 5)[:5]
    header = b"DMRP" + struct.pack("<Iii", 1, rec.width, rec.height)
    header += struct.pack("<9d", *np.ravel(K)) + struct.pack("<5d", *dist5) + struct.pack("<d", ratio or 0.0)
    params = engine_params(cfg)
    header += struct.pack("<I", len(params))
    for name, v in params.items():
        header += struct.pack("<H", len(name)) + name.encode() + struct.pack("<d", v)
    res_out = open(results_path, "wb") if results_path else None
    if res_out:
        res_out.write(header)
    with open(dump_path, "wb") as out:
        out.write(header)
        k = 0
        for t, kind, x, y, z in imu[["t_ns", "type", "x", "y", "z"]].itertuples(index=False):
            while k < len(arr) and arr[k] <= t:
                _, img = next(frames)
                row = f.iloc[k]
                ts, exp, skew = int(row.t_ns), int(row.exposure_ns or 0), int(row.rolling_shutter_skew_ns or 0)
                out.write(struct.pack("<Bqqq", 1, ts, exp, skew) + np.ascontiguousarray(img).tobytes())
                eng.on_frame(img, ts, exp, skew)
                if res_out:
                    t_, pp, p, wx, wy, sg, valid, nr, r15, rmed, _ = eng.frame_log[-1]
                    res_out.write(struct.pack("<Bqddddd", 2, t_, pp, p, wx, wy, sg) + struct.pack("<Bidd", int(valid), nr, r15, rmed))
                log.append(None)  # placeholder, filled from the frame hook below
                events()
                k += 1
            rec_imu = struct.pack("<BqBddd", 0, int(t), 0 if kind == "gyro" else 1, float(x), float(y), float(z))
            out.write(rec_imu)
            if res_out:
                res_out.write(rec_imu)
            eng.on_imu(int(t), kind, x, y, z)
            eng.poll(int(t))
            events()
    if res_out:
        res_out.close()
    return eng, log


def hook_frames():
    """Wraps FrontEnd.process so each frame's result is logged in the C++ 'F' format."""
    from deskmouse import frontend
    orig = frontend.FrontEnd.process
    sink: list = []

    def process(self, y, t_ns, dpsi):
        r = orig(self, y, t_ns, dpsi)
        sink.append(f"F {r.t_ns} {int(r.valid)} {r.w[0]:.9g} {r.w[1]:.9g} {r.sigma:.9g} {r.n_tracks} {r.n_inliers}")
        return r
    frontend.FrontEnd.process = process
    return sink


def parse(lines):
    F, E = [], []
    for ln in lines:
        p = ln.split()
        if p[0] == "F":
            F.append((int(p[1]), int(p[2]), float(p[3]), float(p[4]), int(p[7])))
        elif p[0] == "E":
            E.append((int(p[1]), p[2], int(p[3]), int(p[4]), int(p[5])))
    return F, E


def path_at(E, ts):
    """Cumulative cursor position (px) after all moves up to each time in ts."""
    mv = np.array([(t, dx, dy) for t, k, dx, dy, _ in E if k == "move"], float).reshape(-1, 3)
    if not len(mv):
        return np.zeros((len(ts), 2)), 0.0
    cum = np.cumsum(mv[:, 1:], 0)
    idx = np.searchsorted(mv[:, 0], ts, side="right") - 1
    pos = np.where(idx[:, None] >= 0, cum[np.clip(idx, 0, None)], 0)
    length = float(np.linalg.norm(mv[:, 1:], axis=1).sum())
    return pos, length


def compare(py_lines, cpp_lines):
    Fp, Ep = parse(py_lines)
    Fc, Ec = parse(cpp_lines)
    n = min(len(Fp), len(Fc))
    both = [(a, b) for a, b in zip(Fp[:n], Fc[:n]) if a[1] and b[1]]
    valid_agree = sum(a[1] == b[1] for a, b in zip(Fp[:n], Fc[:n])) / max(n, 1)
    wp = np.array([[a[2], a[3]] for a, _ in both]).reshape(-1, 2)
    wc = np.array([[b[2], b[3]] for _, b in both]).reshape(-1, 2)
    w_rel = float(np.median(np.linalg.norm(wp - wc, axis=1) / np.maximum(np.linalg.norm(wp, axis=1), 1e-9))) if len(wp) else 0.0
    ts = np.array([a[0] for a in Fp[:n]], float)
    pp, Lp = path_at(Ep, ts)
    pc, Lc = path_at(Ec, ts)
    err = float(np.linalg.norm(pp - pc, axis=1).max()) if n else 0.0

    def clicks(E):
        return [(t, k) for t, k, *_ in E if k in ("left", "right")]

    def wheel(E):
        return sum(w for *_, w in E)
    cp, cc = clicks(Ep), clicks(Ec)
    matched = sum(any(k == k2 and abs(t - t2) <= 5e6 for t2, k2 in cc) for t, k in cp)
    return {
        "frames": [len(Fp), len(Fc)], "valid_agree": round(valid_agree, 4), "w_median_rel_diff": round(w_rel, 5),
        "path_len_px": [round(Lp), round(Lc)], "path_max_err_px": round(err, 1),
        "path_err_pct": round(100 * err / max(Lp, 1), 3),
        "clicks": [len(cp), len(cc)], "clicks_matched": matched, "wheel": [wheel(Ep), wheel(Ec)],
    }


def noise_floor(rec, cfg, ratio, seconds, frame_log) -> float:
    """Path error (%) of Python vs Python with frames dithered by one grey level on 2 % of pixels."""
    from deskmouse.io import Recording
    orig = Recording.frames_y

    def noisy(self, *a, **k):
        rng = np.random.default_rng(7)
        for i, img in orig(self, *a, **k):
            m = rng.random(img.shape) < 0.02
            yield i, np.clip(img.astype(np.int16) + m * rng.choice([-1, 1], img.shape), 0, 255).astype(np.uint8)
    logs = []
    try:
        for fy in (orig, noisy):
            Recording.frames_y = fy
            frame_log.clear()
            _, log = run(rec, cfg, ratio, seconds, Path(os.devnull))
            it = iter(frame_log)
            logs.append([next(it) if ln is None else ln for ln in log])
    finally:
        Recording.frames_y = orig
    return compare(*logs)["path_err_pct"]


def adb(*args, **kw):
    return subprocess.run([ADB, *args], env=ENV, check=True, capture_output=True, text=True, **kw).stdout


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recordings", nargs="+")
    ap.add_argument("--seconds", type=float, default=20.0, help="replay only the start (the dump holds raw frames)")
    ap.add_argument("--tuned", action="store_true", help="use PC-part/game/phone_params.json like the live mouse")
    ap.add_argument("--keep", help="write both logs (py.txt, cpp.txt) to this folder")
    a = ap.parse_args()

    tuned = {}
    if a.tuned:
        tuned = json.loads((ROOT / "PC-part/game/phone_params.json").read_text())
    kw = {k: v for k, v in tuned.items() if k in EngineConfig.__dataclass_fields__}
    ratio = float(tuned.get("gauge_ratio", 0.0))

    exe, cxx, cv = find_build()
    adb("shell", f"mkdir -p {PHONE_DIR}")
    for p in (exe, cxx, cv):
        adb("push", str(p), f"{PHONE_DIR}/")
    adb("shell", f"chmod 755 {PHONE_DIR}/deskmouse_replay")

    results = {}
    frame_log = hook_frames()
    for r in a.recordings:
        rec = load(r)
        with tempfile.TemporaryDirectory() as td:
            dump = Path(td) / "in.bin"
            dump_res = Path(td) / "res.bin"
            frame_log.clear()
            eng, log = run(rec, EngineConfig(**kw), ratio, a.seconds, dump, results_path=dump_res)
            # merge frame lines into the placeholders, in order
            it = iter(frame_log)
            py_lines = [next(it) if ln is None else ln for ln in log]
            adb("push", str(dump), f"{PHONE_DIR}/in.bin")
            timing = adb("shell", f"cd {PHONE_DIR} && LD_LIBRARY_PATH=. ./deskmouse_replay in.bin out.txt").strip()
            out = Path(td) / "out.txt"
            adb("pull", f"{PHONE_DIR}/out.txt", str(out))
            cpp_lines = out.read_text().splitlines()
            # exact engine check: C++ engine fed the Python front-end results
            adb("push", str(dump_res), f"{PHONE_DIR}/res.bin")
            adb("shell", f"cd {PHONE_DIR} && LD_LIBRARY_PATH=. ./deskmouse_replay res.bin out2.txt")
            adb("pull", f"{PHONE_DIR}/out2.txt", str(out))
            eng_lines = out.read_text().splitlines()
            adb("shell", f"rm -f {PHONE_DIR}/in.bin {PHONE_DIR}/res.bin {PHONE_DIR}/out.txt {PHONE_DIR}/out2.txt")
        if a.keep:
            k = Path(a.keep) / rec.path.name
            k.mkdir(parents=True, exist_ok=True)
            (k / "py.txt").write_text("\n".join(py_lines))
            (k / "cpp.txt").write_text("\n".join(cpp_lines))
        res = compare(py_lines, cpp_lines)
        py_ev = [ln for ln in py_lines if ln.startswith("E ")]
        cpp_ev = [ln for ln in eng_lines if ln.startswith("E ")]
        res["engine_exact"] = {"events": [len(py_ev), len(cpp_ev)], "identical": py_ev == cpp_ev,
                               "first_diff": next((i for i, (x, y) in enumerate(zip(py_ev, cpp_ev)) if x != y), None)}
        res["noise_floor_pct"] = noise_floor(rec, EngineConfig(**kw), ratio, a.seconds, frame_log)
        res["phone_timing"] = timing
        res["py_stats"] = {k: eng.stats[k] for k in ("taps", "scale_updates", "lifts")}
        res["cpp_stats"] = next((ln for ln in cpp_lines if ln.startswith("S ")), "")
        results[rec.path.name] = res
        print(rec.path.name, json.dumps(res, indent=1), flush=True)
    ok = all(v["engine_exact"]["identical"] and v["path_err_pct"] <= max(1.0, 1.5 * v["noise_floor_pct"])
             and v["clicks"][0] == v["clicks_matched"] == v["clicks"][1] for v in results.values())
    print("PARITY", "PASS" if ok else "FAIL")


if __name__ == "__main__":
    main()
