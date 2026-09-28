"""Auto-tune the phone mouse from an aim-lab session (called by aimlab.py after a phone session).

usage: python PC-part/game/tune.py [SESSION_DIR]      (default: latest phone session)

What it adjusts (saved to phone_params.json, used by aimlab.py --input phone and mvp_live.py):
  mount_yaw_deg, aspect  from the *ballistic* (first, uncorrected) part of each aiming movement vs the target
                         direction, minus the same measure on the latest mouse session (your own bias).
  tap_thr_min            between the weakest rhythm taps and the strongest non-tap jolts during Trace.
  scroll_omega           above the fastest accidental twist outside the Scroll level.
  still_flow_px          from Precision jitter / timeouts.
It also reports extra lag vs the mouse (Trace) as information.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
SESSIONS = HERE / "sessions"
PARAMS = HERE / "phone_params.json"
DAMP = 0.7


def latest_session(kind: str, exclude: Path | None = None) -> Path | None:
    c = sorted(p for p in SESSIONS.glob(f"*_{kind}") if (p / "session.json").exists() and p != exclude)
    return c[-1] if c else None


def load(d: Path) -> dict:
    s = {"dir": d, "meta": json.loads((d / "session.json").read_text())}
    fr = np.genfromtxt(d / "game_frames.csv", delimiter=",", names=True)
    s["frames"] = fr
    ev = []
    with open(d / "game_events.csv") as f:
        for row in csv.DictReader(f):
            ev.append((int(float(row["t_pc_ns"])), int(row["level"]), row["kind"], row["data"]))
    s["events"] = ev
    s["levels"] = s["meta"]["levels"]
    if (d / "phone_imu.csv").exists():
        s["imu"] = np.genfromtxt(d / "phone_imu.csv", delimiter=",", names=True)
    return s


def level_index(s, name):
    return s["levels"].index(name) if name in s["levels"] else None


# ------------------------------------------------------------------ direction / aspect
def ballistic_pairs(s) -> tuple[np.ndarray, np.ndarray]:
    """(target vectors, primary-submovement vectors) in 'up is +y' coordinates, from click-target levels."""
    fr = s["frames"]
    G, M = [], []
    for name in ("Calibrate", "Targets"):
        li = level_index(s, name)
        if li is None:
            continue
        evs = [e for e in s["events"] if e[1] == li and e[2] in ("spawn", "hit")]
        for a, b in zip(evs[:-1], evs[1:]):
            if not (a[2] == "spawn" and b[2] == "hit"):
                continue
            tgt = json.loads(a[3])
            m = (fr["t_pc_ns"] >= a[0]) & (fr["t_pc_ns"] <= b[0]) & (fr["level"] == li)
            if m.sum() < 10:
                continue
            t = fr["t_pc_ns"][m] / 1e9
            p = np.stack([fr["cx"][m], fr["cy"][m]], 1)
            g = np.array(tgt[:2]) - p[0]
            if np.linalg.norm(g) < 150:
                continue
            # resample to 120 Hz, smooth, find the end of the first submovement
            tt = np.arange(t[0], t[-1], 1 / 120)
            if len(tt) < 8:
                continue
            q = np.stack([np.interp(tt, t, p[:, 0]), np.interp(tt, t, p[:, 1])], 1)
            v = np.linalg.norm(np.gradient(q, axis=0), axis=1) * 120
            v = np.convolve(v, np.ones(5) / 5, mode="same")
            k = int(np.argmax(v))
            if v[k] < 200:
                continue
            end = k + int(np.argmax(v[k:] < 0.3 * v[k])) if np.any(v[k:] < 0.3 * v[k]) else len(v) - 1
            mv = q[end] - q[0]
            if np.linalg.norm(mv) < 50:
                continue
            G.append((g[0], -g[1]))
            M.append((mv[0], -mv[1]))
    return np.array(G), np.array(M)


def fit_distortion(G, M):
    """M ≈ s · diag(1, a) · R(θ) · G   → (θ deg, a, s, residual)."""
    def f(x):
        s, th, a = x
        c, n = math.cos(th), math.sin(th)
        P = G @ np.array([[c, n], [-n, c]])  # rows: R(θ) g
        P = P * np.array([1.0, a]) * s
        return ((P - M) / np.linalg.norm(G, axis=1, keepdims=True)).ravel()
    r = least_squares(f, [1.0, 0.0, 1.0], loss="soft_l1", f_scale=0.3)
    s, th, a = r.x
    return math.degrees(th), a, s, float(np.sqrt(np.mean(r.fun ** 2)))


# ------------------------------------------------------------------ lag
def trace_lag_ms(s) -> float | None:
    li = level_index(s, "Trace")
    if li is None:
        return None
    fr = s["frames"]
    m = (fr["level"] == li) & np.isfinite(fr["tx"])
    if m.sum() < 200:
        return None
    t = fr["t_pc_ns"][m] / 1e9
    t0 = t[0] + 3.5
    tt = np.arange(t0, t[-1], 1 / 120)
    c = np.stack([np.interp(tt, t, fr["cx"][m]), np.interp(tt, t, fr["cy"][m])], 1)
    g = np.stack([np.interp(tt, t, fr["tx"][m]), np.interp(tt, t, fr["ty"][m])], 1)
    best = None
    for lag in range(0, 90):  # up to 750 ms
        e = np.mean(np.sum((c[lag:] - g[:len(g) - lag]) ** 2, 1))
        if best is None or e < best[0]:
            best = (e, lag)
    return 1000 * best[1] / 120


# ------------------------------------------------------------------ phone signals
def phone_clock_offset(s) -> float:
    imu = s["imu"]
    return float(np.percentile(imu["arrival_pc_ns"] - imu["t_phone_ns"], 1))


def level_span_phone(s, li, off):
    fr = s["frames"]
    t = fr["t_pc_ns"][fr["level"] == li]
    return (t.min() - off, t.max() - off) if len(t) else None


def tap_threshold(s, cur: float):
    imu = s["imu"]
    off = phone_clock_offset(s)
    acc = imu[imu["kind"] == 1]
    t = acc["t_phone_ns"][1:]
    j = np.abs(np.diff(acc["z"]))
    li_r, li_t = level_index(s, "Rhythm"), level_index(s, "Trace")
    if li_r is None or li_t is None:
        return None, "tap: need Rhythm and Trace levels"
    t0 = [e[0] for e in s["events"] if e[1] == li_r and e[2] == "level_start"][0]
    notes = []
    for e in s["events"]:
        if e[1] == li_r and e[2] in ("note", "expired"):
            notes.append(json.loads(e[3])[0])
    notes = sorted(set(notes))
    S = []
    for nt in notes:
        tp = t0 + nt * 1e9 - off
        m = (t > tp - 0.25e9) & (t < tp + 0.25e9)
        if m.any():
            S.append(j[m].max())
    # background: every level where nobody taps (Trace, Precision, Scroll)
    m = np.zeros(len(t), bool)
    for name in ("Trace", "Precision", "Scroll"):
        li = level_index(s, name)
        if li is not None:
            span = level_span_phone(s, li, off)
            m |= (t > span[0] + (3.5e9 if name == "Trace" else 0)) & (t < span[1])
    if not S or not m.any():
        return None, "tap: not enough data"
    S = np.array(S)
    bmax = float(j[m].max())
    real = S[S > 3 * max(bmax, 0.05)]  # notes where a clear tap happened
    if len(real) < 8:
        return None, (f"tap: only {len(real)} clear taps in Rhythm (need 8), keeping threshold {cur:.2f}")
    s20 = float(np.percentile(real, 20))
    lo, hi = bmax * 1.3, s20 * 0.7
    thr = math.sqrt(lo * hi) if lo < hi else lo
    thr = float(np.clip(thr, 0.15, 2.0))
    new = cur + DAMP * (thr - cur)
    rec = float(np.mean(S >= new))
    note = f"tap: rhythm taps p20 {s20:.2f}, strongest non-tap jolt (Trace/Precision/Scroll) {bmax:.2f} -> threshold {cur:.2f} -> {new:.2f} (rhythm recall {100 * rec:.0f}%)"
    if lo >= hi:
        note += "  [overlap: favouring no false clicks]"
    return new, note


def scroll_threshold(s, cur: float):
    imu = s["imu"]
    off = phone_clock_offset(s)
    g = imu[imu["kind"] == 0]
    t = g["t_phone_ns"]
    w = np.abs(g["z"] - np.median(g["z"]))
    n = 20  # 40 ms at 500 Hz: sustained = rolling minimum
    if len(w) < n:
        return None, "scroll: not enough data"
    sus = np.lib.stride_tricks.sliding_window_view(w, n).min(1)
    ts = t[n - 1:]
    li_s = level_index(s, "Scroll")
    bg, sc = [], []
    for li, name in enumerate(s["levels"]):
        span = level_span_phone(s, li, off)
        if span is None:
            continue
        m = (ts > span[0]) & (ts < span[1])
        (sc if li == li_s else bg).append(sus[m])
    if not bg or not sc:
        return None, "scroll: need Scroll and other levels"
    bmax = float(np.percentile(np.concatenate(bg), 99.5))  # accidental twists (ignores rare pick-ups)
    sc = np.concatenate(sc)
    p50 = float(np.percentile(sc, 99))  # scroll twists at full speed (most of the level is not twisting)
    lo, hi = max(0.12, bmax * 1.2), p50 * 0.6
    thr = float(np.clip((lo + hi) / 2 if lo < hi else lo, 0.12, 1.0))
    new = cur + DAMP * (thr - cur)
    note = f"scroll: accidental twists up to {bmax:.2f} rad/s (p99.5), scroll twists {p50:.2f} (p99) -> threshold {cur:.2f} -> {new:.2f}"
    if lo >= hi:
        note += "  [overlap: favouring no accidental scroll]"
    return new, note


def run(d: Path, overrides: dict | None = None, params_path: Path | None = None, baseline: str = "mouse") -> list[str]:
    from deskmouse.engine import EngineConfig  # noqa: F401  (path set by caller)
    params_path = params_path or (HERE / "bot_params.json" if d.name.endswith("_bot") else PARAMS)
    overrides = dict(overrides or (json.loads(params_path.read_text()) if params_path.exists() else {}))
    defaults = EngineConfig()
    cur = {k: overrides.get(k, getattr(defaults, k)) for k in ("mount_yaw_deg", "aspect", "dpi", "tap_thr_min", "scroll_omega", "still_flow_px")}
    s = load(d)
    if d.name.endswith("_bot"):  # bot test: baseline = latest undistorted bot session
        base_dir = next((p for p in sorted(SESSIONS.glob("*_bot"), reverse=True)
                         if p != d and json.loads((p / "session.json").read_text()).get("bot_rot", 0) == 0), None)
    else:
        base_dir = latest_session(baseline)
    base = load(base_dir) if base_dir else None
    lines = []
    new = dict(cur)

    G, M = ballistic_pairs(s)
    if len(G) >= 8:
        th, a, sc, res = fit_distortion(G, M)
        th_b, a_b, sc_b = 0.0, 1.0, 1.0
        if base is not None:
            Gb, Mb = ballistic_pairs(base)
            if len(Gb) >= 8:
                th_b, a_b, sc_b, _ = fit_distortion(Gb, Mb)
        dth = th - th_b
        da = a / a_b
        new["mount_yaw_deg"] = cur["mount_yaw_deg"] - DAMP * float(np.clip(dth, -45, 45))
        new["aspect"] = float(np.clip(cur["aspect"] / (1 + DAMP * (float(np.clip(da, 0.6, 1.6)) - 1)), 0.5, 2.0))
        lines.append(f"direction: {len(G)} aiming moves; cursor rotated {dth:+.1f} deg vs intent (your mouse bias {th_b:+.1f}), "
                     f"vertical/horizontal gain {da:.2f} -> mount_yaw {cur['mount_yaw_deg']:.1f} -> {new['mount_yaw_deg']:.1f}, "
                     f"aspect {cur['aspect']:.2f} -> {new['aspect']:.2f}")
        # speed: your first move should cover as much of the distance as it does with the mouse
        ratio = float(np.clip(sc_b / sc, 0.5, 2.5))
        new["dpi"] = float(np.clip(cur["dpi"] * (1 + DAMP * (ratio - 1)), 200, 4000))
        lines.append(f"speed: first move covers {sc:.2f} of the distance (mouse {sc_b:.2f}) -> dpi {cur['dpi']:.0f} -> {new['dpi']:.0f}")
    else:
        lines.append(f"direction: only {len(G)} clean aiming moves, not tuning")

    if "imu" in s:
        v, note = tap_threshold(s, cur["tap_thr_min"])
        lines.append(note)
        if v is not None:
            new["tap_thr_min"] = v
        v, note = scroll_threshold(s, cur["scroll_omega"])
        lines.append(note)
        if v is not None:
            new["scroll_omega"] = v

    pr = s["meta"]["results"].get("Precision")
    if pr and pr.get("hover_jitter_px") is not None:
        bj = (base["meta"]["results"].get("Precision") or {}).get("hover_jitter_px") if base else None
        j = pr["hover_jitter_px"]
        f = 1.0
        if j > max(0.5, 2 * (bj or 0.25)):
            f = 1.3
        elif pr.get("timeouts", 0) >= 3 and j < 0.3:
            f = 0.8
        new["still_flow_px"] = float(np.clip(cur["still_flow_px"] * f, 0.05, 0.6))
        lines.append(f"stillness: hover jitter {j:.2f} px/frame (mouse {bj if bj is not None else '-'}), "
                     f"timeouts {pr.get('timeouts')} -> still_flow_px {cur['still_flow_px']:.2f} -> {new['still_flow_px']:.2f}")


    out = dict(overrides)
    out.update(new)
    hist = out.get("history", [])
    hist.append({"session": d.name, "before": cur, "after": new, "results": s["meta"]["results"]})
    out["history"] = hist[-20:]
    params_path.write_text(json.dumps(out, indent=1))
    lines.append(f"saved {params_path.name}; next session uses the new settings")
    return lines


if __name__ == "__main__":
    sys.path.insert(0, str(HERE.parent / "proto"))
    d = Path(sys.argv[1]) if len(sys.argv) > 1 else latest_session("phone")
    if d is None:
        sys.exit("no phone session found")
    for ln in run(d):
        print(ln)
