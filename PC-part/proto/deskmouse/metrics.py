"""Trajectory metrics that work with or without exact ground truth (DESIGN.md §7)."""
from __future__ import annotations

import numpy as np
import pandas as pd


def pauses(df: pd.DataFrame, speed_thr: float = 0.01, min_s: float = 0.4) -> list[tuple[int, int]]:
    """Index ranges where the estimated reference point is (nearly) still."""
    t = df["t_ns"].to_numpy() / 1e9
    p = df[["x", "y"]].to_numpy()
    v = np.linalg.norm(np.gradient(p, t, axis=0), axis=1)
    v = pd.Series(v).rolling(9, center=True, min_periods=1).median().to_numpy()
    still = v < speed_thr
    out, start = [], None
    for i, s in enumerate(still):
        if s and start is None:
            start = i
        if (not s or i == len(still) - 1) and start is not None:
            end = i if not s else i + 1
            if t[end - 1] - t[start] >= min_s:
                out.append((start, end))
            start = None
    return out


def strokes(df: pd.DataFrame) -> np.ndarray:
    """Displacement vectors between consecutive pauses (median position of each pause)."""
    pos = [df[["x", "y"]].iloc[a:b].median().to_numpy() for a, b in pauses(df)]
    return np.diff(np.array(pos), axis=0) if len(pos) > 1 else np.zeros((0, 2))


def ruler_metrics(df: pd.DataFrame, nominal_m: float = 0.2) -> dict:
    s = strokes(df)
    s = s[np.linalg.norm(s, axis=1) > 0.3 * nominal_m]
    if len(s) == 0:
        return {"strokes": 0}
    # main axis from the strokes themselves (sign-insensitive PCA)
    u, sv, vt = np.linalg.svd(s - 0, full_matrices=False)
    axis = vt[0]
    along = s @ axis
    across = s @ np.array([-axis[1], axis[0]])
    L = np.abs(along)
    return {
        "strokes": int(len(s)),
        "length_mean_m": float(L.mean()),
        "length_std_pct": float(100 * L.std() / L.mean()),
        "length_err_vs_nominal_pct": float(100 * (L.mean() - nominal_m) / nominal_m),
        "cross_axis_pct": float(100 * np.sqrt(np.mean(across**2)) / L.mean()),
        "axis_angle_deg": float(np.degrees(np.arctan2(axis[1], axis[0]))),
        "roundtrip_closure_mm": float(1000 * np.linalg.norm(s.sum(axis=0)) / max(1, len(s) // 2)),
    }


def gt_errors(df: pd.DataFrame, gt: pd.DataFrame) -> dict:
    """Against exact ground truth (simulator): position error after aligning the start."""
    x = np.interp(df["t_ns"], gt["t_ns"], gt["x"])
    y = np.interp(df["t_ns"], gt["t_ns"], gt["y"])
    e = np.hypot(df["x"] - x, df["y"] - y)
    path = np.sum(np.hypot(np.diff(gt["x"]), np.diff(gt["y"])))
    return {"final_err_mm": float(1000 * e.iloc[-1]), "max_err_mm": float(1000 * e.max()),
            "rms_err_mm": float(1000 * np.sqrt(np.mean(e**2))), "path_m": float(path),
            "final_err_pct_path": float(100 * e.iloc[-1] / max(path, 1e-9))}
