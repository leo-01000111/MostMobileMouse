"""Vision-only odometry (M1): front end + gyro yaw, scale from the ceiling-height gauge.

Yaw ψ of body B in W: ψ = -∫ gyro_z_dev (Z_B = -z_dev, DESIGN.md §2), bias from the initial still period.
Front-end model: d_i = ρ_i w, with ρ_i = 1/height of the point above the lens and
w = -M_ci · Δc_B (lens displacement in body axes of the current frame). Relative ρ's have one unknown
scale λ (ρ_true = λ ρ_rel); vision-only fixes it by assuming the ceiling_percentile of ρ is the ceiling.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

from .config import FrontEndConfig, VoConfig
from .frontend import FrontEnd
from .io import Recording


def gyro_integrator(rec: Recording, bias_s: float):
    """Returns Z(t_ns) = ∫(gyro_z_dev - bias) dt (rad), bias estimate."""
    g = rec.imu_of("gyro")
    t = g["t_ns"].to_numpy()
    z = g["z"].to_numpy()
    bias = float(np.mean(z[t < t[0] + bias_s * 1e9])) if len(t) else 0.0
    zc = z - bias
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (zc[1:] + zc[:-1]) * np.diff(t) / 1e9)])
    return (lambda tq: np.interp(tq, t, cum)), bias


def frame_times(rec: Recording, clock_offset_ns: int) -> np.ndarray:
    """Mid-exposure (+ half rolling-shutter skew) in the IMU clock (§6.2 output timestamp)."""
    f = rec.frames
    exp = f["exposure_ns"].fillna(0).to_numpy()
    skew = f["rolling_shutter_skew_ns"].fillna(0).clip(lower=0).to_numpy()
    return (f["t_ns"].to_numpy() + exp // 2 + skew // 2 + clock_offset_ns).astype(np.int64)


def run_vision_only(rec: Recording, fe_cfg: FrontEndConfig | None = None, vo_cfg: VoConfig | None = None,
                    max_frames: int | None = None, freeze_gauge_after_s: float | None = None) -> pd.DataFrame:
    """freeze_gauge_after_s: stop updating the ceiling gauge after this time (diagnostic: separates the
    fragile vision-only gauge from drift of the tracker's own relative scale)."""
    fe_cfg = fe_cfg or FrontEndConfig()
    vo_cfg = vo_cfg or VoConfig()
    if "sim" in rec.meta:  # simulator's own axis conventions
        fe_cfg.rot_sign = rec.meta["sim"].get("rot_sign", fe_cfg.rot_sign)
        vo_cfg.M_ci = rec.meta["sim"].get("M_ci", vo_cfg.M_ci)
        vo_cfg.r_cam = tuple(rec.meta["sim"].get("r_cam", vo_cfg.r_cam))
        fe_cfg.clock_offset_ns = 0
    K, dist = rec.intrinsics()
    fe = FrontEnd(K, dist, (rec.width, rec.height), fe_cfg)
    Z, bias = gyro_integrator(rec, vo_cfg.gyro_bias_init_s)
    tf = frame_times(rec, fe_cfg.clock_offset_ns)
    Minv = np.linalg.inv(np.array(vo_cfg.M_ci, float))
    r_cam = np.array(vo_cfg.r_cam, float)

    lens = np.zeros(2)
    pending = np.zeros(2)  # world-frame motion in relative units while the gauge is still unknown
    lam_hist: list[float] = []
    rows = []
    z_prev = None
    for k, y in rec.frames_y(0, max_frames):
        zk = float(Z(tf[k]))
        res = fe.process(y, int(tf[k]), 0.0 if z_prev is None else zk - z_prev)
        z_prev = zk
        psi = -zk
        frozen = freeze_gauge_after_s is not None and lam_hist and (tf[k] - tf[0]) / 1e9 > freeze_gauge_after_s
        if res.valid and len(res.rhos) >= 8 and not frozen:
            p = np.percentile(res.rhos, vo_cfg.ceiling_percentile)
            if p > 0:
                lam_hist.append(1.0 / (vo_cfg.ceiling_height_m * p))
                lam_hist = lam_hist[-300:]
        lam = float(np.median(lam_hist)) if lam_hist else np.nan
        if res.valid:
            dc_body = -Minv @ res.w  # relative units; divide by λ for metres
            c, s = np.cos(psi), np.sin(psi)
            step = np.array([c * dc_body[0] - s * dc_body[1], s * dc_body[0] + c * dc_body[1]])
            if np.isfinite(lam):
                lens += (pending + step) / lam
                pending[:] = 0
            else:
                pending += step
        c, s = np.cos(psi), np.sin(psi)
        ref = lens - np.array([c * r_cam[0] - s * r_cam[1], s * r_cam[0] + c * r_cam[1]])
        rows.append(dict(t_ns=int(tf[k]), x=ref[0], y=ref[1], psi=psi, n_tracks=res.n_tracks,
                         n_inliers=res.n_inliers, q=res.q, sigma=res.sigma, scale_anomaly=res.scale_anomaly,
                         dpsi_cam=res.dpsi_cam, lam=lam, valid=res.valid))
    df = pd.DataFrame(rows)
    df.attrs["gyro_bias"] = bias
    df.attrs["final_rhos"] = np.array([t.rho for t in fe.tracks])
    return df
