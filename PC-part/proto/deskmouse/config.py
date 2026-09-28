"""All tunable parameters (DESIGN.md §6 defaults, plus the multi-depth front end from notes/DECISIONS.md)."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class FrontEndConfig:
    # LK tracking (§6.2 step 2-3)
    lk_win: int = 21
    lk_levels: int = 3
    lk_iters: int = 30
    lk_eps: float = 0.01
    fb_max_px: float = 0.5
    # Feature replenishment (§6.2 step 8)
    min_tracks: int = 80
    max_tracks: int = 150
    gftt_quality: float = 0.01
    gftt_min_dist: int = 12
    grid: int = 4
    saturation: int = 250
    saturation_margin: int = 3
    clahe_below_median: float = 60.0  # apply CLAHE when median intensity is below this (§6.2 step 1)
    clahe_always: bool = False
    # Motion model: d_i = rho_i * w + dpsi * J n_i + sigma * n_i (multi-depth, notes/DECISIONS.md)
    ransac_iters: int = 60
    inlier_px: float = 0.5            # residual threshold, pixels
    rho_min_motion_px: float = 3.0    # accumulated |w| (in px of a rho=1 track) before a track's depth counts as known
    bad_frames_drop: int = 3          # drop a track after this many consecutive outlier frames
    cov_floor_px: float = 0.02
    q_inliers_full: int = 30
    q_inliers_zero: int = 8
    scale_anomaly: float = 0.015      # |sigma| per frame that flags lift/tilt (§6.2 step 7)
    # Gyro-image rotation link: image rotation (rad) = rot_sign * device gyro z integrated
    rot_sign: float = 1.0
    clock_offset_ns: int = 2_000_000  # camera mid-exposure time + offset = IMU time (measured on S24 twist takes)


@dataclass
class VoConfig:
    ceiling_height_m: float = 1.9     # gauge for vision-only mode: farthest depth cluster = ceiling
    ceiling_percentile: float = 15.0  # which percentile of established rho is taken as the ceiling
    M_ci: list = field(default_factory=lambda: [[1.0, 0.0], [0.0, -1.0]])  # image xy = M_ci · body XY
    r_cam: tuple[float, float] = (0.0, 0.0)  # lens offset from reference point, body frame (m)
    gyro_bias_init_s: float = 1.0    # average gyro over the first seconds (assumed still) for bias
