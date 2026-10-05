// All tunable parameters. Mirrors PC-part/proto/deskmouse/config.py (FrontEndConfig) and
// engine.py (EngineConfig): same names, same defaults. Change both sides together.
#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace deskmouse {

struct FrontEndConfig {
    // LK tracking (§6.2 step 2-3)
    int lk_win = 21;
    int lk_levels = 3;
    int lk_iters = 30;
    double lk_eps = 0.01;
    double fb_max_px = 0.5;
    // Feature replenishment (§6.2 step 8)
    int min_tracks = 80;
    int max_tracks = 150;
    double gftt_quality = 0.01;
    int gftt_min_dist = 12;
    int grid = 4;
    int saturation = 250;
    int saturation_margin = 3;
    double clahe_below_median = 60.0;  // apply CLAHE when median intensity is below this (§6.2 step 1)
    bool clahe_always = false;
    // Motion model: d_i = rho_i * w + dpsi * J n_i + sigma * n_i
    int ransac_iters = 60;
    double inlier_px = 0.5;
    double rho_min_motion_px = 3.0;
    int bad_frames_drop = 3;
    double cov_floor_px = 0.02;
    int q_inliers_full = 30;
    int q_inliers_zero = 8;
    double scale_anomaly = 0.015;
    double rot_sign = 1.0;
    int64_t clock_offset_ns = 2'000'000;
};

struct EngineConfig {
    // geometry (S24, measured from place-1 recordings)
    std::array<double, 4> M_ci{0.0, -1.0, 1.0, 0.0};  // image = M_ci · body XY, row-major
    double rot_sign = 1.0;
    int64_t clock_offset_ns = 2'000'000;
    std::array<double, 2> r_cam{-0.022, 0.044};
    double mount_yaw_deg = 0.0;
    double aspect = 1.0;
    bool world_aligned = false;
    bool instant_left = false;
    // output mapping (§6.8)
    double dpi = 800.0;
    double g_max = 2.5;
    double v_lo = 0.05;
    double v_hi = 0.5;
    // latency compensation
    double latency_comp_ms = 0.0;
    bool lead_use_imu = true;
    double lead_max_m = 0.03;
    double lead_vel_alpha = 0.5;
    // 1€ smoothing
    bool smooth = true;
    double oe_min_cutoff_hz = 1.0;
    double oe_beta = 0.01;
    double oe_d_cutoff_hz = 1.0;
    // stillness (§6.4)
    double still_gyro = 0.03;
    double still_acc_std = 0.08;
    double still_ms = 60.0;
    double still_flow_px = 0.15;
    double gyro_bias_tau_s = 3.0;
    // scale
    double ceiling_h = 1.9;
    double ceiling_percentile = 15.0;
    double scale_alpha = 0.35;
    double stroke_min_m = 0.03;
    double stroke_max_s = 2.0;
    double stroke_min_peak_speed = 0.08;
    // taps (§6.6)
    double tap_thr_min = 0.4;
    double tap_mad_k = 8.0;
    double tap_refractory_ms = 100.0;
    double tap_suppress_before_ms = 15.0;
    double tap_suppress_after_ms = 120.0;
    double double_tap_ms = 250.0;
    // scroll (§6.7)
    bool scroll_enabled = true;
    double scroll_omega = 0.3;
    double scroll_enter_ms = 40.0;
    double scroll_exit_omega = 0.1;
    double scroll_exit_ms = 150.0;
    double scroll_step_deg = 4.0;
    double scroll_sign = 1.0;
    double scroll_max_ref_speed = 0.03;
    // lift (§6.5)
    double lift_tilt_deg = 4.0;
    double lift_scale_anomaly = 0.015;
};

// Sets a parameter by its Python name (engine fields first, then front end fields with an "fe." prefix).
// Arrays: M_ci.0..3, r_cam.0..1. Booleans: 0 / non-zero. Returns false for an unknown name.
bool set_param(EngineConfig& e, FrontEndConfig& f, const std::string& name, double v);

}  // namespace deskmouse
