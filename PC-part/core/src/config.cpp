#include "deskmouse/config.hpp"

#include <functional>
#include <unordered_map>

namespace deskmouse {

namespace {
using Setter = std::function<void(EngineConfig&, FrontEndConfig&, double)>;

template <class T>
Setter E(T EngineConfig::*m) { return [m](EngineConfig& e, FrontEndConfig&, double v) { e.*m = static_cast<T>(v); }; }
template <class T>
Setter F(T FrontEndConfig::*m) { return [m](EngineConfig&, FrontEndConfig& f, double v) { f.*m = static_cast<T>(v); }; }
Setter B(bool EngineConfig::*m) { return [m](EngineConfig& e, FrontEndConfig&, double v) { e.*m = v != 0; }; }

const std::unordered_map<std::string, Setter>& table() {
    static const std::unordered_map<std::string, Setter> t = {
        {"M_ci.0", [](auto& e, auto&, double v) { e.M_ci[0] = v; }},
        {"M_ci.1", [](auto& e, auto&, double v) { e.M_ci[1] = v; }},
        {"M_ci.2", [](auto& e, auto&, double v) { e.M_ci[2] = v; }},
        {"M_ci.3", [](auto& e, auto&, double v) { e.M_ci[3] = v; }},
        {"r_cam.0", [](auto& e, auto&, double v) { e.r_cam[0] = v; }},
        {"r_cam.1", [](auto& e, auto&, double v) { e.r_cam[1] = v; }},
        {"rot_sign", E(&EngineConfig::rot_sign)},
        {"clock_offset_ns", E(&EngineConfig::clock_offset_ns)},
        {"mount_yaw_deg", E(&EngineConfig::mount_yaw_deg)},
        {"aspect", E(&EngineConfig::aspect)},
        {"world_aligned", B(&EngineConfig::world_aligned)},
        {"instant_left", B(&EngineConfig::instant_left)},
        {"dpi", E(&EngineConfig::dpi)},
        {"g_max", E(&EngineConfig::g_max)},
        {"v_lo", E(&EngineConfig::v_lo)},
        {"v_hi", E(&EngineConfig::v_hi)},
        {"latency_comp_ms", E(&EngineConfig::latency_comp_ms)},
        {"lead_use_imu", B(&EngineConfig::lead_use_imu)},
        {"lead_max_m", E(&EngineConfig::lead_max_m)},
        {"lead_vel_alpha", E(&EngineConfig::lead_vel_alpha)},
        {"smooth", B(&EngineConfig::smooth)},
        {"oe_min_cutoff_hz", E(&EngineConfig::oe_min_cutoff_hz)},
        {"oe_beta", E(&EngineConfig::oe_beta)},
        {"oe_d_cutoff_hz", E(&EngineConfig::oe_d_cutoff_hz)},
        {"still_gyro", E(&EngineConfig::still_gyro)},
        {"still_acc_std", E(&EngineConfig::still_acc_std)},
        {"still_ms", E(&EngineConfig::still_ms)},
        {"still_flow_px", E(&EngineConfig::still_flow_px)},
        {"gyro_bias_tau_s", E(&EngineConfig::gyro_bias_tau_s)},
        {"ceiling_h", E(&EngineConfig::ceiling_h)},
        {"ceiling_percentile", E(&EngineConfig::ceiling_percentile)},
        {"scale_alpha", E(&EngineConfig::scale_alpha)},
        {"stroke_min_m", E(&EngineConfig::stroke_min_m)},
        {"stroke_max_s", E(&EngineConfig::stroke_max_s)},
        {"stroke_min_peak_speed", E(&EngineConfig::stroke_min_peak_speed)},
        {"tap_thr_min", E(&EngineConfig::tap_thr_min)},
        {"tap_mad_k", E(&EngineConfig::tap_mad_k)},
        {"tap_refractory_ms", E(&EngineConfig::tap_refractory_ms)},
        {"tap_suppress_before_ms", E(&EngineConfig::tap_suppress_before_ms)},
        {"tap_suppress_after_ms", E(&EngineConfig::tap_suppress_after_ms)},
        {"double_tap_ms", E(&EngineConfig::double_tap_ms)},
        {"shock_jerk", E(&EngineConfig::shock_jerk)},
        {"shock_suppress_before_ms", E(&EngineConfig::shock_suppress_before_ms)},
        {"shock_suppress_after_ms", E(&EngineConfig::shock_suppress_after_ms)},
        {"scroll_enabled", B(&EngineConfig::scroll_enabled)},
        {"scroll_omega", E(&EngineConfig::scroll_omega)},
        {"scroll_enter_ms", E(&EngineConfig::scroll_enter_ms)},
        {"scroll_exit_omega", E(&EngineConfig::scroll_exit_omega)},
        {"scroll_exit_ms", E(&EngineConfig::scroll_exit_ms)},
        {"scroll_step_deg", E(&EngineConfig::scroll_step_deg)},
        {"scroll_sign", E(&EngineConfig::scroll_sign)},
        {"scroll_max_ref_speed", E(&EngineConfig::scroll_max_ref_speed)},
        {"lift_tilt_deg", E(&EngineConfig::lift_tilt_deg)},
        {"lift_scale_anomaly", E(&EngineConfig::lift_scale_anomaly)},
        {"kf", B(&EngineConfig::kf)},
        {"kf_acc_noise", E(&EngineConfig::kf_acc_noise)},
        {"kf_scale_noise", E(&EngineConfig::kf_scale_noise)},
        {"kf_learn_scale", B(&EngineConfig::kf_learn_scale)},
        {"kf_scale_sigma0", E(&EngineConfig::kf_scale_sigma0)},
        {"kf_scale_sigma_cal", E(&EngineConfig::kf_scale_sigma_cal)},
        {"kf_stroke_sigma", E(&EngineConfig::kf_stroke_sigma)},
        {"kf_cam_sigma_px", E(&EngineConfig::kf_cam_sigma_px)},
        {"kf_cam_sigma_rel", E(&EngineConfig::kf_cam_sigma_rel)},
        {"kf_gate", E(&EngineConfig::kf_gate)},
        {"kf_resync_frames", E(&EngineConfig::kf_resync_frames)},
        {"kf_leash_mm", E(&EngineConfig::kf_leash_mm)},
        {"kf_predict_now", B(&EngineConfig::kf_predict_now)},
        {"fe.lk_win", F(&FrontEndConfig::lk_win)},
        {"fe.lk_levels", F(&FrontEndConfig::lk_levels)},
        {"fe.lk_iters", F(&FrontEndConfig::lk_iters)},
        {"fe.lk_eps", F(&FrontEndConfig::lk_eps)},
        {"fe.fb_max_px", F(&FrontEndConfig::fb_max_px)},
        {"fe.min_tracks", F(&FrontEndConfig::min_tracks)},
        {"fe.max_tracks", F(&FrontEndConfig::max_tracks)},
        {"fe.gftt_quality", F(&FrontEndConfig::gftt_quality)},
        {"fe.gftt_min_dist", F(&FrontEndConfig::gftt_min_dist)},
        {"fe.grid", F(&FrontEndConfig::grid)},
        {"fe.saturation", F(&FrontEndConfig::saturation)},
        {"fe.saturation_margin", F(&FrontEndConfig::saturation_margin)},
        {"fe.clahe_below_median", F(&FrontEndConfig::clahe_below_median)},
        {"fe.clahe_always", [](auto&, auto& f, double v) { f.clahe_always = v != 0; }},
        {"fe.ransac_iters", F(&FrontEndConfig::ransac_iters)},
        {"fe.inlier_px", F(&FrontEndConfig::inlier_px)},
        {"fe.rho_min_motion_px", F(&FrontEndConfig::rho_min_motion_px)},
        {"fe.bad_frames_drop", F(&FrontEndConfig::bad_frames_drop)},
        {"fe.cov_floor_px", F(&FrontEndConfig::cov_floor_px)},
        {"fe.q_inliers_full", F(&FrontEndConfig::q_inliers_full)},
        {"fe.q_inliers_zero", F(&FrontEndConfig::q_inliers_zero)},
        {"fe.scale_anomaly", F(&FrontEndConfig::scale_anomaly)},
    };
    return t;
}
}  // namespace

bool set_param(EngineConfig& e, FrontEndConfig& f, const std::string& name, double v) {
    const auto& t = table();
    const auto it = t.find(name);
    if (it == t.end()) return false;
    it->second(e, f, v);
    return true;
}

}  // namespace deskmouse
