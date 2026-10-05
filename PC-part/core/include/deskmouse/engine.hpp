// Real-time MVP engine: IMU + camera frames in, mouse events out. Port of PC-part/proto/deskmouse/engine.py.
// Feed inputs in arrival order: on_imu() for every gyro/accel sample (then poll()), on_frame() for every frame.
#pragma once

#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "deskmouse/config.hpp"
#include "deskmouse/frontend.hpp"

namespace deskmouse {

enum class EventKind { Move, Left, Right, Wheel, Lift, Land };

struct Event {
    int64_t t_ns;
    EventKind kind;
    int dx = 0, dy = 0, wheel = 0;
};

const char* event_name(EventKind k);

class Engine {
public:
    // gauge_ratio: calibrated scale ÷ ceiling-gauge scale from a previous session (0 = none).
    Engine(const cv::Matx33d& K, const std::array<double, 5>& camera2_dist, cv::Size size, const EngineConfig& cfg,
           FrontEndConfig fe_cfg, double gauge_ratio = 0.0);

    enum ImuKind { Gyro = 0, Accel = 1 };
    void on_imu(int64_t t_ns, int kind, double x, double y, double z);
    void poll(int64_t t_ns);
    void on_frame(const cv::Mat& y, int64_t t_sensor_ns, int64_t exposure_ns = 0, int64_t skew_ns = 0);
    // Everything after the front end (engine.py on_result); used by the exact engine parity check.
    void on_result(int64_t t, double psi_prev, double psi, double wx, double wy, double sigma, bool valid, int n_rhos,
                   double rho15, double rho_med);

    std::vector<Event> drain();
    std::optional<double> gauge_ratio() const;

    // state for the UI / logs
    bool still() const { return still_; }
    bool lifted() const { return lifted_; }
    bool scrolling() const { return scroll_; }
    std::optional<double> scale() const { return lam_; }
    std::map<std::string, int> stats;

    // last front-end result (parity logging)
    FrameResult last_result;
    const FrontEnd& front_end() const { return fe_; }

private:
    double psi_at(int64_t t_ns) const;
    void still_update(int64_t t_ns);
    void tap_update(int64_t t_ns, double az);
    void scroll_update(int64_t t_ns, double wz);
    void lift_update(int64_t t_ns);
    void stroke_end(int64_t t_ns);
    cv::Vec2d to_output(const cv::Vec2d& v_world, double psi) const;
    void emit(int64_t t, const cv::Vec2d& counts);
    cv::Vec2d lead_step(int64_t t, const cv::Vec2d& v_counts, double psi, double k);

    EngineConfig cfg_;
    FrontEnd fe_;
    cv::Matx22d Minv_;
    cv::Vec2d r_cam_;
    // IMU state
    cv::Vec3d gyro_bias_{0, 0, 0};
    std::deque<std::pair<int64_t, cv::Vec3d>> bias_init_;
    bool bias_ok_ = false;
    std::deque<std::pair<int64_t, double>> gyro_t_;  // (t, Z) cumulative integral of bias-corrected device gyro z
    double Zcum_ = 0;
    std::optional<int64_t> last_gyro_t_;
    cv::Vec2d tilt_{0, 0};
    std::deque<std::pair<int64_t, double>> recent_gyro_;
    struct Acc { int64_t t; double bx, by; };
    std::deque<Acc> recent_acc_, acc_buf_;
    std::optional<cv::Vec2d> rest_acc_;
    bool still_ = false;
    std::optional<int64_t> imu_still_since_;
    double last_flow_px_ = 0;
    // taps
    std::deque<double> jerk_;
    std::optional<double> last_az_;
    double tap_thr_;
    int64_t last_tap_t_ = -(int64_t(1) << 62);
    std::optional<int64_t> pending_tap_t_;
    int64_t suppress_until_ = -(int64_t(1) << 62);
    int64_t suppress_from_ = int64_t(1) << 62;
    // scroll
    bool scroll_ = false;
    std::optional<int64_t> scroll_cand_since_, scroll_quiet_since_;
    double scroll_acc_ = 0, scroll_psi_last_ = 0;
    // lift
    bool lifted_ = false;
    int anomaly_frames_ = 0;
    // vision / output
    std::optional<double> lam_;
    double gauge_ratio0_;
    std::vector<double> lam_gauge_;
    cv::Vec2d stroke_vis_{0, 0};
    std::optional<int64_t> stroke_start_;
    std::optional<double> prev_Z_;
    double last_ref_speed_ = 0;
    std::vector<Event> events_;
    cv::Vec2d v_ref_{0, 0}, lead_{0, 0};
    std::optional<int64_t> last_frame_t_, last_emit_t_;
    cv::Vec2d pos_raw_{0, 0}, pos_f_{0, 0}, pos_sent_{0, 0};
    double speed_f_ = 0;
};

}  // namespace deskmouse
