// Velocity Kalman filter: port of PC-part/proto/deskmouse/fusion.py (see there for the model).
#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>

namespace deskmouse {

class VelocityKF {
public:
    VelocityKF(double acc_noise, double scale_noise, double gate, int resync_frames, bool learn_scale,
               size_t history = 1500);

    bool ready() const { return lam0_.has_value() && hist_.size() > 1; }
    bool has_scale() const { return lam0_.has_value(); }
    double lam() const;
    std::optional<int64_t> t() const { return t_; }

    void set_scale(double lam, double sigma);
    void scale_measurement(double lam_meas, double sigma);
    void zero_velocity(double sigma = 0.0);
    void predict(int64_t t, const cv::Vec2d& a_world);
    cv::Vec2d position_at(int64_t t) const;
    bool camera(int64_t t0, int64_t t1, const cv::Vec2d& d_rel, double sigma_rel);

    std::map<std::string, int> stats{{"updates", 0}, {"rejected", 0}, {"resyncs", 0}};

private:
    struct Rec { int64_t t; cv::Vec2d V, PV, C; };
    std::optional<cv::Vec2d> interval_mean_V(int64_t t0, int64_t t1) const;
    cv::Vec2d PV_at(int64_t t) const;

    double q_acc_, q_l_, gate_;
    int resync_frames_;
    bool learn_scale_;
    size_t history_;
    cv::Vec2d V_{0, 0}, PV_{0, 0}, C_{0, 0}, c_{0, 0};
    double l_ = 0;
    std::optional<double> lam0_;
    cv::Matx33d P_ = cv::Matx33d::diag(cv::Vec3d(1, 1, 0));
    std::optional<int64_t> t_;
    std::deque<Rec> hist_;
    int rejected_run_ = 0;
};

}  // namespace deskmouse
