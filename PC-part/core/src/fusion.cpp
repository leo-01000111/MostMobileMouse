#include "deskmouse/fusion.hpp"

#include <cmath>

#include "util.hpp"

namespace deskmouse {

VelocityKF::VelocityKF(double acc_noise, double scale_noise, double gate, int resync_frames, bool learn_scale,
                       size_t history)
    : q_acc_(acc_noise), q_l_(scale_noise), gate_(gate), resync_frames_(resync_frames), learn_scale_(learn_scale),
      history_(history) {}

double VelocityKF::lam() const { return *lam0_ * std::exp(l_); }

void VelocityKF::set_scale(double lam, double sigma) {
    lam0_ = lam;
    l_ = 0.0;
    for (int i = 0; i < 3; ++i) P_(2, i) = P_(i, 2) = 0;
    P_(2, 2) = sigma * sigma;
}

void VelocityKF::scale_measurement(double lam_meas, double sigma) {
    const double y = std::log(lam_meas / lam());
    const double S = P_(2, 2) + sigma * sigma;
    const cv::Vec3d K(P_(0, 2) / S, P_(1, 2) / S, P_(2, 2) / S);
    c_ += cv::Vec2d(K[0], K[1]) * y;
    l_ += K[2] * y;
    const cv::Matx13d row(P_(2, 0), P_(2, 1), P_(2, 2));
    P_ = P_ - cv::Matx31d(K[0], K[1], K[2]) * row;
}

void VelocityKF::zero_velocity(double sigma) {
    c_ = -V_;
    P_(0, 0) = P_(1, 1) = sigma * sigma;
    P_(0, 1) = P_(1, 0) = 0;
    P_(0, 2) = P_(1, 2) = P_(2, 0) = P_(2, 1) = 0;
}

void VelocityKF::predict(int64_t t, const cv::Vec2d& a_world) {
    if (t_) {
        const double dt = double(t - *t_) / 1e9;
        if (0 < dt && dt < 0.05) {
            const cv::Vec2d Vn = V_ + a_world * dt;
            PV_ += (V_ + Vn) * 0.5 * dt;
            C_ += c_ * dt;
            V_ = Vn;
            P_(0, 0) += q_acc_ * q_acc_ * dt;
            P_(1, 1) += q_acc_ * q_acc_ * dt;
            P_(2, 2) += q_l_ * q_l_ * dt;
        }
    }
    t_ = t;
    hist_.push_back({t, V_, PV_, C_});
    if (hist_.size() > history_) hist_.pop_front();
}

cv::Vec2d VelocityKF::PV_at(int64_t t) const {
    const auto gx = [](const Rec& r) { return r.t; };
    return {util::interp(double(t), hist_, gx, [](const Rec& r) { return r.PV[0]; }),
            util::interp(double(t), hist_, gx, [](const Rec& r) { return r.PV[1]; })};
}

std::optional<cv::Vec2d> VelocityKF::interval_mean_V(int64_t t0, int64_t t1) const {
    if (hist_.empty() || t1 <= t0 || hist_.front().t > t0) return std::nullopt;
    if (t1 > hist_.back().t) return std::nullopt;
    return (PV_at(t1) - PV_at(t0)) * (1.0 / (double(t1 - t0) / 1e9));
}

cv::Vec2d VelocityKF::position_at(int64_t t) const {
    if (t >= hist_.back().t) return PV_ + C_;
    return PV_at(t) + C_ - c_ * (double(*t_ - t) / 1e9);
}

bool VelocityKF::camera(int64_t t0, int64_t t1, const cv::Vec2d& d_rel, double sigma_rel) {
    if (!ready()) return false;
    const auto Vbar = interval_mean_V(t0, t1);
    if (!Vbar) return false;
    const double dt = double(t1 - t0) / 1e9;
    const cv::Vec2d z = d_rel * (1.0 / dt);
    const double lam = this->lam();
    const cv::Vec2d va = *Vbar + c_;
    const cv::Vec2d h = va * (1.0 / lam);
    cv::Matx23d H = cv::Matx23d::zeros();
    H(0, 0) = H(1, 1) = 1 / lam;
    if (learn_scale_) { H(0, 2) = -va[0] / lam; H(1, 2) = -va[1] / lam; }
    const double r = (sigma_rel / dt) * (sigma_rel / dt);
    const cv::Vec2d y = z - h;
    const cv::Matx22d S = H * P_ * H.t() + cv::Matx22d(r, 0, 0, r);
    const cv::Matx22d Si = S.inv();
    const double m2 = (y.t() * Si * y)(0);
    if (m2 > gate_ * gate_) {
        stats["rejected"] += 1;
        rejected_run_ += 1;
        if (rejected_run_ >= resync_frames_) {
            // lost lock (e.g. long bump, wrong scale): trust the camera again
            c_ = z * lam - *Vbar;
            const double s2 = ((sigma_rel / dt) * lam) * ((sigma_rel / dt) * lam);
            P_(0, 0) = P_(1, 1) = s2;
            P_(0, 1) = P_(1, 0) = 0;
            P_(0, 2) = P_(1, 2) = P_(2, 0) = P_(2, 1) = 0;
            rejected_run_ = 0;
            stats["resyncs"] += 1;
        }
        return false;
    }
    rejected_run_ = 0;
    const cv::Matx32d K = P_ * H.t() * Si;
    const cv::Vec3d dx = K * y;
    c_ += cv::Vec2d(dx[0], dx[1]);
    l_ += dx[2];
    P_ = (cv::Matx33d::eye() - K * H) * P_;
    P_ = (P_ + P_.t()) * 0.5;
    // the corrected offset also applies between the measured interval and now
    C_ += cv::Vec2d(dx[0], dx[1]) * ((double(*t_) - (double(t0) + double(t1)) / 2) / 1e9);
    stats["updates"] += 1;
    return true;
}

}  // namespace deskmouse
