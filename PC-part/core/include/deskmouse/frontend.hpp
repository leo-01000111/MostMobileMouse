// Vision front end with per-track inverse depth. Port of PC-part/proto/deskmouse/frontend.py.
//   d_i = rho_i * w + dpsi * J n_i + sigma * n_i
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

#include "deskmouse/config.hpp"

namespace deskmouse {

struct FrameResult {
    int64_t t_ns = 0;
    cv::Vec2d w{0, 0};
    cv::Matx22d w_cov = cv::Matx22d::eye();
    double dpsi_cam = 0;
    double sigma = 0;
    int n_tracks = 0;
    int n_inliers = 0;
    double q = 0;
    bool scale_anomaly = false;
    bool valid = false;
    std::vector<double> rhos;  // established track inverse depths
};

class FrontEnd {
public:
    FrontEnd(const cv::Matx33d& K, const std::array<double, 5>& camera2_dist, cv::Size size, const FrontEndConfig& cfg);

    // y: Y plane (CV_8UC1, not retained); t_ns: mid-exposure time (IMU clock);
    // dpsi_gyro: device-gyro z integrated since the last frame.
    FrameResult process(const cv::Mat& y, int64_t t_ns, double dpsi_gyro);

    double focal() const { return f_; }
    int n_points() const { return static_cast<int>(pts_.size()); }
    const std::vector<cv::Point2f>& points() const { return pts_; }

private:
    struct Track {
        double rho;
        double a = 0, b = 0;  // Σ d·w, Σ |w|²
        int bad = 0, age = 0;
    };

    cv::Mat prep(const cv::Mat& y);
    std::vector<cv::Point2d> normalize(const std::vector<cv::Point2f>& p) const;
    void replenish(const cv::Mat& img);
    bool fit(const std::vector<double>& rho, const std::vector<cv::Point2d>& n, const std::vector<cv::Point2d>& d,
             const std::vector<char>& usable, cv::Vec4d& x, std::vector<char>& inl);

    FrontEndConfig cfg_;
    cv::Matx33d K_;
    cv::Mat dist_;  // OpenCV order k1,k2,p1,p2,k3
    int w_img_, h_img_;
    double f_;
    cv::Mat prev_;
    std::vector<cv::Point2f> pts_;
    std::vector<Track> tracks_;
    bool bootstrapped_ = false;
    cv::Ptr<cv::CLAHE> clahe_;
    uint64_t rng_ = 0;  // SplitMix64 state, same generator as frontend.py
    uint64_t next_random();
};

}  // namespace deskmouse
