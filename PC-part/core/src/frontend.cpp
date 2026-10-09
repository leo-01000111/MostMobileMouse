#include "deskmouse/frontend.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>

#include "util.hpp"

namespace deskmouse {

namespace {
// Same literals as frontend.py.
constexpr double kMovingPx = 0.05;   // |w| in px above which the phone counts as moving for depth updates
constexpr int kMinKnownTracks = 8;   // known-depth tracks needed before only those are used
constexpr int kMinFitTracks = 3;
constexpr double kBadResFactor = 2.0;  // outlier frame = residual above this × inlier_px
constexpr double kCovSmallN = 1e-6;
}  // namespace

FrontEnd::FrontEnd(const cv::Matx33d& K, const std::array<double, 5>& d, cv::Size size, const FrontEndConfig& cfg)
    : cfg_(cfg), K_(K), w_img_(size.width), h_img_(size.height) {
    // Camera2 distortion is [k1,k2,k3,p1,p2]; OpenCV wants [k1,k2,p1,p2,k3]
    dist_ = (cv::Mat_<double>(1, 5) << d[0], d[1], d[3], d[4], d[2]);
    f_ = 0.5 * (K(0, 0) + K(1, 1));
    clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
}

cv::Mat FrontEnd::prep(const cv::Mat& y) {
    bool apply = cfg_.clahe_always;
    if (!apply) {
        // np.median of a uint8 image, via a histogram
        int hist[256] = {0};
        for (int r = 0; r < y.rows; ++r) {
            const uchar* p = y.ptr<uchar>(r);
            for (int c = 0; c < y.cols; ++c) ++hist[p[c]];
        }
        const int64_t n = int64_t(y.rows) * y.cols;
        auto kth = [&](int64_t k) {  // k-th smallest, 0-based
            int64_t acc = 0;
            for (int v = 0; v < 256; ++v) { acc += hist[v]; if (acc > k) return v; }
            return 255;
        };
        const double med = n % 2 ? kth(n / 2) : 0.5 * (kth(n / 2 - 1) + kth(n / 2));
        apply = med < cfg_.clahe_below_median;
    }
    cv::Mat out;
    if (apply) clahe_->apply(y, out); else out = y.clone();
    return out;
}

std::vector<cv::Point2d> FrontEnd::normalize(const std::vector<cv::Point2f>& p) const {
    std::vector<cv::Point2d> out;
    if (p.empty()) return out;
    std::vector<cv::Point2d> in(p.begin(), p.end());
    cv::undistortPoints(in, out, cv::Mat(K_), dist_);
    return out;
}

void FrontEnd::replenish(const cv::Mat& img) {
    const auto& c = cfg_;
    if (int(pts_.size()) >= c.min_tracks) return;
    cv::Mat mask(img.size(), CV_8UC1, cv::Scalar(255));
    cv::Mat sat = img >= c.saturation;  // 255 where saturated
    if (cv::countNonZero(sat)) {
        const int k = 2 * c.saturation_margin + 1;
        cv::dilate(sat, sat, cv::Mat::ones(k, k, CV_8UC1));
        mask.setTo(0, sat);
    }
    for (const auto& p : pts_) cv::circle(mask, cv::Point(int(p.x), int(p.y)), c.gftt_min_dist, cv::Scalar(0), -1);
    const int need = c.max_tracks - int(pts_.size());
    const int gh = img.rows / c.grid, gw = img.cols / c.grid;
    std::vector<int> counts(c.grid * c.grid, 0);
    for (const auto& p : pts_) counts[std::min(int(p.y) / gh, c.grid - 1) * c.grid + std::min(int(p.x) / gw, c.grid - 1)]++;
    const int per_cell = std::max(1, need / (c.grid * c.grid));
    std::vector<cv::Point2f> added;
    for (int gy = 0; gy < c.grid; ++gy)
        for (int gx = 0; gx < c.grid; ++gx) {
            const int have = counts[gy * c.grid + gx];
            if (have >= per_cell) continue;
            const cv::Rect roi(gx * gw, gy * gh, gw, gh);
            std::vector<cv::Point2f> p;
            // clone: a numpy slice reaches OpenCV as an isolated image, so the corner filters must not see
            // pixels outside the cell (an ROI view would)
            cv::goodFeaturesToTrack(img(roi).clone(), p, per_cell - have, c.gftt_quality, c.gftt_min_dist, mask(roi).clone());
            for (auto& q : p) added.emplace_back(q.x + float(gx * gw), q.y + float(gy * gh));
        }
    if (added.empty()) return;
    if (int(added.size()) > need) added.resize(need);
    std::vector<double> known;
    for (const auto& t : tracks_)
        if (t.b * f_ * f_ >= c.rho_min_motion_px * c.rho_min_motion_px) known.push_back(t.rho);
    const double rho0 = known.empty() ? 1.0 : util::median(known);
    for (const auto& p : added) { tracks_.push_back(Track{rho0}); pts_.push_back(p); }
}

uint64_t FrontEnd::next_random() {
    uint64_t z = (rng_ += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

bool FrontEnd::fit(const std::vector<double>& rho, const std::vector<cv::Point2d>& n, const std::vector<cv::Point2d>& d,
                   const std::vector<char>& usable, cv::Vec4d& x, std::vector<char>& inl) {
    const auto& c = cfg_;
    const size_t N = rho.size();
    const double thr = c.inlier_px / f_;
    inl.assign(N, 0);
    std::vector<int> idx;
    for (size_t i = 0; i < N; ++i) if (usable[i]) idx.push_back(int(i));
    if (idx.size() < kMinFitTracks) return false;

    // RANSAC: each hypothesis from 2 tracks (4 equations, unknowns wx, wy, dpsi)
    const uint64_t m = idx.size();
    int best_n = -1;
    std::vector<char> best, cur(N);
    for (int h = 0; h < c.ransac_iters; ++h) {
        const uint64_t a = next_random() % m;
        uint64_t b = next_random() % (m - 1);
        b += b >= a;
        const int i = idx[a], j = idx[b];
        cv::Matx33d AtA = cv::Matx33d::eye() * 1e-12;
        cv::Vec3d Atb(0, 0, 0);
        for (int s : {i, j}) {
            const cv::Vec3d r0(rho[s], 0, -n[s].y), r1(0, rho[s], n[s].x);
            AtA += r0 * r0.t() + r1 * r1.t();
            Atb += r0 * d[s].x + r1 * d[s].y;
        }
        const cv::Vec3d xh = AtA.solve(Atb, cv::DECOMP_LU);
        int cnt = 0;
        for (size_t k = 0; k < N; ++k) {
            const double px = rho[k] * xh[0] - xh[2] * n[k].y - d[k].x;
            const double py = rho[k] * xh[1] + xh[2] * n[k].x - d[k].y;
            cur[k] = usable[k] && std::hypot(px, py) < thr;
            cnt += cur[k];
        }
        if (cnt > best_n) { best_n = cnt; best = cur; }
    }
    if (best_n < kMinFitTracks) return false;

    inl = best;
    for (int it = 0; it < 2; ++it) {  // refine with sigma, then recompute inliers once
        int m = 0;
        for (char v : inl) m += v;
        cv::Mat A(2 * m, 4, CV_64F), b(2 * m, 1, CV_64F), xs;
        int r = 0;
        for (size_t k = 0; k < N; ++k) {
            if (!inl[k]) continue;
            double* a0 = A.ptr<double>(r);
            double* a1 = A.ptr<double>(r + 1);
            a0[0] = rho[k]; a0[1] = 0; a0[2] = -n[k].y; a0[3] = n[k].x;
            a1[0] = 0; a1[1] = rho[k]; a1[2] = n[k].x; a1[3] = n[k].y;
            b.at<double>(r) = d[k].x;
            b.at<double>(r + 1) = d[k].y;
            r += 2;
        }
        cv::solve(A, b, xs, cv::DECOMP_SVD);
        x = cv::Vec4d(xs.at<double>(0), xs.at<double>(1), xs.at<double>(2), xs.at<double>(3));
        int cnt = 0;
        for (size_t k = 0; k < N; ++k) {
            const double px = rho[k] * x[0] - x[2] * n[k].y + x[3] * n[k].x - d[k].x;
            const double py = rho[k] * x[1] + x[2] * n[k].x + x[3] * n[k].y - d[k].y;
            inl[k] = std::hypot(px, py) < thr;
            cnt += inl[k];
        }
        if (cnt < kMinFitTracks) return false;
    }
    return true;
}

FrameResult FrontEnd::process(const cv::Mat& y, int64_t t_ns, double dpsi_gyro) {
    const auto& c = cfg_;
    cv::Mat img = prep(y);
    FrameResult result;
    result.t_ns = t_ns;
    if (prev_.empty() || pts_.empty()) {
        prev_ = img;
        replenish(img);
        return result;
    }

    const double ang = c.rot_sign * dpsi_gyro;
    const double ca = std::cos(ang), sa = std::sin(ang);
    const double cx = K_(0, 2), cy = K_(1, 2);
    std::vector<cv::Point2f> p1(pts_.size()), p0r = pts_;
    for (size_t i = 0; i < pts_.size(); ++i) {
        const double qx = pts_[i].x - cx, qy = pts_[i].y - cy;
        p1[i] = cv::Point2f(float(ca * qx - sa * qy + cx), float(sa * qx + ca * qy + cy));
    }
    const cv::TermCriteria crit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, c.lk_iters, c.lk_eps);
    const cv::Size win(c.lk_win, c.lk_win);
    std::vector<uchar> st, st2;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_, img, pts_, p1, st, err, win, c.lk_levels, crit, cv::OPTFLOW_USE_INITIAL_FLOW);
    cv::calcOpticalFlowPyrLK(img, prev_, p1, p0r, st2, err, win, c.lk_levels, crit, cv::OPTFLOW_USE_INITIAL_FLOW);

    std::vector<Track> tracks;
    std::vector<cv::Point2f> q0, q1;
    for (size_t i = 0; i < pts_.size(); ++i) {
        const float fb = float(std::hypot(double(p0r[i].x) - pts_[i].x, double(p0r[i].y) - pts_[i].y));
        const bool inside = p1[i].x >= 0 && p1[i].y >= 0 && p1[i].x < w_img_ && p1[i].y < h_img_;
        if (st[i] == 1 && st2[i] == 1 && fb < c.fb_max_px && inside) {
            tracks.push_back(tracks_[i]); q0.push_back(pts_[i]); q1.push_back(p1[i]);
        }
    }
    const size_t N = tracks.size();
    const auto n0 = normalize(q0), n1 = normalize(q1);
    std::vector<cv::Point2d> n0r(N), d(N);
    std::vector<double> rho(N);
    std::vector<char> known(N), usable(N);
    int n_known = 0;
    const double rho_min2 = c.rho_min_motion_px * c.rho_min_motion_px;
    for (size_t i = 0; i < N; ++i) {
        n0r[i] = cv::Point2d(ca * n0[i].x - sa * n0[i].y, sa * n0[i].x + ca * n0[i].y);  // rotate by +ang
        d[i] = n1[i] - n0r[i];
        rho[i] = tracks[i].rho;
        known[i] = tracks[i].b * f_ * f_ >= rho_min2;
        n_known += known[i];
    }
    for (size_t i = 0; i < N; ++i) usable[i] = n_known >= kMinKnownTracks ? known[i] : 1;  // bootstrap: all at rho=1
    // Unknown depths are noisy partial estimates and the fit divides by them (translation came out ~5x too small).
    if (n_known < kMinKnownTracks) std::fill(rho.begin(), rho.end(), 1.0);

    cv::Vec4d x;
    std::vector<char> inl;
    const bool ok = fit(rho, n0r, d, usable, x, inl);
    result.n_tracks = int(N);
    std::vector<char> keep(N, 1);
    if (ok) {
        const cv::Vec2d w(x[0], x[1]);
        const double dpsi = x[2], sigma = x[3];
        const double w2 = w.dot(w);
        const bool moving = w2 * f_ * f_ > kMovingPx * kMovingPx;
        for (size_t i = 0; i < N; ++i) {
            auto& t = tracks[i];
            const cv::Vec2d dc(d[i].x + dpsi * n0r[i].y - sigma * n0r[i].x, d[i].y - dpsi * n0r[i].x - sigma * n0r[i].y);
            t.age += 1;
            if (moving) {
                t.a += dc.dot(w);
                t.b += w2;
                if (t.b > 0) t.rho = t.a / t.b;
            }
            const double res = cv::norm(dc - t.rho * w) * f_;
            if (known[i] && moving && res > kBadResFactor * c.inlier_px) t.bad += 1;
            else if (res < c.inlier_px) t.bad = 0;
            if (t.bad >= c.bad_frames_drop || (t.rho <= 0 && t.b * f_ * f_ > rho_min2)) keep[i] = 0;
        }
        int n_in = 0;
        std::vector<cv::Vec2d> r;
        for (size_t i = 0; i < N; ++i) {
            n_in += inl[i] && usable[i];
            if (!inl[i]) continue;
            r.emplace_back(d[i].x - (rho[i] * x[0] - x[2] * n0r[i].y + x[3] * n0r[i].x),
                           d[i].y - (rho[i] * x[1] + x[2] * n0r[i].x + x[3] * n0r[i].y));
        }
        cv::Matx22d cov = cv::Matx22d::eye() * kCovSmallN;
        if (r.size() > 2) {  // np.cov(r.T): sample covariance, ddof = 1
            cv::Vec2d m(0, 0);
            for (const auto& v : r) m += v;
            m *= 1.0 / double(r.size());
            cov = cv::Matx22d::zeros();
            for (const auto& v : r) { const cv::Vec2d e = v - m; cov += e * e.t(); }
            cov *= 1.0 / double(r.size() - 1);
        }
        cov *= 1.0 / std::max(n_in, 1);
        const double fl = (c.cov_floor_px / f_) * (c.cov_floor_px / f_);
        const cv::Matx22d floor_m(fl, 0, 0, fl);
        for (int k = 0; k < 4; ++k) cov.val[k] = std::max(cov.val[k], floor_m.val[k]);  // elementwise, like np.maximum
        const double q = std::clamp(double(n_in - c.q_inliers_zero) / double(c.q_inliers_full - c.q_inliers_zero), 0.0, 1.0);
        if (!bootstrapped_ && moving && n_in >= c.q_inliers_zero) bootstrapped_ = true;
        result.w = w;
        result.w_cov = cov;
        result.dpsi_cam = dpsi;
        result.sigma = sigma;
        result.n_inliers = n_in;
        result.q = q;
        result.scale_anomaly = std::abs(sigma) > c.scale_anomaly;
        result.valid = true;
        for (size_t i = 0; i < N; ++i) if (known[i]) result.rhos.push_back(tracks[i].rho);
    }

    tracks_.clear();
    pts_.clear();
    for (size_t i = 0; i < N; ++i) if (keep[i]) { tracks_.push_back(tracks[i]); pts_.push_back(q1[i]); }
    prev_ = img;
    replenish(img);
    return result;
}

}  // namespace deskmouse
