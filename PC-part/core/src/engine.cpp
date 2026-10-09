#include "deskmouse/engine.hpp"

#include <cmath>

#include "util.hpp"

namespace deskmouse {

namespace {
// Same literals as engine.py.
constexpr size_t kGyroHistory = 1500, kRecent = 64, kAccHistory = 3000, kJerkHistory = 500, kBiasInit = 100;
constexpr size_t kBiasMean = 50, kJerkEvery = 50, kGaugeKeep = 120, kGaugeMin = 30;
constexpr double kImuRateHz = 500.0;         // bias EMA step and IMU lead integration assume this rate
constexpr int64_t kTapQuietNs = 300'000'000;  // jerk samples this long after a tap don't train the threshold
constexpr double kStrokeMinS = 0.1, kStrokeMarginS = 0.06, kStrokeMinCos = 0.9;
constexpr size_t kStrokeMinSamples = 20;
constexpr int kMinRhos = 8, kAnomalyFrames = 2, kMinStillSamples = 5;
constexpr double kInchM = 0.0254, kPi = 3.14159265358979323846;

cv::Vec2d rot(double a, const cv::Vec2d& v) {
    const double c = std::cos(a), s = std::sin(a);
    return {c * v[0] - s * v[1], s * v[0] + c * v[1]};
}
double deg2rad(double d) { return d * kPi / 180.0; }

template <class D, class T>
void push_capped(D& d, T&& v, size_t cap) {
    d.push_back(std::forward<T>(v));
    while (d.size() > cap) d.pop_front();
}
}  // namespace

const char* event_name(EventKind k) {
    switch (k) {
        case EventKind::Move: return "move";
        case EventKind::Left: return "left";
        case EventKind::Right: return "right";
        case EventKind::Wheel: return "wheel";
        case EventKind::Lift: return "lift";
        case EventKind::Land: return "land";
    }
    return "?";
}

static FrontEndConfig with_rot_sign(FrontEndConfig f, double s) { f.rot_sign = s; return f; }

Engine::Engine(const cv::Matx33d& K, const std::array<double, 5>& dist, cv::Size size, const EngineConfig& cfg,
               FrontEndConfig fe_cfg, double gauge_ratio)
    : cfg_(cfg), fe_(K, dist, size, with_rot_sign(fe_cfg, cfg.rot_sign)), tap_thr_(cfg.tap_thr_min),
      kf_(cfg.kf_acc_noise, cfg.kf_scale_noise, cfg.kf_gate, cfg.kf_resync_frames, cfg.kf_learn_scale),
      gauge_ratio0_(gauge_ratio) {
    const cv::Matx22d M(cfg.M_ci[0], cfg.M_ci[1], cfg.M_ci[2], cfg.M_ci[3]);
    Minv_ = M.inv();
    r_cam_ = cv::Vec2d(cfg.r_cam[0], cfg.r_cam[1]);
    stats = {{"frames", 0}, {"taps", 0}, {"scale_updates", 0}, {"lifts", 0}, {"shocks", 0}};
}

// ------------------------------------------------------------------ IMU
double Engine::psi_at(int64_t t_ns) const {
    if (gyro_t_.empty()) return 0.0;
    return -util::interp(double(t_ns), gyro_t_, [](const auto& e) { return e.first; }, [](const auto& e) { return e.second; });
}

void Engine::on_imu(int64_t t_ns, int kind, double x, double y, double z) {
    const auto& c = cfg_;
    if (kind == Gyro) {
        const cv::Vec3d g(x, y, z);
        // bias: set from the first detected still window, then EMA while still (§6.1)
        push_capped(bias_init_, std::make_pair(t_ns, g), kBiasInit);
        const cv::Vec3d gc = g - gyro_bias_;
        if (last_gyro_t_) {
            const double dt = double(t_ns - *last_gyro_t_) / 1e9;
            Zcum_ += gc[2] * dt;
            tilt_ += cv::Vec2d(gc[0], gc[1]) * dt;
        }
        last_gyro_t_ = t_ns;
        push_capped(gyro_t_, std::make_pair(t_ns, Zcum_), kGyroHistory);
        push_capped(recent_gyro_, std::make_pair(t_ns, cv::norm(gc)), kRecent);
        if (still_) {
            if (!bias_ok_) {
                cv::Vec3d m(0, 0, 0);
                const size_t n = std::min(kBiasMean, bias_init_.size());
                for (size_t i = bias_init_.size() - n; i < bias_init_.size(); ++i) m += bias_init_[i].second;
                gyro_bias_ = m * (1.0 / double(n));
                bias_ok_ = true;
            } else {
                const double a = 1.0 / kImuRateHz / c.gyro_bias_tau_s;
                gyro_bias_ = (1 - a) * gyro_bias_ + a * g;
            }
        }
        scroll_update(t_ns, gc[2]);
        lift_update(t_ns);
    } else if (kind == Accel) {
        const Acc s{t_ns, -x, y};  // body X, Y specific force
        if (c.kf) {
            cv::Vec2d a(0, 0);
            if (rest_acc_) a = rot(-Zcum_, cv::Vec2d(s.bx - (*rest_acc_)[0], s.by - (*rest_acc_)[1]));
            kf_.predict(t_ns, a);
        }
        push_capped(acc_buf_, s, kAccHistory);
        push_capped(recent_acc_, s, kRecent);
        shock_update(t_ns, s.bx, s.by);
        tap_update(t_ns, z);
        still_update(t_ns);
    }
}

void Engine::still_update(int64_t t_ns) {
    const auto& c = cfg_;
    const double win = c.still_ms * 1e6;
    std::vector<double> g;
    for (const auto& [t, v] : recent_gyro_) if (double(t) > double(t_ns) - win) g.push_back(v);
    if (!bias_ok_) {  // bias unknown: judge by the spread of the raw rate instead of its size
        std::vector<cv::Vec3d> raw;
        for (const auto& [t, v] : bias_init_) if (double(t) > double(t_ns) - win) raw.push_back(v);
        g.clear();
        if (raw.size() > kMinStillSamples) {
            cv::Vec3d m(0, 0, 0);
            for (const auto& v : raw) m += v;
            m *= 1.0 / double(raw.size());
            double mx = 0;
            for (const auto& v : raw) for (int k = 0; k < 3; ++k) mx = std::max(mx, std::abs(v[k] - m[k]));
            g.push_back(mx);
        }
    }
    std::vector<cv::Vec2d> a;
    for (const auto& s : recent_acc_) if (double(s.t) > double(t_ns) - win) a.emplace_back(s.bx, s.by);
    cv::Vec2d mean(0, 0);
    for (const auto& v : a) mean += v;
    if (!a.empty()) mean *= 1.0 / double(a.size());
    bool imu_still = false;
    if (!g.empty() && *std::max_element(g.begin(), g.end()) < c.still_gyro && a.size() > kMinStillSamples) {
        cv::Vec2d var(0, 0);  // np.std, ddof = 0
        for (const auto& v : a) { const cv::Vec2d e = v - mean; var += cv::Vec2d(e[0] * e[0], e[1] * e[1]); }
        var *= 1.0 / double(a.size());
        imu_still = std::max(std::sqrt(var[0]), std::sqrt(var[1])) < c.still_acc_std;
    }
    if (imu_still) { if (!imu_still_since_) imu_still_since_ = t_ns; }
    else imu_still_since_.reset();
    const bool was = still_;
    still_ = imu_still_since_ && double(t_ns - *imu_still_since_) >= win && last_flow_px_ < c.still_flow_px;
    if (still_) {
        if (!a.empty()) rest_acc_ = mean;
        tilt_ = cv::Vec2d(0, 0);
    }
    if (still_ && !was) stroke_end(t_ns);
    if (!still_ && was) {
        stroke_start_ = t_ns;
        stroke_vis_ = cv::Vec2d(0, 0);
    }
}

void Engine::shock_update(int64_t t_ns, double bx, double by) {
    const auto& c = cfg_;
    if (last_axy_ && std::hypot(bx - (*last_axy_)[0], by - (*last_axy_)[1]) > c.shock_jerk) {
        if (double(t_ns) - c.shock_suppress_before_ms * 1e6 > double(shock_until_)) {  // new window, else extend
            shock_from_ = int64_t(double(t_ns) - c.shock_suppress_before_ms * 1e6);
            stats["shocks"] += 1;
        }
        shock_until_ = int64_t(double(t_ns) + c.shock_suppress_after_ms * 1e6);
    }
    last_axy_ = cv::Vec2d(bx, by);
}

void Engine::tap_update(int64_t t_ns, double az) {
    const auto& c = cfg_;
    if (!last_az_) { last_az_ = az; return; }
    const double j = std::abs(az - *last_az_);
    last_az_ = az;
    if (t_ns - last_tap_t_ > kTapQuietNs) {
        push_capped(jerk_, j, kJerkHistory);
        if (jerk_.size() % kJerkEvery == 0) {
            std::vector<double> arr(jerk_.begin(), jerk_.end());
            const double med = util::median(arr);
            for (auto& v : arr) v = std::abs(v - med);
            tap_thr_ = std::max(c.tap_thr_min, med + c.tap_mad_k * util::median(arr));
        }
    }
    if (j > tap_thr_ && double(t_ns - last_tap_t_) > c.tap_refractory_ms * 1e6 && !lifted_ && !scroll_) {
        last_tap_t_ = t_ns;
        stats["taps"] += 1;
        suppress_from_ = int64_t(double(t_ns) - c.tap_suppress_before_ms * 1e6);
        suppress_until_ = int64_t(double(t_ns) + c.tap_suppress_after_ms * 1e6);
        const bool dbl = pending_tap_t_ && double(t_ns - *pending_tap_t_) < c.double_tap_ms * 1e6;
        if (c.instant_left) {
            if (dbl) { events_.push_back({t_ns, EventKind::Right}); pending_tap_t_.reset(); }
            else { events_.push_back({t_ns, EventKind::Left}); pending_tap_t_ = t_ns; }
            return;
        }
        if (dbl) { events_.push_back({t_ns, EventKind::Right}); pending_tap_t_.reset(); }
        else pending_tap_t_ = t_ns;
    }
}

void Engine::poll(int64_t t_ns) {
    if (!pending_tap_t_ || double(t_ns - *pending_tap_t_) < cfg_.double_tap_ms * 1e6) return;
    if (!cfg_.instant_left) events_.push_back({t_ns, EventKind::Left});
    pending_tap_t_.reset();
}

void Engine::scroll_update(int64_t t_ns, double wz) {
    const auto& c = cfg_;
    if (!c.scroll_enabled) { scroll_ = false; return; }
    if (!scroll_) {
        const bool calibrated = stats["scale_updates"] > 0;
        if (bias_ok_ && std::abs(wz) > c.scroll_omega && (!calibrated || last_ref_speed_ < c.scroll_max_ref_speed)) {
            if (!scroll_cand_since_) scroll_cand_since_ = t_ns;
            if (double(t_ns - *scroll_cand_since_) > c.scroll_enter_ms * 1e6) {
                scroll_ = true;
                scroll_acc_ = 0;
                scroll_psi_last_ = -Zcum_;
                scroll_quiet_since_.reset();
            }
        } else {
            scroll_cand_since_.reset();
        }
        return;
    }
    const double psi = -Zcum_;
    scroll_acc_ += psi - scroll_psi_last_;
    scroll_psi_last_ = psi;
    const double step = deg2rad(c.scroll_step_deg);
    const int n = int(scroll_acc_ / step);  // truncates toward zero, like Python int()
    if (n) {
        scroll_acc_ -= n * step;
        events_.push_back({t_ns, EventKind::Wheel, 0, 0, int(c.scroll_sign * n)});
    }
    if (std::abs(wz) < c.scroll_exit_omega) {
        if (!scroll_quiet_since_) scroll_quiet_since_ = t_ns;
        if (double(t_ns - *scroll_quiet_since_) > c.scroll_exit_ms * 1e6) { scroll_ = false; scroll_cand_since_.reset(); }
    } else {
        scroll_quiet_since_.reset();
    }
}

void Engine::lift_update(int64_t t_ns) {
    const double tilt = cv::norm(tilt_) * 180.0 / kPi;
    if (!lifted_ && tilt > cfg_.lift_tilt_deg) {
        lifted_ = true;
        stats["lifts"] += 1;
        events_.push_back({t_ns, EventKind::Lift});
    } else if (lifted_ && still_) {
        lifted_ = false;
        events_.push_back({t_ns, EventKind::Land});
    }
}

// ------------------------------------------------------------------ strokes / scale
void Engine::stroke_end(int64_t t_ns) {
    const auto& c = cfg_;
    if (!stroke_start_ || !rest_acc_) return;
    const double dur = double(t_ns - *stroke_start_) / 1e9;
    const cv::Vec2d vis = stroke_vis_;
    stroke_start_.reset();
    if (dur > c.stroke_max_s || dur < kStrokeMinS || cv::norm(vis) == 0) return;
    std::vector<const Acc*> arr;
    const double lo = double(t_ns) - (dur + kStrokeMarginS) * 1e9;
    for (const auto& s : acc_buf_) if (lo <= double(s.t) && s.t <= t_ns) arr.push_back(&s);
    if (arr.size() < kStrokeMinSamples) return;
    const size_t n = arr.size();
    std::vector<double> tt(n);
    std::vector<cv::Vec2d> aw(n), v(n);
    for (size_t i = 0; i < n; ++i) {
        tt[i] = double(arr[i]->t) / 1e9;
        const cv::Vec2d acc(arr[i]->bx - (*rest_acc_)[0], arr[i]->by - (*rest_acc_)[1]);
        aw[i] = rot(psi_at(arr[i]->t), acc);  // body accel into world by yaw
    }
    v[0] = cv::Vec2d(0, 0);
    for (size_t i = 1; i < n; ++i) v[i] = v[i - 1] + (aw[i] + aw[i - 1]) * 0.5 * (tt[i] - tt[i - 1]);
    const cv::Vec2d v_end = v[n - 1];
    for (size_t i = 0; i < n; ++i) v[i] -= v_end * ((tt[i] - tt[0]) / (tt[n - 1] - tt[0]));  // ZUPT at both ends
    cv::Vec2d p(0, 0);
    double peak = 0;
    for (size_t i = 0; i < n; ++i) {
        if (i) p += (v[i] + v[i - 1]) * 0.5 * (tt[i] - tt[i - 1]);
        peak = std::max(peak, cv::norm(v[i]));
    }
    const double L = cv::norm(p);
    if (L < c.stroke_min_m || peak < c.stroke_min_peak_speed) return;
    const double cosang = p.dot(vis) / (L * cv::norm(vis));
    if (cosang < kStrokeMinCos) return;
    const double lam_meas = L / cv::norm(vis);
    if (calibrating_) {
        // calibration (new ceiling): the scale is the median of this run's strokes only, the old prior is ignored
        cal_strokes_.push_back(lam_meas);
        lam_ = util::median(cal_strokes_);
        if (c.kf) kf_.set_scale(*lam_, c.kf_scale_sigma_cal);
    } else if (c.kf && kf_.has_scale()) {
        kf_.scale_measurement(lam_meas, c.kf_stroke_sigma);
        lam_ = kf_.lam();
    } else {
        const double a = stats["scale_updates"] == 0 ? 1.0 : c.scale_alpha;
        lam_ = lam_ ? (1 - a) * *lam_ + a * lam_meas : lam_meas;
    }
    stats["scale_updates"] += 1;
}

// ------------------------------------------------------------------ frames
void Engine::on_frame(const cv::Mat& y, int64_t t_sensor_ns, int64_t exposure_ns, int64_t skew_ns) {
    const auto& c = cfg_;
    const int64_t t = t_sensor_ns + exposure_ns / 2 + std::max<int64_t>(skew_ns, 0) / 2 + c.clock_offset_ns;
    const double Z = -psi_at(t);
    const double dZ = prev_Z_ ? Z - *prev_Z_ : 0.0;
    const double psi_prev = prev_Z_ ? -*prev_Z_ : -Z;
    prev_Z_ = Z;
    const double psi = -Z;
    last_result = fe_.process(y, t, dZ);
    const auto& res = last_result;
    const int n_rhos = int(res.rhos.size());
    const double rho15 = n_rhos ? util::percentile(res.rhos, c.ceiling_percentile) : 0.0;
    const double rho_med = n_rhos ? util::median(res.rhos) : 1.0;
    on_result(t, psi_prev, psi, res.w[0], res.w[1], res.sigma, res.valid, n_rhos, rho15, rho_med);
}

void Engine::on_result(int64_t t, double psi_prev, double psi, double wx, double wy, double sigma, bool valid, int n_rhos,
                       double rho15, double rho_med) {
    const auto& c = cfg_;
    stats["frames"] += 1;
    if (!valid) return;
    const cv::Vec2d w(wx, wy);
    last_flow_px_ = cv::norm(w) * rho_med * fe_.focal();
    // scale: ceiling gauge until the first stroke calibration
    if (n_rhos >= kMinRhos && rho15 > 0) {
        lam_gauge_.push_back(1.0 / (c.ceiling_h * rho15));
        if (lam_gauge_.size() > kGaugeKeep) lam_gauge_.erase(lam_gauge_.begin(), lam_gauge_.end() - kGaugeKeep);
    }
    if (!lam_ && lam_gauge_.size() >= kGaugeMin) lam_ = (gauge_ratio0_ ? gauge_ratio0_ : 1.0) / util::median(lam_gauge_);
    // lift from vision scale change
    anomaly_frames_ = std::abs(sigma) > c.lift_scale_anomaly ? anomaly_frames_ + 1 : 0;
    if (anomaly_frames_ >= kAnomalyFrames && !lifted_) {
        lifted_ = true;
        stats["lifts"] += 1;
        events_.push_back({t, EventKind::Lift});
    }

    const cv::Vec2d d_rel_body = -(Minv_ * w);
    const cv::Vec2d d_rel_world = rot(psi, d_rel_body);
    if (!still_) stroke_vis_ += d_rel_world;
    if (c.kf) {
        kf_frame(t, psi_prev, psi, d_rel_world, rho_med);
        return;
    }
    if (!lam_) return;
    const cv::Vec2d lens = *lam_ * d_rel_world;
    // reference point = lens - R(ψ) r_cam
    const cv::Vec2d lever = rot(psi, r_cam_) - rot(psi_prev, r_cam_);
    cv::Vec2d ref = lens - lever;
    double dt = last_frame_t_ ? std::clamp(double(t - *last_frame_t_) / 1e9, 1.0 / 120, 0.1) : 1.0 / 60;
    last_frame_t_ = t;
    double speed = cv::norm(ref) / dt;
    last_ref_speed_ = speed;
    if (scroll_ || (suppress_from_ <= t && t <= suppress_until_)) return;  // lead left untouched while suppressed
    if (shock_from_ <= t && t <= shock_until_ && !lifted_) {
        // knock: hold the motion and release the net once the image stops ringing
        held_ += ref;
        held_dt_ += dt;
        return;
    }
    bool released = false;
    if (held_dt_ > 0) {
        if (!lifted_) {
            // a stroke that ended in the knock still gets its motion; a still frame adds none
            ref = still_ ? held_ : ref + held_;
            dt = still_ ? held_dt_ : dt + held_dt_;
            speed = cv::norm(ref) / dt;
            released = true;
        }
        held_ = cv::Vec2d(0, 0);
        held_dt_ = 0;
    }
    if ((still_ || lifted_) && !released) {
        emit(t, -lead_);  // give back whatever lead is left, then nothing
        lead_ = v_ref_ = cv::Vec2d(0, 0);
        return;
    }
    output(t, ref, dt, speed, psi);
}

// Reference-point displacement (world, m) -> output frame, pointer acceleration, emit.
void Engine::output(int64_t t, const cv::Vec2d& ref, double dt, double speed, double psi) {
    const auto& c = cfg_;
    const cv::Vec2d out = to_output(ref, psi);
    const double s = std::clamp((speed - c.v_lo) / (c.v_hi - c.v_lo), 0.0, 1.0);
    const double gain = 1 + (c.g_max - 1) * s * s * (3 - 2 * s);
    const double k = c.dpi / kInchM * gain;
    const cv::Vec2d counts = out * k;
    emit(t, counts + lead_step(t, counts / dt, psi, k));
}

// Kalman path (engine.py _kf_frame): the camera corrects the filter; the cursor follows the filter's position.
void Engine::kf_frame(int64_t t, double psi_prev, double psi, const cv::Vec2d& d_rel_world, double rho_med) {
    const auto& c = cfg_;
    const auto t_prev = last_frame_t_;
    const double dt = t_prev ? std::clamp(double(t - *t_prev) / 1e9, 1.0 / 120, 0.1) : 1.0 / 60;
    last_frame_t_ = t;
    if (!lam_) return;
    if (!kf_.has_scale()) kf_.set_scale(*lam_, c.kf_scale_sigma0);
    if (t_prev && !lifted_) {
        const double sig_px = c.kf_cam_sigma_px + c.kf_cam_sigma_rel * last_flow_px_;
        kf_.camera(*t_prev, t, d_rel_world, sig_px / (fe_.focal() * std::max(rho_med, 1e-6)));
    }
    lam_ = kf_.lam();
    if (!kf_.ready()) return;
    const int64_t tq = c.kf_predict_now ? *kf_.t() : t;
    cv::Vec2d p = kf_.position_at(tq);
    const bool landed = kf_was_lifted_ && !lifted_;
    kf_was_lifted_ = lifted_;
    if (landed) {
        kf_.zero_velocity();
        p = kf_.position_at(c.kf_predict_now ? *kf_.t() : t);
    }
    if (!kf_out_ || lifted_ || landed) {
        kf_out_ = kf_emitted_ = p;
        return;
    }
    if (scroll_ || (suppress_from_ <= t && t <= suppress_until_)) {
        kf_out_ = kf_emitted_ = p;  // suppressed motion is dropped, not caught up later
        return;
    }
    if (still_) {
        const cv::Vec2d d = p - *kf_out_;
        const double n = cv::norm(d);
        const double leash = c.kf_leash_mm / 1000;
        if (n > leash) *kf_out_ += d * (1 - leash / n);
    } else {
        kf_out_ = p;
    }
    const cv::Vec2d move = *kf_out_ - *kf_emitted_;
    kf_emitted_ = kf_out_;
    if (move[0] == 0 && move[1] == 0) return;
    const cv::Vec2d lever = rot(psi, r_cam_) - rot(psi_prev, r_cam_);
    const cv::Vec2d ref = still_ ? move : move - lever;
    const double speed = cv::norm(ref) / dt;
    last_ref_speed_ = speed;
    output(t, ref, dt, speed, psi);
}

cv::Vec2d Engine::to_output(const cv::Vec2d& v_world, double psi) const {
    const auto& c = cfg_;
    const cv::Vec2d v = c.world_aligned ? v_world : rot(-psi, v_world);
    const cv::Vec2d m = rot(deg2rad(c.mount_yaw_deg), v);
    return {m[0], m[1] * c.aspect};
}

void Engine::emit(int64_t t, const cv::Vec2d& counts) {
    const auto& c = cfg_;
    pos_raw_ += counts;
    if (c.smooth) {
        const double dt = last_emit_t_ ? std::clamp(double(t - *last_emit_t_) / 1e9, 1.0 / 240, 0.1) : 1.0 / 60;
        auto alpha = [dt](double fc) { return 1.0 / (1.0 + 1.0 / (2 * kPi * fc * dt)); };
        const double v = cv::norm(pos_raw_ - pos_f_) / dt;
        speed_f_ += alpha(c.oe_d_cutoff_hz) * (v - speed_f_);
        pos_f_ += alpha(c.oe_min_cutoff_hz + c.oe_beta * speed_f_) * (pos_raw_ - pos_f_);
    } else {
        pos_f_ = pos_raw_;
    }
    last_emit_t_ = t;
    const cv::Vec2d n(std::trunc(pos_f_[0] - pos_sent_[0]), std::trunc(pos_f_[1] - pos_sent_[1]));
    pos_sent_ += n;
    // +X → right, +Y (forward) → up (screen dy negative)
    if (n[0] != 0 || n[1] != 0) events_.push_back({t, EventKind::Move, int(n[0]), int(-n[1])});
}

cv::Vec2d Engine::lead_step(int64_t t, const cv::Vec2d& v_counts, double psi, double k) {
    const auto& c = cfg_;
    if (c.latency_comp_ms <= 0) return {0, 0};
    v_ref_ = c.lead_vel_alpha * v_counts + (1 - c.lead_vel_alpha) * v_ref_;  // counts/s
    cv::Vec2d v = v_ref_;
    if (c.lead_use_imu && rest_acc_ && !acc_buf_.empty()) {
        cv::Vec2d a_body(0, 0);
        bool any = false;
        for (const auto& s : acc_buf_)
            if (s.t > t) { a_body += cv::Vec2d(s.bx, s.by) - *rest_acc_; any = true; }
        if (any) v += to_output(rot(psi, a_body / kImuRateHz), psi) * k;  // Δv since the frame, m/s
    }
    cv::Vec2d target = v * (c.latency_comp_ms / 1000);
    const double lim = c.lead_max_m * c.dpi / kInchM;
    const double nrm = cv::norm(target);
    if (nrm > lim) target *= lim / nrm;
    const cv::Vec2d d = target - lead_;
    lead_ = target;
    return d;
}

std::optional<double> Engine::gauge_ratio() const {
    if (!lam_ || *lam_ == 0 || lam_gauge_.empty() || stats.at("scale_updates") == 0) return std::nullopt;
    return *lam_ * util::median(lam_gauge_);
}

std::vector<Event> Engine::drain() {
    std::vector<Event> ev;
    ev.swap(events_);
    return ev;
}

}  // namespace deskmouse
