// Parity replay: runs the C++ engine over an input dump written by PC-part/proto/scripts/parity.py and prints
// events and per-frame front-end results in the same text format as the Python side.
//
// usage: deskmouse_replay <dump.bin> <out.txt> [n_frames_with_points]
//
// Dump (little-endian): "DMRP", u32 version=1, i32 width, i32 height, f64 K[9], f64 dist[5] (Camera2 order),
// f64 gauge_ratio, u32 n_params, n × (u16 len, name, f64 value), then records:
//   u8 0: IMU   i64 t, u8 kind (0 gyro, 1 accel), f64 x, y, z   (followed by poll(t), like mvp_replay.py)
//   u8 1: FRAME i64 t_sensor, i64 exposure, i64 skew, width*height bytes Y
//   u8 2: RESULT i64 t, f64 psi_prev, psi, wx, wy, sigma, u8 valid, i32 n_rhos, f64 rho15, rho_med
//         (a Python front-end result instead of a frame: checks everything after the front end exactly)
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "deskmouse/engine.hpp"

using namespace deskmouse;

template <class T>
static bool rd(std::ifstream& f, T& v) { return bool(f.read(reinterpret_cast<char*>(&v), sizeof(T))); }

int main(int argc, char** argv) {
    if (argc < 3) { std::cerr << "usage: deskmouse_replay <dump.bin> <out.txt>\n"; return 2; }
    std::ifstream f(argv[1], std::ios::binary);
    char magic[4];
    uint32_t ver = 0, n_params = 0;
    int32_t W = 0, H = 0;
    if (!f.read(magic, 4) || std::string(magic, 4) != "DMRP" || !rd(f, ver) || ver != 1) { std::cerr << "bad dump\n"; return 1; }
    rd(f, W); rd(f, H);
    cv::Matx33d K;
    for (int i = 0; i < 9; ++i) rd(f, K.val[i]);
    std::array<double, 5> dist{};
    for (auto& d : dist) rd(f, d);
    double ratio = 0;
    rd(f, ratio);
    rd(f, n_params);
    EngineConfig ec;
    FrontEndConfig fc;
    for (uint32_t i = 0; i < n_params; ++i) {
        uint16_t len = 0;
        rd(f, len);
        std::string name(len, '\0');
        f.read(name.data(), len);
        double v = 0;
        rd(f, v);
        if (!set_param(ec, fc, name, v)) std::cerr << "unknown param " << name << "\n";
    }
    Engine eng(K, dist, cv::Size(W, H), ec, fc, ratio);
    FILE* out = std::fopen(argv[2], "w");
    cv::Mat img(H, W, CV_8UC1);
    int frames = 0;
    const int pts_frames = argc > 3 ? std::atoi(argv[3]) : 0;  // debug: dump tracked points for the first frames
    double fe_ms = 0, fe_max = 0;
    auto dump_events = [&] {
        for (const auto& e : eng.drain())
            std::fprintf(out, "E %lld %s %d %d %d\n", (long long)e.t_ns, event_name(e.kind), e.dx, e.dy, e.wheel);
    };
    uint8_t type;
    while (rd(f, type)) {
        if (type == 0) {
            int64_t t; uint8_t kind; double x, y, z;
            rd(f, t); rd(f, kind); rd(f, x); rd(f, y); rd(f, z);
            eng.on_imu(t, kind, x, y, z);
            eng.poll(t);
        } else if (type == 1) {
            int64_t t, exp, skew;
            rd(f, t); rd(f, exp); rd(f, skew);
            f.read(reinterpret_cast<char*>(img.data), int64_t(W) * H);
            const auto t0 = std::chrono::steady_clock::now();
            eng.on_frame(img, t, exp, skew);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            fe_ms += ms; fe_max = std::max(fe_max, ms); ++frames;
            const auto& r = eng.last_result;
            std::fprintf(out, "F %lld %d %.9g %.9g %.9g %d %d\n", (long long)r.t_ns, int(r.valid), r.w[0], r.w[1], r.sigma,
                         r.n_tracks, r.n_inliers);
            if (frames <= pts_frames)
                for (const auto& p : eng.front_end().points()) std::fprintf(out, "P %.4f %.4f\n", p.x, p.y);
        } else if (type == 2) {
            int64_t t; double pp, p, wx, wy, sg, r15, rmed; uint8_t valid; int32_t nr;
            rd(f, t); rd(f, pp); rd(f, p); rd(f, wx); rd(f, wy); rd(f, sg); rd(f, valid); rd(f, nr); rd(f, r15); rd(f, rmed);
            eng.on_result(t, pp, p, wx, wy, sg, valid != 0, nr, r15, rmed);
            ++frames;
        } else {
            std::cerr << "bad record type " << int(type) << "\n";
            return 1;
        }
        dump_events();
    }
    const auto gr = eng.gauge_ratio();
    std::fprintf(out, "S frames %d taps %d scale_updates %d lifts %d lam %.9g gauge_ratio %.9g\n", eng.stats["frames"],
                 eng.stats["taps"], eng.stats["scale_updates"], eng.stats["lifts"], eng.scale().value_or(0), gr.value_or(0));
    std::fclose(out);
    std::printf("frames %d, engine per frame: mean %.2f ms, max %.2f ms\n", frames, frames ? fe_ms / frames : 0.0, fe_max);
    return 0;
}
