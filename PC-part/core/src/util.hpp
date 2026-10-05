// numpy-compatible helpers (median, percentile, interp), so the port matches the Python reference.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace deskmouse::util {

// np.median: mean of the two middle values for an even count.
inline double median(std::vector<double> v) {
    const size_t n = v.size();
    if (n == 0) return 0.0;
    std::sort(v.begin(), v.end());
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// np.percentile with the default linear interpolation.
inline double percentile(std::vector<double> v, double p) {
    const size_t n = v.size();
    if (n == 0) return 0.0;
    std::sort(v.begin(), v.end());
    const double idx = p / 100.0 * double(n - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = std::min(lo + 1, n - 1);
    return v[lo] + (idx - double(lo)) * (v[hi] - v[lo]);
}

// np.interp on sorted xs: clamps to the end values outside the range.
template <class Seq, class GetX, class GetY>
double interp(double x, const Seq& s, GetX gx, GetY gy) {
    if (s.empty()) return 0.0;
    if (x <= double(gx(s.front()))) return gy(s.front());
    if (x >= double(gx(s.back()))) return gy(s.back());
    auto it = std::upper_bound(s.begin(), s.end(), x, [&](double v, const auto& e) { return v < double(gx(e)); });
    const auto& b = *it;
    const auto& a = *(it - 1);
    const double xa = double(gx(a)), xb = double(gx(b));
    return xb == xa ? gy(b) : gy(a) + (x - xa) * (gy(b) - gy(a)) / (xb - xa);
}

}  // namespace deskmouse::util
