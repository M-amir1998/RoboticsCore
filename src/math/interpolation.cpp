#include "roboticscore/math/interpolation.hpp"

#include <algorithm>
#include <cstddef>

#include "roboticscore/core/error.hpp"

namespace roboticscore::math {

std::size_t find_segment(std::span<const double> xs, double x) {
  detail::require(xs.size() >= 2, "find_segment: table needs at least two points");
  // First element strictly greater than x; the segment starts one before it.
  const auto it = std::upper_bound(xs.begin(), xs.end(), x);
  const auto pos = static_cast<std::size_t>(it - xs.begin());
  if (pos == 0) {
    return 0;
  }
  return std::min(pos - 1, xs.size() - 2);
}

double interp1(std::span<const double> xs, std::span<const double> ys, double x,
               Extrapolation mode) {
  detail::require(xs.size() >= 2, "interp1: table needs at least two points");
  detail::require(xs.size() == ys.size(), "interp1: xs and ys must have equal size");
  for (std::size_t i = 1; i < xs.size(); ++i) {
    detail::require(xs[i] > xs[i - 1], "interp1: xs must be strictly increasing");
  }

  if (mode == Extrapolation::kClamp) {
    if (x <= xs.front()) {
      return ys.front();
    }
    if (x >= xs.back()) {
      return ys.back();
    }
  }
  const std::size_t i = find_segment(xs, x);
  const double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
  return ys[i] + t * (ys[i + 1] - ys[i]);
}

}  // namespace roboticscore::math
