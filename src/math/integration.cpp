#include "roboticscore/math/integration.hpp"

#include <cstddef>

namespace roboticscore::math {

double trapezoid(std::span<const double> xs, std::span<const double> ys) {
  detail::require(xs.size() == ys.size(), "trapezoid: xs and ys must have equal size");
  detail::require(xs.size() >= 2, "trapezoid: at least two samples are required");
  double area = 0.0;
  for (std::size_t i = 1; i < xs.size(); ++i) {
    area += 0.5 * (xs[i] - xs[i - 1]) * (ys[i] + ys[i - 1]);
  }
  return area;
}

}  // namespace roboticscore::math
