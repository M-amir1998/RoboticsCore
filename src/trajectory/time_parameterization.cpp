#include "roboticscore/trajectory/time_parameterization.hpp"

#include <algorithm>
#include <cmath>

#include "roboticscore/core/error.hpp"

namespace roboticscore::trajectory {

SpeedProfile compute_speed_profile(const std::vector<double>& s,
                                   const std::vector<double>& curvature, double v_max, double a_max,
                                   double a_lat_max, double v_start, double v_end) {
  detail::require(s.size() >= 2 && s.size() == curvature.size(),
                  "compute_speed_profile: need >= 2 samples with matching curvature");
  detail::require(v_max > 0.0 && a_max > 0.0 && a_lat_max > 0.0,
                  "compute_speed_profile: limits must be positive");
  detail::require(v_start >= 0.0 && v_end >= 0.0,
                  "compute_speed_profile: boundary speeds must be non-negative");
  for (std::size_t i = 1; i < s.size(); ++i) {
    detail::require(s[i] > s[i - 1], "compute_speed_profile: s must be strictly increasing");
  }
  const std::size_t n = s.size();
  SpeedProfile p;
  p.s = s;
  p.v.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double k = std::abs(curvature[i]);
    p.v[i] = k > 1e-9 ? std::min(v_max, std::sqrt(a_lat_max / k)) : v_max;
  }
  p.v.front() = std::min(p.v.front(), v_start);
  p.v.back() = std::min(p.v.back(), v_end);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double ds = s[i + 1] - s[i];
    p.v[i + 1] = std::min(p.v[i + 1], std::sqrt(p.v[i] * p.v[i] + 2.0 * a_max * ds));
  }
  for (std::size_t i = n - 1; i-- > 0;) {
    const double ds = s[i + 1] - s[i];
    p.v[i] = std::min(p.v[i], std::sqrt(p.v[i + 1] * p.v[i + 1] + 2.0 * a_max * ds));
  }
  p.t.assign(n, 0.0);
  p.a.assign(n, 0.0);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double ds = s[i + 1] - s[i];
    const double vsum = p.v[i] + p.v[i + 1];
    // Both ends at rest cannot happen inside a valid profile except for zero-length paths;
    // guard with the acceleration-limited time of a rest-to-rest segment.
    const double dt = vsum > 1e-12 ? 2.0 * ds / vsum : 2.0 * std::sqrt(ds / a_max);
    p.t[i + 1] = p.t[i] + dt;
    p.a[i] = (p.v[i + 1] * p.v[i + 1] - p.v[i] * p.v[i]) / (2.0 * ds);
  }
  return p;
}

}  // namespace roboticscore::trajectory
