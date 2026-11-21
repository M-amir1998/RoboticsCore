#pragma once

/// @file time_parameterization.hpp
/// @brief Velocity profiles along a geometric path under speed, tangential-acceleration and
///        lateral-acceleration limits.
///
/// The classic two-pass algorithm: the speed limit at each sample is
///   v_lim(s) = min(v_max, sqrt(a_lat_max / |kappa(s)|)),
/// a forward pass enforces v_{i+1}^2 <= v_i^2 + 2 a_max ds (acceleration) and a backward pass
/// enforces v_i^2 <= v_{i+1}^2 + 2 a_max ds (braking). The result is the time-optimal profile
/// for a point mass under these (decoupled) constraints. Time stamps follow from
/// dt = 2 ds / (v_i + v_{i+1}) (constant acceleration per segment).

#include <vector>

namespace roboticscore::trajectory {

/// Velocity profile on arc-length samples.
struct SpeedProfile {
  std::vector<double> s;
  std::vector<double> v;
  std::vector<double> a;  ///< Tangential acceleration on [s_i, s_{i+1}] (last entry 0).
  std::vector<double> t;
  [[nodiscard]] double duration() const { return t.empty() ? 0.0 : t.back(); }
};

/// @throws std::invalid_argument for size mismatch, non-increasing s or non-positive limits.
[[nodiscard]] SpeedProfile compute_speed_profile(const std::vector<double>& s,
                                                 const std::vector<double>& curvature, double v_max,
                                                 double a_max, double a_lat_max,
                                                 double v_start = 0.0, double v_end = 0.0);

}  // namespace roboticscore::trajectory
