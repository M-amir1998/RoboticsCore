#pragma once

/// @file polynomial.hpp
/// @brief Polynomial trajectories in one dimension with boundary conditions.
///
///   cubic    p(0) = p0, p'(0) = v0, p(T) = p1, p'(T) = v1
///   quintic  additionally p''(0) = a0, p''(T) = a1 (minimum-jerk when v = a = 0 at both ends)
///
/// Multi-dimensional trajectories use one polynomial per axis with a shared duration.

#include <vector>

#include "roboticscore/core/types.hpp"

namespace roboticscore::trajectory {

/// p(t) = sum_k c_k t^k on t in [0, duration].
class Polynomial {
 public:
  Polynomial() = default;
  /// @throws std::invalid_argument if duration is not positive or no coefficients are given.
  Polynomial(std::vector<double> coefficients, double duration);

  /// k-th derivative at time t (t is not clamped, so evaluation can extrapolate).
  [[nodiscard]] double derivative(double t, int order) const;
  [[nodiscard]] double position(double t) const { return derivative(t, 0); }
  [[nodiscard]] double velocity(double t) const { return derivative(t, 1); }
  [[nodiscard]] double acceleration(double t) const { return derivative(t, 2); }
  [[nodiscard]] double jerk(double t) const { return derivative(t, 3); }

  [[nodiscard]] double duration() const noexcept { return duration_; }
  [[nodiscard]] const std::vector<double>& coefficients() const noexcept { return coefficients_; }

 private:
  std::vector<double> coefficients_{0.0};
  double duration_ = 1.0;
};

/// Constant-velocity segment from p0 to p1 over duration T.
[[nodiscard]] Polynomial linear_trajectory(double p0, double p1, double duration);

/// Cubic with position and velocity boundary conditions.
[[nodiscard]] Polynomial cubic_trajectory(double p0, double v0, double p1, double v1,
                                          double duration);

/// Quintic with position, velocity and acceleration boundary conditions.
[[nodiscard]] Polynomial quintic_trajectory(double p0, double v0, double a0, double p1, double v1,
                                            double a1, double duration);

/// Point-to-point motion with a trapezoidal velocity profile (triangular when the distance is
/// too short to reach v_max): bounded velocity and acceleration, discontinuous jerk.
class TrapezoidalProfile {
 public:
  /// @throws std::invalid_argument if v_max or a_max is not positive.
  TrapezoidalProfile(double start, double goal, double v_max, double a_max);

  [[nodiscard]] double position(double t) const;
  [[nodiscard]] double velocity(double t) const;
  [[nodiscard]] double acceleration(double t) const;
  [[nodiscard]] double duration() const noexcept { return t_total_; }
  [[nodiscard]] double peak_velocity() const noexcept { return v_peak_; }

 private:
  double start_;
  double direction_;
  double distance_;
  double a_max_;
  double v_peak_;
  double t_accel_;
  double t_cruise_;
  double t_total_;
};

}  // namespace roboticscore::trajectory
