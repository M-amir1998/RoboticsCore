#include "roboticscore/trajectory/polynomial.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/LU>

#include "roboticscore/core/error.hpp"

namespace roboticscore::trajectory {

Polynomial::Polynomial(std::vector<double> coefficients, double duration)
    : coefficients_(std::move(coefficients)), duration_(duration) {
  detail::require(!coefficients_.empty(), "Polynomial: at least one coefficient is required");
  detail::require(duration > 0.0, "Polynomial: duration must be positive");
}

double Polynomial::derivative(double t, int order) const {
  detail::require(order >= 0, "Polynomial::derivative: order must be non-negative");
  // Horner evaluation of the differentiated coefficients.
  double result = 0.0;
  const auto n = static_cast<int>(coefficients_.size());
  for (int k = n - 1; k >= order; --k) {
    double factor = 1.0;
    for (int j = 0; j < order; ++j) {
      factor *= static_cast<double>(k - j);
    }
    result = result * t + factor * coefficients_[static_cast<std::size_t>(k)];
  }
  return result;
}

Polynomial linear_trajectory(double p0, double p1, double duration) {
  detail::require(duration > 0.0, "linear_trajectory: duration must be positive");
  return Polynomial({p0, (p1 - p0) / duration}, duration);
}

Polynomial cubic_trajectory(double p0, double v0, double p1, double v1, double duration) {
  detail::require(duration > 0.0, "cubic_trajectory: duration must be positive");
  const double t = duration;
  const double a2 = (3.0 * (p1 - p0) - (2.0 * v0 + v1) * t) / (t * t);
  const double a3 = (2.0 * (p0 - p1) + (v0 + v1) * t) / (t * t * t);
  return Polynomial({p0, v0, a2, a3}, duration);
}

Polynomial quintic_trajectory(double p0, double v0, double a0, double p1, double v1, double a1,
                              double duration) {
  detail::require(duration > 0.0, "quintic_trajectory: duration must be positive");
  const double t = duration;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;
  const double c0 = p0;
  const double c1 = v0;
  const double c2 = 0.5 * a0;
  Mat3 m;
  m << t3, t4, t5,             //
      3 * t2, 4 * t3, 5 * t4,  //
      6 * t, 12 * t2, 20 * t3;
  const Vec3 rhs(p1 - c0 - c1 * t - c2 * t2, v1 - c1 - 2.0 * c2 * t, a1 - 2.0 * c2);
  const Vec3 c = m.partialPivLu().solve(rhs);
  return Polynomial({c0, c1, c2, c(0), c(1), c(2)}, duration);
}

TrapezoidalProfile::TrapezoidalProfile(double start, double goal, double v_max, double a_max)
    : start_(start),
      direction_(goal >= start ? 1.0 : -1.0),
      distance_(std::abs(goal - start)),
      a_max_(a_max) {
  detail::require(v_max > 0.0 && a_max > 0.0, "TrapezoidalProfile: limits must be positive");
  // Triangular profile if v_max cannot be reached within half the distance.
  if (v_max * v_max / a_max > distance_) {
    v_peak_ = std::sqrt(distance_ * a_max);
    t_accel_ = v_peak_ / a_max;
    t_cruise_ = 0.0;
  } else {
    v_peak_ = v_max;
    t_accel_ = v_max / a_max;
    t_cruise_ = (distance_ - v_max * t_accel_) / v_max;
  }
  t_total_ = 2.0 * t_accel_ + t_cruise_;
}

double TrapezoidalProfile::position(double t) const {
  t = std::clamp(t, 0.0, t_total_);
  double s = 0.0;
  if (t < t_accel_) {
    s = 0.5 * a_max_ * t * t;
  } else if (t < t_accel_ + t_cruise_) {
    s = 0.5 * a_max_ * t_accel_ * t_accel_ + v_peak_ * (t - t_accel_);
  } else {
    const double td = t_total_ - t;
    s = distance_ - 0.5 * a_max_ * td * td;
  }
  return start_ + direction_ * s;
}

double TrapezoidalProfile::velocity(double t) const {
  if (t <= 0.0 || t >= t_total_) return 0.0;
  if (t < t_accel_) return direction_ * a_max_ * t;
  if (t < t_accel_ + t_cruise_) return direction_ * v_peak_;
  return direction_ * a_max_ * (t_total_ - t);
}

double TrapezoidalProfile::acceleration(double t) const {
  if (t <= 0.0 || t >= t_total_) return 0.0;
  if (t < t_accel_) return direction_ * a_max_;
  if (t < t_accel_ + t_cruise_) return 0.0;
  return -direction_ * a_max_;
}

}  // namespace roboticscore::trajectory
