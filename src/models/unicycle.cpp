#include "roboticscore/models/unicycle.hpp"

#include <algorithm>
#include <cmath>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"

namespace roboticscore::models {
namespace {

// d/da sinc(a) = (a cos a - sin a) / a^2, with series -a/3 + a^3/30 near zero.
double sinc_derivative(double a) {
  if (std::abs(a) < 1e-3) {
    const double a2 = a * a;
    return -a / 3.0 + a * a2 / 30.0;
  }
  return (a * std::cos(a) - std::sin(a)) / (a * a);
}

}  // namespace

double sinc(double a) {
  if (std::abs(a) < 1e-4) {
    const double a2 = a * a;
    return 1.0 - a2 / 6.0 + a2 * a2 / 120.0;
  }
  return std::sin(a) / a;
}

Vec3 Unicycle::step(const Vec3& state, const UnicycleInput& u, double dt) {
  detail::require(dt >= 0.0, "Unicycle::step: dt must be non-negative");
  const double half = 0.5 * u.omega * dt;
  const double chord = u.v * dt * sinc(half);
  const double heading = state(2) + half;
  return {state(0) + chord * std::cos(heading), state(1) + chord * std::sin(heading),
          math::wrap_to_pi(state(2) + u.omega * dt)};
}

geometry::Pose2 Unicycle::step(const geometry::Pose2& pose, const UnicycleInput& u, double dt) {
  const Vec3 next = step(Vec3(pose.x(), pose.y(), pose.theta()), u, dt);
  return {next(0), next(1), next(2)};
}

UnicycleJacobians Unicycle::jacobians(const Vec3& state, const UnicycleInput& u, double dt) {
  detail::require(dt >= 0.0, "Unicycle::jacobians: dt must be non-negative");
  const double a = 0.5 * u.omega * dt;
  const double s = sinc(a);
  const double ds = sinc_derivative(a);
  const double phi = state(2) + a;
  const double c = std::cos(phi);
  const double sn = std::sin(phi);

  UnicycleJacobians j;
  j.wrt_state(0, 2) = -u.v * dt * s * sn;
  j.wrt_state(1, 2) = u.v * dt * s * c;

  j.wrt_input(0, 0) = dt * s * c;
  j.wrt_input(1, 0) = dt * s * sn;
  const double k = 0.5 * u.v * dt * dt;
  j.wrt_input(0, 1) = k * (ds * c - s * sn);
  j.wrt_input(1, 1) = k * (ds * sn + s * c);
  j.wrt_input(2, 1) = dt;
  return j;
}

DifferentialDrive::DifferentialDrive(const DifferentialDriveParams& params) : params_(params) {
  detail::require(params.wheel_radius > 0.0 && params.track_width > 0.0,
                  "DifferentialDrive: wheel radius and track width must be positive");
  detail::require(params.max_wheel_speed > 0.0,
                  "DifferentialDrive: max wheel speed must be positive");
}

UnicycleInput DifferentialDrive::forward(const WheelSpeeds& w) const {
  const double r = params_.wheel_radius;
  return {0.5 * r * (w.right + w.left), r * (w.right - w.left) / params_.track_width};
}

WheelSpeeds DifferentialDrive::inverse(const UnicycleInput& u) const {
  const double half_track = 0.5 * params_.track_width;
  return {(u.v - u.omega * half_track) / params_.wheel_radius,
          (u.v + u.omega * half_track) / params_.wheel_radius};
}

UnicycleInput DifferentialDrive::saturate(const UnicycleInput& u) const {
  const WheelSpeeds w = inverse(u);
  const double peak = std::max(std::abs(w.left), std::abs(w.right));
  if (peak <= params_.max_wheel_speed) {
    return u;
  }
  const double scale = params_.max_wheel_speed / peak;
  return {u.v * scale, u.omega * scale};
}

geometry::Pose2 DifferentialDrive::step(const geometry::Pose2& pose, const WheelSpeeds& w,
                                        double dt) const {
  return Unicycle::step(pose, forward(w), dt);
}

}  // namespace roboticscore::models
