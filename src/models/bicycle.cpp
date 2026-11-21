#include "roboticscore/models/bicycle.hpp"

#include <algorithm>
#include <cmath>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/integration.hpp"
#include "roboticscore/models/unicycle.hpp"

namespace roboticscore::models {

KinematicBicycle::KinematicBicycle(const BicycleParams& params) : params_(params) {
  detail::require(params.wheelbase > 0.0, "KinematicBicycle: wheelbase must be positive");
  detail::require(params.max_steer > 0.0 && params.max_steer < math::kHalfPi,
                  "KinematicBicycle: max_steer must lie in (0, pi/2)");
  detail::require(params.max_speed > 0.0 && params.max_accel > 0.0,
                  "KinematicBicycle: speed and acceleration limits must be positive");
}

Vec4 KinematicBicycle::derivative(const BicycleState& x, const BicycleInput& u) const {
  const double yaw = x(2);
  const double v = x(3);
  return {v * std::cos(yaw), v * std::sin(yaw), v * std::tan(u(1)) / params_.wheelbase, u(0)};
}

BicycleState KinematicBicycle::step(const BicycleState& x, const BicycleInput& u, double dt) const {
  detail::require(dt >= 0.0, "KinematicBicycle::step: dt must be non-negative");
  const BicycleInput clamped(std::clamp(u(0), -params_.max_accel, params_.max_accel),
                             std::clamp(u(1), -params_.max_steer, params_.max_steer));
  const auto f = [&](double /*t*/, const Vec4& s) -> Vec4 {
    return derivative(s, clamped);
  };
  BicycleState next = math::rk4_step(f, x, 0.0, dt);
  next(2) = math::wrap_to_pi(next(2));
  next(3) = std::clamp(next(3), -params_.max_speed, params_.max_speed);
  return next;
}

geometry::Pose2 KinematicBicycle::step_pose(const geometry::Pose2& pose, double v, double delta,
                                            double dt) const {
  return Unicycle::step(pose, {v, v * curvature(delta)}, dt);
}

double KinematicBicycle::curvature(double delta) const {
  return std::tan(delta) / params_.wheelbase;
}

double KinematicBicycle::steering_for_curvature(double kappa) const {
  return std::atan(kappa * params_.wheelbase);
}

double KinematicBicycle::min_turning_radius() const {
  return params_.wheelbase / std::tan(params_.max_steer);
}

BicycleLinearization KinematicBicycle::linearize(const BicycleState& x, const BicycleInput& u,
                                                 double dt) const {
  const double yaw = x(2);
  const double v = x(3);
  const double delta = u(1);
  const double c_delta = std::cos(delta);

  Mat4 ac = Mat4::Zero();
  ac(0, 2) = -v * std::sin(yaw);
  ac(0, 3) = std::cos(yaw);
  ac(1, 2) = v * std::cos(yaw);
  ac(1, 3) = std::sin(yaw);
  ac(2, 3) = std::tan(delta) / params_.wheelbase;

  BicycleInputMatrix bc = BicycleInputMatrix::Zero();
  bc(2, 1) = v / (params_.wheelbase * c_delta * c_delta);
  bc(3, 0) = 1.0;

  BicycleLinearization lin;
  lin.a = Mat4::Identity() + dt * ac;
  lin.b = dt * bc;
  lin.c = dt * (derivative(x, u) - ac * x - bc * u);
  return lin;
}

Vec2 AckermannGeometry::wheel_angles(double delta) const {
  detail::require(wheelbase > 0.0 && track_width > 0.0,
                  "AckermannGeometry: wheelbase and track width must be positive");
  if (std::abs(delta) < 1e-12) {
    return Vec2::Zero();
  }
  // Signed turning radius of the rear-axle centre; positive for left turns.
  const double radius = wheelbase / std::tan(delta);
  const double left = std::atan(wheelbase / (radius - 0.5 * track_width));
  const double right = std::atan(wheelbase / (radius + 0.5 * track_width));
  return {left, right};
}

Vec2 VehicleFootprint::center(const geometry::Pose2& pose) const {
  return pose * Vec2(0.5 * length - rear_overhang, 0.0);
}

geometry::Polygon VehicleFootprint::polygon(const geometry::Pose2& pose) const {
  return geometry::Polygon::rectangle(center(pose), length, width, pose.theta());
}

double VehicleFootprint::bounding_radius() const {
  return 0.5 * std::hypot(length, width);
}

}  // namespace roboticscore::models
