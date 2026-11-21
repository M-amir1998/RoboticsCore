#pragma once

/// @file bicycle.hpp
/// @brief Kinematic bicycle model for car-like vehicles, Ackermann steering geometry and the
///        vehicle footprint used for collision checking.
///
/// Reference point: centre of the rear axle. State (x, y, yaw, v), input (a, delta):
///
///   x' = v cos(yaw),  y' = v sin(yaw),  yaw' = v tan(delta) / L,  v' = a.
///
/// The model assumes no tyre slip, which is accurate at the low lateral accelerations of
/// parking and urban manoeuvres but optimistic at high speed.

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/collision.hpp"
#include "roboticscore/geometry/se2.hpp"

namespace roboticscore::models {

/// Physical limits and geometry of a car-like vehicle.
struct BicycleParams {
  double wheelbase = 2.7;   ///< Rear axle to front axle [m].
  double max_steer = 0.6;   ///< Steering angle limit |delta| [rad].
  double max_speed = 15.0;  ///< |v| limit [m/s].
  double max_accel = 3.0;   ///< |a| limit [m/s^2].
};

using BicycleState = Vec4;  ///< (x, y, yaw, v)
using BicycleInput = Vec2;  ///< (a, delta)
using BicycleInputMatrix = Eigen::Matrix<double, 4, 2>;

/// Affine discrete-time model x_{k+1} ~= A x_k + B u_k + c around a linearisation point.
struct BicycleLinearization {
  Mat4 a = Mat4::Identity();
  BicycleInputMatrix b = BicycleInputMatrix::Zero();
  Vec4 c = Vec4::Zero();
};

/// Kinematic bicycle model.
class KinematicBicycle {
 public:
  /// @throws std::invalid_argument for non-positive wheelbase or limits.
  explicit KinematicBicycle(const BicycleParams& params);

  [[nodiscard]] const BicycleParams& params() const noexcept { return params_; }

  /// Continuous-time state derivative (inputs are used as given, not clamped).
  [[nodiscard]] Vec4 derivative(const BicycleState& x, const BicycleInput& u) const;

  /// RK4 step with acceleration and steering clamped to the limits, speed clamped to
  /// max_speed and yaw wrapped to [-pi, pi).
  [[nodiscard]] BicycleState step(const BicycleState& x, const BicycleInput& u, double dt) const;

  /// Exact constant-speed, constant-steering arc of length v*dt from @p pose.
  [[nodiscard]] geometry::Pose2 step_pose(const geometry::Pose2& pose, double v, double delta,
                                          double dt) const;

  /// Path curvature produced by steering angle @p delta.
  [[nodiscard]] double curvature(double delta) const;

  /// Steering angle producing curvature @p kappa (not clamped).
  [[nodiscard]] double steering_for_curvature(double kappa) const;

  /// Minimum turning radius at max_steer.
  [[nodiscard]] double min_turning_radius() const;

  /// First-order (forward Euler) discretisation of the analytic Jacobians at (x, u). Used by
  /// linear MPC; the affine term c makes the model exact at the linearisation point.
  [[nodiscard]] BicycleLinearization linearize(const BicycleState& x, const BicycleInput& u,
                                               double dt) const;

 private:
  BicycleParams params_;
};

/// Ackermann steering geometry: individual front wheel angles for a bicycle steering angle.
struct AckermannGeometry {
  double wheelbase = 2.7;    ///< [m]
  double track_width = 1.6;  ///< Distance between the front wheels [m].

  /// (left, right) wheel angles for a virtual centre-wheel angle @p delta. The inner wheel
  /// turns more than the outer one so both wheel axes meet on the rear axle line.
  [[nodiscard]] Vec2 wheel_angles(double delta) const;
};

/// Rectangular vehicle body for collision checking.
struct VehicleFootprint {
  double length = 4.5;         ///< Bumper to bumper [m].
  double width = 1.9;          ///< [m]
  double rear_overhang = 0.9;  ///< Rear axle to rear bumper [m].

  /// Body polygon for a rear-axle pose.
  [[nodiscard]] geometry::Polygon polygon(const geometry::Pose2& pose) const;

  /// Radius of the smallest disc centred at the body centre that covers the body.
  [[nodiscard]] double bounding_radius() const;

  /// Body centre for a rear-axle pose.
  [[nodiscard]] Vec2 center(const geometry::Pose2& pose) const;
};

}  // namespace roboticscore::models
