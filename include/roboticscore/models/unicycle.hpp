#pragma once

/// @file unicycle.hpp
/// @brief Unicycle and differential-drive kinematics.
///
/// The unicycle state is a planar pose (x, y, theta) and the input is (v, omega). For inputs
/// held constant over dt the motion is an exact circular arc, written in the well-conditioned
/// chord form
///
///   x' = x + v dt sinc(omega dt / 2) cos(theta + omega dt / 2)
///   y' = y + v dt sinc(omega dt / 2) sin(theta + omega dt / 2)
///   theta' = theta + omega dt
///
/// which reduces smoothly to straight-line motion as omega -> 0 (no special case).

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/se2.hpp"

namespace roboticscore::models {

/// Velocity command for planar mobile robots.
struct UnicycleInput {
  double v = 0.0;      ///< Forward speed [m/s].
  double omega = 0.0;  ///< Yaw rate [rad/s].
};

/// Jacobians of the discrete unicycle step with respect to state (x, y, theta) and input
/// (v, omega).
struct UnicycleJacobians {
  Mat3 wrt_state = Mat3::Identity();
  Eigen::Matrix<double, 3, 2> wrt_input = Eigen::Matrix<double, 3, 2>::Zero();
};

/// Kinematic unicycle with exact constant-input integration.
class Unicycle {
 public:
  /// Pose after applying @p u for @p dt seconds (exact arc).
  /// @throws std::invalid_argument if dt is negative.
  [[nodiscard]] static geometry::Pose2 step(const geometry::Pose2& pose, const UnicycleInput& u,
                                            double dt);

  /// Same as step() on a vector state (x, y, theta); theta is wrapped to [-pi, pi).
  [[nodiscard]] static Vec3 step(const Vec3& state, const UnicycleInput& u, double dt);

  /// Analytic Jacobians of step() at (state, u), valid for every omega including 0.
  [[nodiscard]] static UnicycleJacobians jacobians(const Vec3& state, const UnicycleInput& u,
                                                   double dt);
};

/// sin(a) / a with a Taylor series near zero.
[[nodiscard]] double sinc(double a);

/// Differential-drive robot parameters.
struct DifferentialDriveParams {
  double wheel_radius = 0.05;     ///< [m]
  double track_width = 0.3;       ///< Distance between wheel contact points [m].
  double max_wheel_speed = 20.0;  ///< Wheel angular speed limit [rad/s].
};

/// Wheel angular velocities of a differential-drive robot [rad/s].
struct WheelSpeeds {
  double left = 0.0;
  double right = 0.0;
};

/// Differential-drive kinematics on top of the unicycle model.
class DifferentialDrive {
 public:
  /// @throws std::invalid_argument for non-positive geometry or speed limit.
  explicit DifferentialDrive(const DifferentialDriveParams& params);

  [[nodiscard]] const DifferentialDriveParams& params() const noexcept { return params_; }

  /// Body velocity produced by the given wheel speeds.
  [[nodiscard]] UnicycleInput forward(const WheelSpeeds& w) const;

  /// Wheel speeds that realise the body velocity @p u (no saturation).
  [[nodiscard]] WheelSpeeds inverse(const UnicycleInput& u) const;

  /// Scales @p u so no wheel exceeds max_wheel_speed while preserving the path curvature
  /// omega / v (both wheels are scaled by the same factor).
  [[nodiscard]] UnicycleInput saturate(const UnicycleInput& u) const;

  /// Pose after driving with wheel speeds @p w for @p dt seconds.
  [[nodiscard]] geometry::Pose2 step(const geometry::Pose2& pose, const WheelSpeeds& w,
                                     double dt) const;

 private:
  DifferentialDriveParams params_;
};

}  // namespace roboticscore::models
