#pragma once

/// @file robot_models.hpp
/// @brief Process and measurement models for planar mobile-robot localisation, expressed in
///        the generic estimation::ProcessModel / MeasurementModel form.
///
/// State: (x, y, theta). Control: (v, omega). These factories keep robot-specific modelling
/// out of the generic filters, which never know that a state contains a heading.

#include "roboticscore/core/types.hpp"
#include "roboticscore/estimation/models.hpp"

namespace roboticscore::localization {

/// Control-dependent velocity noise of the odometry motion model (Thrun, Burgard & Fox,
/// "Probabilistic Robotics", 2005, sec. 5.3):
///   var(v)     = alpha1 v^2 + alpha2 omega^2 + min_var_v
///   var(omega) = alpha3 v^2 + alpha4 omega^2 + min_var_omega
/// The floors keep the covariance positive definite when the robot is stationary.
struct VelocityNoise {
  double alpha1 = 0.01;
  double alpha2 = 0.001;
  double alpha3 = 0.001;
  double alpha4 = 0.01;
  double min_var_v = 1e-6;
  double min_var_omega = 1e-6;

  /// Diagonal control-space covariance M for command (v, omega).
  [[nodiscard]] Mat2 control_covariance(double v, double omega) const;
};

/// Unicycle process model; process noise is V M V^T with V = df/du (control noise mapped
/// into state space). Residual/normalize/mean handle the heading.
[[nodiscard]] estimation::ProcessModel unicycle_process_model(const VelocityNoise& noise);

/// Range and bearing to a known point landmark. The Jacobian is undefined when the robot
/// sits on the landmark; it throws NumericalError for ranges below 1e-9 m.
[[nodiscard]] estimation::MeasurementModel range_bearing_model(const Vec2& landmark,
                                                               double range_std,
                                                               double bearing_std);

/// Noisy absolute position (GPS-like) measurement of (x, y).
[[nodiscard]] estimation::MeasurementModel position_model(double position_std);

/// Predicted (range, bearing) from pose state (x, y, theta) to @p landmark.
[[nodiscard]] Vec2 predict_range_bearing(const Vec3& pose, const Vec2& landmark);

/// Pose difference a - b with the heading wrapped to [-pi, pi).
[[nodiscard]] VecX pose_residual(const VecX& a, const VecX& b);

/// Wraps the heading (index 2) of a pose state.
[[nodiscard]] VecX normalize_pose(const VecX& x);

/// Weighted mean of pose sigma points (columns) using a circular mean for the heading.
[[nodiscard]] VecX pose_mean(const MatX& points, const VecX& weights);

/// Log-likelihood of a range-bearing observation for a single pose hypothesis, with
/// independent Gaussian range and bearing errors. Used by particle filters and FastSLAM.
[[nodiscard]] double range_bearing_log_likelihood(const Vec3& pose, const Vec2& landmark,
                                                  const Vec2& z, double range_std,
                                                  double bearing_std);

}  // namespace roboticscore::localization
