#pragma once

/// @file models.hpp
/// @brief Callable process and measurement model descriptions shared by the EKF and UKF.
///
/// Models are plain structs of std::function so any nonlinear system can be plugged in
/// without inheritance. Optional members fall back to sensible defaults:
///   - jacobian empty    -> central finite differences (EKF only)
///   - residual empty    -> Euclidean difference a - b
///   - normalize empty   -> identity
///   - mean empty        -> weighted arithmetic mean (UKF only)
/// Supply residual/normalize/mean for states or measurements containing angles.

#include <functional>
#include <limits>

#include "roboticscore/core/types.hpp"

namespace roboticscore::estimation {

/// Discrete-time process x_k = f(x_{k-1}, u_k, dt) + w, w ~ N(0, Q(x, u, dt)).
struct ProcessModel {
  std::function<VecX(const VecX& x, const VecX& u, double dt)> f;
  /// df/dx evaluated at (x, u, dt). Optional for the EKF.
  std::function<MatX(const VecX& x, const VecX& u, double dt)> jacobian;
  /// Process noise covariance in state space.
  std::function<MatX(const VecX& x, const VecX& u, double dt)> noise;
  /// Difference a - b in the state tangent space (e.g. wrapping headings).
  std::function<VecX(const VecX& a, const VecX& b)> residual;
  /// Maps a state back to its canonical range (e.g. heading to [-pi, pi)).
  std::function<VecX(const VecX& x)> normalize;
  /// Weighted mean of sigma points stored column-wise (UKF).
  std::function<VecX(const MatX& points, const VecX& weights)> mean;
};

/// Measurement z = h(x) + v, v ~ N(0, R).
struct MeasurementModel {
  std::function<VecX(const VecX& x)> h;
  /// dh/dx evaluated at x. Optional for the EKF.
  std::function<MatX(const VecX& x)> jacobian;
  MatX noise;
  /// Difference a - b in measurement space (e.g. wrapping bearings).
  std::function<VecX(const VecX& a, const VecX& b)> residual;
  /// Weighted mean of measurement sigma points stored column-wise (UKF).
  std::function<VecX(const MatX& points, const VecX& weights)> mean;
};

/// Innovation statistics of a measurement update.
struct InnovationStats {
  VecX innovation;   ///< y = z - h(x) (through the residual function)
  MatX covariance;   ///< S = H P H^T + R
  double nis = 0.0;  ///< Normalised innovation squared y^T S^{-1} y
};

/// Result of a (possibly gated) measurement update.
struct UpdateResult {
  bool accepted = true;  ///< False if the measurement failed the validation gate.
  InnovationStats stats;
};

/// Disabled validation gate.
inline constexpr double kNoGate = std::numeric_limits<double>::infinity();

}  // namespace roboticscore::estimation
