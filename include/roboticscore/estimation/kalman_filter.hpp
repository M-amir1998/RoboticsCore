#pragma once

/// @file kalman_filter.hpp
/// @brief Discrete linear Kalman filter.
///
///   x_k = A x_{k-1} + B u_k + w,   w ~ N(0, Q)
///   z_k = H x_k + v,               v ~ N(0, R)
///
/// The covariance update uses the Joseph form P = (I - K H) P (I - K H)^T + K R K^T, which
/// stays symmetric positive semidefinite under round-off and for suboptimal gains, followed
/// by explicit symmetrisation.
///
/// Complexity per step: O(n^3 + m^3) for state dimension n and measurement dimension m.

#include "roboticscore/core/types.hpp"
#include "roboticscore/estimation/models.hpp"

namespace roboticscore::estimation {

/// Linear Gaussian state estimator.
class KalmanFilter {
 public:
  /// @throws std::invalid_argument if P0 is not square, symmetric and positive semidefinite
  ///         with the dimension of x0.
  KalmanFilter(VecX x0, const MatX& p0);

  /// Time update without control input.
  /// @throws std::invalid_argument on dimension mismatch.
  void predict(const MatX& a, const MatX& q);

  /// Time update with control input u.
  /// @throws std::invalid_argument on dimension mismatch.
  void predict(const MatX& a, const MatX& b, const VecX& u, const MatX& q);

  /// Measurement update. If NIS exceeds @p gate the measurement is rejected and the state is
  /// left unchanged.
  /// @throws std::invalid_argument on dimension mismatch.
  /// @throws NumericalError if the innovation covariance is not positive definite.
  UpdateResult update(const VecX& z, const MatX& h, const MatX& r, double gate = kNoGate);

  [[nodiscard]] const VecX& state() const noexcept { return x_; }
  [[nodiscard]] const MatX& covariance() const noexcept { return p_; }
  [[nodiscard]] Index dimension() const noexcept { return x_.size(); }

  /// Replaces the estimate. Same validation as the constructor.
  void reset(VecX x, const MatX& p);

 private:
  VecX x_;
  MatX p_;
};

/// Kalman update shared by the linear and extended filters: applies innovation @p y with
/// Jacobian @p h and noise @p r to (x, p). Exposed for reuse in SLAM back ends.
/// @throws NumericalError if the innovation covariance is not positive definite.
UpdateResult kalman_update(VecX& x, MatX& p, const VecX& y, const MatX& h, const MatX& r,
                           double gate = kNoGate);

/// Validates that @p p is a square symmetric PSD covariance of dimension @p n.
/// @throws std::invalid_argument otherwise.
void validate_covariance(const MatX& p, Index n, const char* who);

}  // namespace roboticscore::estimation
