#pragma once

/// @file extended_kalman_filter.hpp
/// @brief Extended Kalman filter over callable nonlinear models.
///
/// Prediction:  x = f(x, u, dt),       P = F P F^T + Q,   F = df/dx
/// Update:      y = residual(z, h(x)), S = H P H^T + R,   K = P H^T S^{-1}
///              x = normalize(x + K y), P = Joseph form
///
/// The EKF linearises around the current estimate, so it is consistent only while the
/// estimation error stays inside the region where f and h are close to linear. See
/// docs/algorithms/estimation/ekf.md for the assumptions and failure modes.

#include <functional>
#include <utility>

#include "roboticscore/core/types.hpp"
#include "roboticscore/estimation/models.hpp"

namespace roboticscore::estimation {

/// Extended Kalman filter.
class ExtendedKalmanFilter {
 public:
  /// @throws std::invalid_argument for an invalid initial covariance.
  ExtendedKalmanFilter(VecX x0, const MatX& p0, StateSpace space = {});

  /// Propagates the estimate through @p model.
  /// @throws std::invalid_argument if the model has no transition function or returns
  ///         results of the wrong dimension.
  void predict(const ProcessModel& model, const VecX& u, double dt);

  /// Fuses measurement @p z. Rejected (state unchanged) if NIS exceeds @p gate.
  /// @throws std::invalid_argument on dimension mismatch.
  /// @throws NumericalError if the innovation covariance is not positive definite.
  UpdateResult update(const MeasurementModel& model, const VecX& z, double gate = kNoGate);

  [[nodiscard]] const VecX& state() const noexcept { return x_; }
  [[nodiscard]] const MatX& covariance() const noexcept { return p_; }

  [[nodiscard]] const StateSpace& state_space() const noexcept { return space_; }

  void reset(VecX x, const MatX& p);

 private:
  VecX x_;
  MatX p_;
  StateSpace space_;
};

}  // namespace roboticscore::estimation
