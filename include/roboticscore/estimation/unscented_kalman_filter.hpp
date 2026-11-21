#pragma once

/// @file unscented_kalman_filter.hpp
/// @brief Unscented Kalman filter with scaled (Van der Merwe) sigma points.
///
/// Sigma points (Wan & van der Merwe, 2000), lambda = alpha^2 (n + kappa) - n:
///   X_0 = x,  X_i = x + [sqrt((n + lambda) P)]_i,  X_{n+i} = x - [sqrt((n + lambda) P)]_i
///   Wm_0 = lambda / (n + lambda),  Wc_0 = Wm_0 + 1 - alpha^2 + beta,
///   Wm_i = Wc_i = 1 / (2 (n + lambda))
/// The matrix square root is a (jittered) lower Cholesky factor.
///
/// Parameter choice: the default alpha = 1, beta = 2, kappa = 0 keeps every covariance weight
/// non-negative. Very small alpha (1e-3, common in the literature) makes Wm_0 large and
/// negative, which can destroy positive definiteness for n > 3 in double precision.
///
/// Complexity per step: O(n^3) for the factorisation plus 2n+1 model evaluations.

#include "roboticscore/core/types.hpp"
#include "roboticscore/estimation/models.hpp"

namespace roboticscore::estimation {

/// Scaling parameters of the sigma-point set.
struct SigmaPointParams {
  double alpha = 1.0;
  double beta = 2.0;
  double kappa = 0.0;
};

/// 2n+1 sigma points (columns) with mean and covariance weights.
struct SigmaPoints {
  MatX points;
  VecX mean_weights;
  VecX covariance_weights;
};

/// Generates scaled sigma points for N(mean, covariance).
/// @throws std::invalid_argument for dimension mismatch or n + lambda <= 0.
/// @throws NumericalError if the covariance cannot be factorised.
[[nodiscard]] SigmaPoints merwe_sigma_points(const VecX& mean, const MatX& covariance,
                                             const SigmaPointParams& params,
                                             const StateSpace& space = {});

/// Unscented Kalman filter.
class UnscentedKalmanFilter {
 public:
  /// @throws std::invalid_argument for an invalid initial covariance.
  UnscentedKalmanFilter(VecX x0, const MatX& p0, SigmaPointParams params = {},
                        StateSpace space = {});

  /// Unscented transform of the state through the process model.
  void predict(const ProcessModel& model, const VecX& u, double dt);

  /// Unscented measurement update; rejected (state unchanged) if NIS exceeds @p gate.
  /// @throws NumericalError if the innovation covariance is not positive definite.
  UpdateResult update(const MeasurementModel& model, const VecX& z, double gate = kNoGate);

  [[nodiscard]] const VecX& state() const noexcept { return x_; }
  [[nodiscard]] const MatX& covariance() const noexcept { return p_; }
  [[nodiscard]] const SigmaPointParams& params() const noexcept { return params_; }

  void reset(VecX x, const MatX& p);

 private:
  VecX x_;
  MatX p_;
  SigmaPointParams params_;
  StateSpace space_;
};

}  // namespace roboticscore::estimation
