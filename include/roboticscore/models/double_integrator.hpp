#pragma once

/// @file double_integrator.hpp
/// @brief n-dimensional double integrator p'' = a, the canonical linear test system for
///        Kalman filtering and LQR.
///
/// State (p_1..p_n, v_1..v_n), input (a_1..a_n). The zero-order-hold discretisation is exact:
///   A = [[I, dt I], [0, I]],   B = [[dt^2/2 I], [dt I]].

#include "roboticscore/core/types.hpp"

namespace roboticscore::models {

/// Double integrator in @p dimension spatial dimensions.
class DoubleIntegrator {
 public:
  /// @throws std::invalid_argument if dimension < 1 or dt <= 0.
  DoubleIntegrator(int dimension, double dt);

  [[nodiscard]] int dimension() const noexcept { return dimension_; }
  [[nodiscard]] double dt() const noexcept { return dt_; }
  [[nodiscard]] const MatX& a() const noexcept { return a_; }
  [[nodiscard]] const MatX& b() const noexcept { return b_; }

  /// Exact discrete step under constant acceleration.
  [[nodiscard]] VecX step(const VecX& x, const VecX& u) const;

  /// Discrete process-noise covariance for white acceleration noise with spectral density
  /// @p q (continuous white-noise acceleration model):
  ///   Q = q [[dt^3/3 I, dt^2/2 I], [dt^2/2 I, dt I]].
  [[nodiscard]] MatX process_noise(double q) const;

  /// Measurement matrix that observes positions only.
  [[nodiscard]] MatX position_measurement() const;

 private:
  int dimension_;
  double dt_;
  MatX a_;
  MatX b_;
};

}  // namespace roboticscore::models
