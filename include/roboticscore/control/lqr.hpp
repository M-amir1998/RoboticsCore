#pragma once

/// @file lqr.hpp
/// @brief Discrete-time LQR: Riccati solvers and an LQR path tracker.
///
/// Infinite horizon: minimise sum x^T Q x + u^T R u for x+ = A x + B u. The optimal feedback
/// u = -K x uses the stabilising solution P of the discrete algebraic Riccati equation (DARE)
///   P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q,   K = (R + B^T P B)^{-1} B^T P A.
/// It is computed with the structure-preserving doubling algorithm (SDA; Chu, Fan, Lin & Weng,
/// Linear Algebra Appl. 2004), which converges quadratically:
///   W_k = I + G_k H_k
///   A_{k+1} = A_k W_k^{-1} A_k
///   G_{k+1} = G_k + A_k W_k^{-1} G_k A_k^T
///   H_{k+1} = H_k + A_k^T H_k W_k^{-1} A_k,   A_0 = A, G_0 = B R^{-1} B^T, H_0 = Q,
/// with H_k -> P. Every solution is checked against the DARE residual before it is reported
/// as converged. A stabilising solution exists if (A, B) is stabilisable and (A, Q^{1/2}) is
/// detectable.

#include <string_view>
#include <vector>

#include "roboticscore/control/path_tracking.hpp"
#include "roboticscore/core/types.hpp"

namespace roboticscore::control {

/// DARE solution and diagnostics.
struct DareResult {
  MatX p;
  MatX k;
  int iterations = 0;
  bool converged = false;
  double residual = 0.0;  ///< Relative Frobenius norm of the DARE residual.
};

/// @throws std::invalid_argument on dimension mismatch, non-symmetric/indefinite Q or R not
///         positive definite.
[[nodiscard]] DareResult solve_dare(const MatX& a, const MatX& b, const MatX& q, const MatX& r,
                                    int max_iterations = 100, double tolerance = 1e-12);

/// Finite-horizon LQR gains K_0..K_{N-1} from the backward Riccati recursion with terminal
/// weight Qf (time-invariant system).
/// @throws std::invalid_argument on dimension mismatch or horizon < 1.
[[nodiscard]] std::vector<MatX> finite_horizon_lqr(const MatX& a, const MatX& b, const MatX& q,
                                                   const MatX& r, const MatX& qf, int horizon);

/// LQR path-tracker parameters.
struct LqrTrackerConfig {
  double q_lateral = 1.0;       ///< Weight on lateral error [1/m^2].
  double q_heading = 1.0;       ///< Weight on heading error [1/rad^2].
  double r_steering = 2.0;      ///< Weight on feedback steering [1/rad^2].
  double min_gain_speed = 0.5;  ///< Speed floor for the gain computation [m/s].
  bool curvature_feedforward = true;
  PidConfig speed_pid{1.5, 0.3, 0.0, -3.0, 3.0, 0.0, AntiWindup::kClamping, 1.0};
};

/// LQR steering on the linearised kinematic error dynamics (rear axle, speed v):
///   e_y+   = e_y + v dt e_psi
///   e_psi+ = e_psi + v dt / (L cos^2(delta_ff)) (delta - delta_ff),  delta_ff = atan(L kappa)
/// The gain is recomputed for the current speed every step (gain scheduling), which also
/// handles reverse driving because v enters with its sign.
class LqrTracker final : public PathTracker {
 public:
  LqrTracker(const models::BicycleParams& vehicle, const LqrTrackerConfig& config = {});
  VehicleCommand compute(const models::BicycleState& state, const trajectory::ReferencePath& path,
                         double dt) override;
  [[nodiscard]] std::string_view name() const override { return "lqr"; }
  [[nodiscard]] const MatX& last_gain() const noexcept { return gain_; }

 private:
  LqrTrackerConfig config_;
  MatX gain_;
};

}  // namespace roboticscore::control
