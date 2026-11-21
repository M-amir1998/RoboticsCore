#pragma once

/// @file mpc.hpp
/// @brief Linear time-varying model predictive path tracking for the kinematic bicycle.
///
/// At every step the controller
///   1. samples a reference (x, y, yaw, v) and feedforward input (a, delta) along the path for
///      N steps of length dt, starting at the projection of the vehicle;
///   2. linearises the bicycle about that reference, x_{k+1} = A_k x_k + B_k u_k + c_k;
///   3. condenses the prediction X = Phi x_0 + Gamma U + Psi and solves the QP
///        min  sum_k |x_k - r_k|^2_Q + |u_k - u_ref,k|^2_R + |u_k - u_{k-1}|^2_Rd  (+ terminal)
///        s.t. |a| <= a_max, |delta| <= delta_max, |delta_k - delta_{k-1}| <= rate * dt,
///             v_min <= v_k <= v_max
///      with the internal ADMM QP solver, warm-started from the shifted previous solution;
///   4. applies the first input (receding horizon).
/// Reference yaw angles are unwrapped around the current yaw so the cost never sees a 2 pi jump.
/// Negative reference speeds (reverse segments) are handled by the same model.

#include <string_view>

#include "roboticscore/control/path_tracking.hpp"
#include "roboticscore/core/types.hpp"
#include "roboticscore/optimization/qp_solver.hpp"

namespace roboticscore::control {

/// MPC parameters.
struct MpcConfig {
  int horizon = 15;
  double dt = 0.1;                         ///< Prediction step [s].
  Vec4 state_weights{2.0, 2.0, 3.0, 0.5};  ///< (x, y, yaw, v)
  Vec2 input_weights{0.05, 0.2};           ///< (a, delta) deviation from feedforward
  Vec2 input_rate_weights{0.1, 3.0};       ///< (a, delta) change per step
  double terminal_weight_scale = 3.0;
  double max_steer_rate = 0.8;  ///< [rad/s]
  double min_speed = -3.0;      ///< [m/s]
  optimization::QpSettings qp{0.1, 1e-6, 1.6, 1e-4, 1e-4, 400, 1e3};
};

class MpcTracker final : public PathTracker {
 public:
  /// @throws std::invalid_argument for invalid configuration values.
  MpcTracker(const models::BicycleParams& vehicle, const MpcConfig& config = {});
  VehicleCommand compute(const models::BicycleState& state, const trajectory::ReferencePath& path,
                         double dt) override;
  void reset() override;
  [[nodiscard]] std::string_view name() const override { return "mpc"; }
  [[nodiscard]] int last_qp_iterations() const noexcept { return qp_iterations_; }
  [[nodiscard]] bool last_qp_solved() const noexcept { return qp_solved_; }

 private:
  MpcConfig config_;
  models::KinematicBicycle model_;
  VecX previous_u_;  ///< Previous optimal input sequence (warm start).
  VecX previous_y_;
  Vec2 last_input_ = Vec2::Zero();
  int qp_iterations_ = 0;
  bool qp_solved_ = false;
};

}  // namespace roboticscore::control
