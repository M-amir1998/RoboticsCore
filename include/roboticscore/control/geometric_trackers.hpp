#pragma once

/// @file geometric_trackers.hpp
/// @brief Geometric path trackers: Pure Pursuit and Stanley. Both are forward-driving
///        controllers; use LQR or MPC for paths with reverse segments.
///
/// Pure Pursuit (Coulter, CMU-RI-TR-92-01, 1992): the rear axle chases a target point a
/// lookahead distance L_d ahead along the path, L_d = clamp(k_v |v| + L_0, L_min, L_max).
/// With alpha the bearing of the target in the vehicle frame, the arc through the target gives
///   delta = atan(2 L sin(alpha) / L_d).
/// Short lookaheads track tightly but oscillate; long lookaheads cut corners.
///
/// Stanley (Hoffmann, Tomlin, Montemerlo & Thrun, ACC 2007): with e the cross-track error of
/// the front axle (positive left) and theta_e = path heading - yaw,
///   delta = theta_e + atan(-k e / (k_soft + |v|)).
/// The softening constant k_soft keeps the gain finite at low speed.

#include <string_view>

#include "roboticscore/control/path_tracking.hpp"

namespace roboticscore::control {

/// Pure Pursuit parameters.
struct PurePursuitConfig {
  double lookahead_gain = 0.5;    ///< k_v [s]
  double lookahead_min = 2.0;     ///< [m]
  double lookahead_max = 15.0;    ///< [m]
  double lookahead_offset = 1.5;  ///< L_0 [m]
  PidConfig speed_pid{1.5, 0.3, 0.0, -3.0, 3.0, 0.0, AntiWindup::kClamping, 1.0};
};

class PurePursuitTracker final : public PathTracker {
 public:
  PurePursuitTracker(const models::BicycleParams& vehicle, const PurePursuitConfig& config = {});
  VehicleCommand compute(const models::BicycleState& state, const trajectory::ReferencePath& path,
                         double dt) override;
  [[nodiscard]] std::string_view name() const override { return "pure_pursuit"; }
  [[nodiscard]] double last_lookahead() const noexcept { return lookahead_; }

 private:
  PurePursuitConfig config_;
  double lookahead_ = 0.0;
};

/// Stanley parameters.
struct StanleyConfig {
  double gain = 1.5;                   ///< k [1/s]
  double softening = 1.0;              ///< k_soft [m/s]
  bool curvature_feedforward = false;  ///< Add atan(L kappa) (not in the original paper).
  PidConfig speed_pid{1.5, 0.3, 0.0, -3.0, 3.0, 0.0, AntiWindup::kClamping, 1.0};
};

class StanleyTracker final : public PathTracker {
 public:
  StanleyTracker(const models::BicycleParams& vehicle, const StanleyConfig& config = {});
  VehicleCommand compute(const models::BicycleState& state, const trajectory::ReferencePath& path,
                         double dt) override;
  [[nodiscard]] std::string_view name() const override { return "stanley"; }

 private:
  StanleyConfig config_;
};

}  // namespace roboticscore::control
