#pragma once

/// @file dwa.hpp
/// @brief Dynamic Window Approach (Fox, Burgard & Thrun, "The Dynamic Window Approach to
///        Collision Avoidance", IEEE RAM 1997).
///
/// Each control cycle searches the velocity space (v, omega) restricted to
///   - the robot's velocity limits,
///   - the dynamic window reachable within one control period under acceleration limits,
///   - admissible velocities, from which the robot can still brake before the nearest
///     obstacle on its curvature: v <= sqrt(2 * dist(v, omega) * a_max).
/// Following the paper, dist(v, omega) is the free distance along the circular arc of
/// curvature omega / v (capped at clearance_cap), so it depends on the direction of motion and
/// not on the speed; standing still is therefore not rewarded with extra clearance. Each
/// command is also rolled out for predict_time seconds and rejected if the rollout touches an
/// obstacle. Admissible commands are scored with
///   G = w_heading * (1 - |heading error| / pi) + w_clearance * dist / clearance_cap
///     + w_velocity * v / max_speed.
///
/// Two deliberate deviations from the paper, both found necessary in testing:
///   - Terms use fixed physical scales instead of per-cycle min-max normalisation, which
///     magnified millimetre differences into full-range score changes and made the robot
///     alternate between nearly identical commands.
///   - The arc curvature uses max(|v|, curvature_speed_floor) as speed, so near-stationary
///     commands are not evaluated as arbitrarily tight (and therefore obstacle-free) circles.
///
/// Moving obstacles are predicted with constant velocity along each rollout (time-aware
/// clearance), an extension of the static formulation of the paper.
///
/// DWA is purely local: it can be trapped by concave obstacles (no global information), so it
/// is normally driven by waypoints from a global planner.

#include <span>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/collision.hpp"
#include "roboticscore/geometry/environment.hpp"
#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/models/unicycle.hpp"

namespace roboticscore::planning {

/// DWA parameters.
struct DwaConfig {
  double max_speed = 1.0;       ///< [m/s]
  double min_speed = 0.0;       ///< Negative values allow reversing [m/s].
  double max_yaw_rate = 1.5;    ///< [rad/s]
  double max_accel = 1.0;       ///< [m/s^2]
  double max_yaw_accel = 3.0;   ///< [rad/s^2]
  double control_period = 0.1;  ///< Dynamic-window horizon [s].
  double predict_time = 2.0;    ///< Rollout horizon [s].
  double rollout_dt = 0.1;      ///< Rollout integration step [s].
  int speed_samples = 11;
  int yaw_rate_samples = 21;
  double heading_weight = 0.8;
  double clearance_weight = 0.4;
  double velocity_weight = 0.2;
  double robot_radius = 0.3;            ///< [m]
  double clearance_cap = 2.0;           ///< Clearances beyond this are scored equally [m].
  double curvature_speed_floor = 0.25;  ///< Minimum speed used to evaluate arcs [m/s].
  /// If no command is admissible, return the rollout with the largest worst-case clearance
  /// (result.valid stays false) instead of a zero command.
  bool escape_when_blocked = false;
};

/// A circular obstacle moving with constant velocity. Rollouts are checked against its
/// predicted position at each rollout time, so the planner can move out of the way of
/// obstacles that approach the robot.
struct MovingObstacle {
  geometry::Circle shape;
  Vec2 velocity = Vec2::Zero();
};

/// A scored rollout.
struct DwaCandidate {
  models::UnicycleInput command;
  std::vector<geometry::Pose2> rollout;
  double clearance = 0.0;      ///< Minimum clearance along the rollout (minus robot radius).
  double free_distance = 0.0;  ///< Free distance along the command's curvature (capped).
  double heading = 0.0;        ///< Raw heading term in [0, pi].
  bool admissible = false;
  double score = 0.0;
};

/// Result of one DWA cycle.
struct DwaResult {
  bool valid = false;  ///< False if no admissible command exists (stop or escape, see config).
  models::UnicycleInput command;  ///< Zero velocity when invalid.
  std::vector<DwaCandidate> candidates;
  std::size_t best = 0;
};

/// Runs one DWA control cycle towards @p goal.
/// @throws std::invalid_argument for invalid configuration values.
[[nodiscard]] DwaResult dwa_plan(const geometry::Pose2& pose, const models::UnicycleInput& velocity,
                                 const Vec2& goal, const geometry::Environment2D& env,
                                 std::span<const MovingObstacle> dynamic_obstacles,
                                 const DwaConfig& config);

}  // namespace roboticscore::planning
