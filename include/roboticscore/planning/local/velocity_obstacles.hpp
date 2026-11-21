#pragma once

/// @file velocity_obstacles.hpp
/// @brief Sampling-based velocity obstacles for disc-shaped agents.
///
/// Velocity Obstacle (Fiorini & Shiller, IJRR 1998): the set of velocities of agent A that
/// lead to a collision with obstacle B (moving at v_B) within the time horizon tau,
///   VO_tau = { v | exists t in [0, tau]: |p_B - p_A - (v - v_B) t| < r_A + r_B }.
///
/// Reciprocal Velocity Obstacle (van den Berg, Lin & Manocha, ICRA 2008): when B is another
/// agent applying the same policy, A only takes half of the avoidance responsibility,
///   RVO = { v | 2 v - v_A in VO }, which removes the reciprocal oscillations of plain VO.
///
/// Velocity selection samples speeds and headings, keeps candidates outside every (R)VO and
/// returns the one closest to the preferred velocity; if none is collision free it minimises
/// the RVO penalty  w / time_to_collision + |v - v_pref|  (van den Berg et al., Sec. V).
///
/// Symmetric crowds (e.g. agents swapping across a circle) admit a degenerate equilibrium in
/// which all agents drift together, because equal velocities are always outside every RVO.
/// When the preferred velocity is in conflict, it is rotated by a small consistent angle
/// (side_bias, a "keep right" convention) before selection, which breaks the symmetry;
/// without it such scenarios can deadlock. Unobstructed agents keep their exact preference.
///
/// This is not ORCA: ORCA replaces the cones by half-planes and solves a linear program.

#include <limits>
#include <span>

#include "roboticscore/core/types.hpp"

namespace roboticscore::planning {

/// Another agent or moving obstacle as seen by the planning agent.
struct VoNeighbor {
  Vec2 position = Vec2::Zero();
  Vec2 velocity = Vec2::Zero();
  double radius = 0.3;
  bool reciprocal = false;  ///< True if the neighbour also avoids (use RVO).
};

/// Parameters of the velocity selection.
struct VelocityObstacleConfig {
  double max_speed = 1.0;
  double time_horizon = 4.0;  ///< tau [s]
  int speed_samples = 8;
  int heading_samples = 48;
  double safety_margin = 0.05;  ///< Added to the combined radius [m].
  double penalty_weight = 1.0;  ///< w in the collision penalty.
  double side_bias = 0.14;      ///< Rotation of the preferred velocity [rad] (~8 deg).
};

/// Selected velocity and diagnostics.
struct VelocitySelection {
  Vec2 velocity = Vec2::Zero();
  bool collision_free = false;  ///< True if outside every (R)VO within the horizon.
  double time_to_collision = std::numeric_limits<double>::infinity();
};

/// Earliest t >= 0 with |rel_pos + rel_vel t| <= radius, or infinity. Returns 0 if the discs
/// already overlap. rel_pos/rel_vel are those of the neighbour relative to the agent:
/// rel_pos = p_B - p_A, rel_vel = v_A - v_B (closing velocity).
[[nodiscard]] double time_to_collision(const Vec2& rel_pos, const Vec2& rel_vel, double radius);

/// Chooses a velocity for an agent at @p position with current velocity @p velocity.
/// @throws std::invalid_argument for invalid configuration values.
[[nodiscard]] VelocitySelection select_velocity(const Vec2& position, const Vec2& velocity,
                                                double radius, const Vec2& preferred_velocity,
                                                std::span<const VoNeighbor> neighbors,
                                                const VelocityObstacleConfig& config);

}  // namespace roboticscore::planning
