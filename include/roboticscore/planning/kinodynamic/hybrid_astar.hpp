#pragma once

/// @file hybrid_astar.hpp
/// @brief Hybrid A* for car-like vehicles (Dolgov, Thrun, Montemerlo & Diebel, "Path
///        Planning for Autonomous Vehicles in Unknown Semi-structured Environments", IJRR 2010).
///
/// Search runs over a discretised (x, y, yaw) grid, but every node keeps its continuous pose,
/// so expanded motions are exact kinematic-bicycle arcs. Successors use a fixed set of
/// steering angles in forward and (optionally) reverse gear. The heuristic is the maximum of
///   - the Reeds-Shepp length to the goal (kinematics, no obstacles), and
///   - a 2D grid Dijkstra distance to the goal (obstacles, no kinematics; computed once).
/// Periodically a Reeds-Shepp "analytic expansion" to the goal is collision checked; if free,
/// it completes the path and lands exactly on the goal pose.
///
/// Cost of a motion of length l (all terms configurable):
///   l * (reverse ? reverse_penalty : 1) + steer_penalty * |delta| * l
///   + steer_change_penalty * |delta - delta_parent| + (gear change ? gear_switch_penalty : 0)
///
/// Hybrid A* is resolution complete only in a weak sense and not optimal: pruning by cell
/// discards states that a finer search would keep, and the grid heuristic can overestimate.

#include <cstddef>
#include <functional>
#include <vector>

#include "roboticscore/geometry/environment.hpp"
#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/models/bicycle.hpp"
#include "roboticscore/planning/kinodynamic/reeds_shepp.hpp"

namespace roboticscore::planning {

/// Hybrid A* parameters.
struct HybridAStarConfig {
  double xy_resolution = 0.5;            ///< Grid cell size for pruning and the 2D heuristic [m].
  double yaw_resolution = 0.0872664626;  ///< Heading bin size [rad] (5 degrees).
  double step_length = 1.2;              ///< Arc length of one motion primitive [m].
  int steering_samples = 5;              ///< Odd number of steering angles in [-max, max].
  bool allow_reverse = true;
  double reverse_penalty = 2.0;         ///< Multiplier on reverse distance (>= 1).
  double gear_switch_penalty = 6.0;     ///< Added per direction change [m-equivalent].
  double steer_penalty = 0.2;           ///< Weight on |delta| * length.
  double steer_change_penalty = 0.5;    ///< Weight on |delta - delta_parent|.
  double collision_step = 0.25;         ///< Pose spacing for footprint collision checks [m].
  int analytic_expansion_interval = 5;  ///< Try a Reeds-Shepp shot every N expansions.
  std::size_t max_expansions = 200000;
};

/// Hybrid A* output.
struct HybridAStarResult {
  bool found = false;
  std::vector<DirectedPose> path;  ///< Densely sampled, start to goal.
  double cost = 0.0;               ///< Search cost (with penalties).
  double length = 0.0;             ///< Driven distance [m].
  int gear_switches = 0;
  std::size_t expanded = 0;
  bool analytic_expansion = false;              ///< The final segment is a Reeds-Shepp shot.
  std::vector<geometry::Pose2> expanded_poses;  ///< In expansion order (visualisation).
};

/// Car-like planner over an Environment2D using the footprint for collision checks.
class HybridAStar {
 public:
  /// @throws std::invalid_argument for invalid configuration values.
  HybridAStar(const geometry::Environment2D& env, const models::BicycleParams& vehicle,
              const models::VehicleFootprint& footprint, HybridAStarConfig config = {});

  /// Plans from @p start to @p goal (rear-axle poses).
  [[nodiscard]] HybridAStarResult plan(const geometry::Pose2& start,
                                       const geometry::Pose2& goal) const;

  /// True if the footprint at @p pose is collision free.
  [[nodiscard]] bool pose_free(const geometry::Pose2& pose) const;

  [[nodiscard]] const HybridAStarConfig& config() const noexcept { return config_; }

 private:
  const geometry::Environment2D& env_;
  models::KinematicBicycle vehicle_;
  models::VehicleFootprint footprint_;
  HybridAStarConfig config_;
};

}  // namespace roboticscore::planning
