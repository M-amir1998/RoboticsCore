#pragma once

/// @file potential_field.hpp
/// @brief Artificial potential field navigation (Khatib, "Real-Time Obstacle Avoidance for
///        Manipulators and Mobile Robots", IJRR 1986), included as an educational baseline.
///
///   U_att(p) = 0.5 k_att |p - g|^2                     for |p - g| <= d*
///            = k_att d* |p - g| - 0.5 k_att d*^2       otherwise (conic, bounded force)
///   U_rep(p) = 0.5 k_rep (1/rho(p) - 1/rho_0)^2        for rho(p) <= rho_0, else 0
/// where rho is the clearance to the nearest obstacle. The robot follows -grad U.
///
/// Known failure modes (Koren & Borenstein, ICRA 1991), all reproducible with this code:
///   - local minima where attraction and repulsion cancel (e.g. a U-shaped obstacle);
///   - oscillation in narrow corridors and close to obstacles;
///   - goals non-reachable with obstacles nearby (repulsion dominates near the goal);
///   - no passage between closely spaced obstacles.
/// The planner reports these as a status instead of looping forever.

#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/environment.hpp"

namespace roboticscore::planning {

/// Potential-field parameters.
struct PotentialFieldConfig {
  double attractive_gain = 1.0;
  double repulsive_gain = 2.0;
  double influence_distance = 2.0;  ///< rho_0 [m]
  double conic_threshold = 3.0;     ///< d* [m]
  double step_size = 0.05;          ///< Distance moved per iteration [m].
  double goal_tolerance = 0.2;      ///< [m]
  int max_iterations = 4000;
  int stall_window = 200;       ///< Iterations over which progress is measured.
  double stall_distance = 0.3;  ///< Net displacement below this over the window = stuck.
};

enum class PotentialFieldStatus { kReachedGoal, kLocalMinimum, kMaxIterations, kCollision };

/// Result of following the potential field.
struct PotentialFieldResult {
  PotentialFieldStatus status = PotentialFieldStatus::kMaxIterations;
  std::vector<Vec2> path;
  std::size_t iterations = 0;
};

/// Total potential U_att + U_rep at @p p.
[[nodiscard]] double potential(const Vec2& p, const Vec2& goal, const geometry::Environment2D& env,
                               const PotentialFieldConfig& config);

/// Force -grad U at @p p (repulsive gradient via central differences of the clearance).
[[nodiscard]] Vec2 potential_force(const Vec2& p, const Vec2& goal,
                                   const geometry::Environment2D& env,
                                   const PotentialFieldConfig& config);

/// Follows the normalised force from @p start until the goal, a local minimum or the
/// iteration limit is reached.
/// @throws std::invalid_argument for invalid configuration values.
[[nodiscard]] PotentialFieldResult potential_field_plan(const geometry::Environment2D& env,
                                                        const Vec2& start, const Vec2& goal,
                                                        const PotentialFieldConfig& config);

}  // namespace roboticscore::planning
