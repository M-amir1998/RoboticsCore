#pragma once

/// @file grid_search.hpp
/// @brief Graph search on occupancy grids: breadth-first search, Dijkstra, A* and Theta*.
///
/// All planners share GridMap::step_cost (unit orthogonal moves, sqrt(2) diagonals, no corner
/// cutting), so their results are directly comparable. Costs and lengths are reported in
/// metres. Ties in the open list are broken deterministically (lower f, then lower h, then
/// insertion order), so expansion counts are reproducible.
///
/// Complexity with a binary heap: O(E log V) time and O(V) memory, V = cells, E ~ k V.
///
///   BFS       optimal in number of steps only (ignores diagonal cost); O(V + E)
///   Dijkstra  optimal; expands every cell cheaper than the goal
///   A*        optimal with an admissible heuristic (see Heuristic); weighted A* with
///             weight w > 1 returns paths at most w times the optimum
///   Theta*    any-angle: parents may be any visible ancestor, yielding shorter, smoother
///             paths that are not guaranteed optimal (Nash et al., AAAI 2007)

#include <cstddef>
#include <functional>
#include <vector>

#include "roboticscore/planning/grid/grid_map.hpp"

namespace roboticscore::planning {

/// Outcome of a grid search.
enum class PlanStatus {
  kFound,
  kNoPath,
  kInvalidStart,  ///< Start is out of bounds or blocked.
  kInvalidGoal,   ///< Goal is out of bounds or blocked.
};

/// Search result and statistics.
struct GridPath {
  PlanStatus status = PlanStatus::kNoPath;
  std::vector<GridIndex> cells;  ///< Start to goal (waypoints only for Theta*).
  double cost = 0.0;             ///< Path length in metres.
  std::size_t expanded = 0;      ///< Nodes removed from the open list and expanded.
  std::size_t generated = 0;     ///< Nodes inserted into the open list.

  [[nodiscard]] bool found() const noexcept { return status == PlanStatus::kFound; }
};

/// Called each time a cell is expanded; used by demos to animate the search front.
using ExpansionObserver = std::function<void(const GridIndex&)>;

/// A* / Dijkstra configuration.
struct GridSearchConfig {
  Connectivity connectivity = Connectivity::kEight;
  Heuristic heuristic = Heuristic::kOctile;
  double heuristic_weight = 1.0;  ///< > 1 gives weighted A* (bounded suboptimal).
};

/// Fewest-steps path (each move costs one step regardless of direction).
[[nodiscard]] GridPath breadth_first_search(const GridMap& map, const GridIndex& start,
                                            const GridIndex& goal,
                                            Connectivity connectivity = Connectivity::kEight,
                                            const ExpansionObserver& observer = {});

/// Uniform-cost search (A* with the zero heuristic).
[[nodiscard]] GridPath dijkstra(const GridMap& map, const GridIndex& start, const GridIndex& goal,
                                Connectivity connectivity = Connectivity::kEight,
                                const ExpansionObserver& observer = {});

/// A* search.
/// @throws std::invalid_argument if heuristic_weight < 1.
[[nodiscard]] GridPath astar(const GridMap& map, const GridIndex& start, const GridIndex& goal,
                             const GridSearchConfig& config = {},
                             const ExpansionObserver& observer = {});

/// Theta* any-angle search. Line of sight requires every cell crossed by the segment between
/// cell centres to be free (exact traversal, conservative at corners).
[[nodiscard]] GridPath theta_star(const GridMap& map, const GridIndex& start, const GridIndex& goal,
                                  const GridSearchConfig& config = {},
                                  const ExpansionObserver& observer = {});

/// True if the straight segment between the centres of @p a and @p b crosses only free cells.
[[nodiscard]] bool line_of_sight(const GridMap& map, const GridIndex& a, const GridIndex& b);

}  // namespace roboticscore::planning
