#pragma once

/// @file dstar_lite.hpp
/// @brief D* Lite incremental replanning on a grid (Koenig & Likhachev, AAAI 2002).
///
/// D* Lite searches backwards from the goal and keeps g (current cost-to-goal estimates) and
/// rhs (one-step lookahead values). When cells change, only the vertices whose costs are
/// affected become inconsistent (g != rhs) and are repaired, so replanning after a local
/// change typically expands far fewer nodes than a fresh A* search. The key modifier km lets
/// the robot move without re-sorting the priority queue.
///
/// Usage pattern for a robot following the plan:
///
///   DStarLite planner(map, start, goal);
///   planner.compute_shortest_path();
///   while (robot not at goal) {
///     robot moves to planner.next_step();   planner.move_start(new_cell);
///     if (sensors detect changes) { planner.update_cells(changes);
///     planner.compute_shortest_path(); }
///   }
///
/// Complexity: the first search is equivalent to a backward A*; each replanning is bounded by
/// the number of inconsistent vertices, O(k log k) for k affected vertices in practice.

#include <cstddef>
#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

#include "roboticscore/planning/grid/grid_map.hpp"
#include "roboticscore/planning/grid/grid_search.hpp"

namespace roboticscore::planning {

/// A single cell state change reported by the robot's sensors.
struct CellChange {
  GridIndex cell;
  bool blocked = true;
};

/// Incremental planner.
class DStarLite {
 public:
  /// @throws std::invalid_argument if start or goal lie outside the map.
  DStarLite(GridMap map, const GridIndex& start, const GridIndex& goal,
            Connectivity connectivity = Connectivity::kEight);

  /// (Re)computes the shortest path from the current start; returns kFound or kNoPath.
  /// Expansions of this call are reported to @p observer.
  PlanStatus compute_shortest_path(const ExpansionObserver& observer = {});

  /// Moves the start (robot position) to @p cell. Call compute_shortest_path() afterwards only
  /// if update_cells() reported changes; moving alone does not invalidate the plan.
  /// @throws std::invalid_argument if @p cell is outside the map.
  void move_start(const GridIndex& cell);

  /// Applies cell changes and marks the affected vertices inconsistent. Returns the number
  /// of cells whose state actually changed.
  std::size_t update_cells(const std::vector<CellChange>& changes);

  /// Current best path from start to goal (empty if none). Extracted greedily from g.
  [[nodiscard]] std::vector<GridIndex> path() const;

  /// Next cell to move to from the current start, or the start itself if no path exists.
  [[nodiscard]] GridIndex next_step() const;

  [[nodiscard]] const GridMap& map() const noexcept { return map_; }
  [[nodiscard]] const GridIndex& start() const noexcept { return start_; }
  [[nodiscard]] const GridIndex& goal() const noexcept { return goal_; }

  /// Cost-to-goal estimate g in cell units (infinity if unknown/unreachable). After a search
  /// the start may keep g = inf; its cost-to-goal is rhs(start).
  [[nodiscard]] double g(const GridIndex& c) const;
  /// One-step lookahead value rhs(c) = min over successors of c(c, s) + g(s).
  [[nodiscard]] double rhs(const GridIndex& c) const;

  /// Total and last-call expansion counts.
  [[nodiscard]] std::size_t total_expanded() const noexcept { return total_expanded_; }
  [[nodiscard]] std::size_t last_expanded() const noexcept { return last_expanded_; }

 private:
  using Key = std::pair<double, double>;
  struct QueueEntry {
    Key key;
    std::uint64_t order;
    std::size_t node;
  };
  struct LowerPriority {
    bool operator()(const QueueEntry& a, const QueueEntry& b) const;
  };

  [[nodiscard]] Key calculate_key(std::size_t s) const;
  [[nodiscard]] double h(std::size_t a, std::size_t b) const;
  [[nodiscard]] double cost(std::size_t a, std::size_t b) const;
  void update_vertex(std::size_t u);
  void push(std::size_t u);
  [[nodiscard]] bool top_key(Key& key);
  template <typename F>
  void for_each_neighbor(std::size_t u, F&& f) const;

  GridMap map_;
  GridIndex start_;
  GridIndex goal_;
  GridIndex last_;
  Connectivity connectivity_;
  Heuristic heuristic_;
  double km_ = 0.0;
  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<std::uint8_t> in_queue_;
  std::vector<Key> queued_key_;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LowerPriority> queue_;
  std::uint64_t order_ = 0;
  std::size_t total_expanded_ = 0;
  std::size_t last_expanded_ = 0;
};

}  // namespace roboticscore::planning
