#include "roboticscore/planning/grid/grid_search.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <queue>

#include "roboticscore/core/error.hpp"

namespace roboticscore::planning {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();

struct OpenEntry {
  double f;
  double h;
  std::uint64_t order;
  std::size_t node;
  double g;
};

// std::priority_queue is a max-heap: "a < b" means a has lower priority.
struct LowerPriority {
  bool operator()(const OpenEntry& a, const OpenEntry& b) const {
    if (a.f != b.f) return a.f > b.f;
    if (a.h != b.h) return a.h > b.h;
    return a.order > b.order;
  }
};

PlanStatus validate(const GridMap& map, const GridIndex& start, const GridIndex& goal) {
  if (map.blocked(start)) return PlanStatus::kInvalidStart;
  if (map.blocked(goal)) return PlanStatus::kInvalidGoal;
  return PlanStatus::kFound;
}

std::vector<GridIndex> reconstruct(const GridSpec& spec, const std::vector<std::size_t>& parent,
                                   std::size_t goal) {
  std::vector<GridIndex> cells;
  for (std::size_t n = goal;; n = parent[n]) {
    cells.push_back(spec.from_linear(n));
    if (parent[n] == n) {
      break;
    }
  }
  std::reverse(cells.begin(), cells.end());
  return cells;
}

int sign_of(int v) {
  if (v > 0) return 1;
  if (v < 0) return -1;
  return 0;
}

double euclid(const GridIndex& a, const GridIndex& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

GridPath best_first(const GridMap& map, const GridIndex& start, const GridIndex& goal,
                    Connectivity connectivity, Heuristic heuristic, double weight, bool any_angle,
                    const ExpansionObserver& observer) {
  GridPath result;
  result.status = validate(map, start, goal);
  if (result.status != PlanStatus::kFound) {
    return result;
  }
  const GridSpec& spec = map.spec();
  const std::size_t n_cells = spec.cell_count();
  std::vector<double> g(n_cells, kInf);
  std::vector<std::size_t> parent(n_cells, kNoParent);
  std::vector<std::uint8_t> closed(n_cells, 0);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, LowerPriority> open;
  std::uint64_t order = 0;

  const std::size_t s = spec.linear_index(start);
  const std::size_t t = spec.linear_index(goal);
  g[s] = 0.0;
  parent[s] = s;
  const double h0 = weight * heuristic_distance(heuristic, start, goal);
  open.push({h0, h0, order++, s, 0.0});
  result.generated = 1;

  while (!open.empty()) {
    const OpenEntry e = open.top();
    open.pop();
    if (closed[e.node] != 0 || e.g > g[e.node]) {
      continue;  // stale entry
    }
    closed[e.node] = 1;
    ++result.expanded;
    const GridIndex c = spec.from_linear(e.node);
    if (observer) {
      observer(c);
    }
    if (e.node == t) {
      result.cells = reconstruct(spec, parent, t);
      result.cost = path_length(spec, result.cells);
      result.status = PlanStatus::kFound;
      return result;
    }
    const std::size_t pc = parent[e.node];
    const GridIndex parent_cell = spec.from_linear(pc);
    for (const GridIndex& off : GridMap::offsets(connectivity)) {
      const GridIndex nb{c.x + off.x, c.y + off.y};
      const double step = map.step_cost(c, nb);
      if (step == kInf) {
        continue;
      }
      const std::size_t ni = spec.linear_index(nb);
      if (closed[ni] != 0) {
        continue;
      }
      std::size_t cand_parent = e.node;
      double cand_g = g[e.node] + step;
      if (any_angle && pc != e.node && line_of_sight(map, parent_cell, nb)) {
        cand_parent = pc;
        cand_g = g[pc] + euclid(parent_cell, nb);
      }
      if (cand_g < g[ni]) {
        g[ni] = cand_g;
        parent[ni] = cand_parent;
        const double h = weight * heuristic_distance(heuristic, nb, goal);
        open.push({cand_g + h, h, order++, ni, cand_g});
        ++result.generated;
      }
    }
  }
  result.status = PlanStatus::kNoPath;
  return result;
}

}  // namespace

bool line_of_sight(const GridMap& map, const GridIndex& a, const GridIndex& b) {
  if (map.blocked(a) || map.blocked(b)) {
    return false;
  }
  // Amanatides-Woo traversal between cell centres in grid units. Centres never lie on cell
  // boundaries, so the start cell is unambiguous.
  const int dx = b.x - a.x;
  const int dy = b.y - a.y;
  const int sx = sign_of(dx);
  const int sy = sign_of(dy);
  const double t_delta_x = dx != 0 ? 1.0 / std::abs(dx) : kInf;
  const double t_delta_y = dy != 0 ? 1.0 / std::abs(dy) : kInf;
  double t_max_x = dx != 0 ? 0.5 * t_delta_x : kInf;
  double t_max_y = dy != 0 ? 0.5 * t_delta_y : kInf;
  GridIndex c = a;
  while (!(c == b)) {
    if (std::abs(t_max_x - t_max_y) <= 1e-12) {
      // Passing exactly through a corner: both edge-adjacent cells must be free.
      if (map.blocked({c.x + sx, c.y}) || map.blocked({c.x, c.y + sy})) {
        return false;
      }
      c = {c.x + sx, c.y + sy};
      t_max_x += t_delta_x;
      t_max_y += t_delta_y;
    } else if (t_max_x < t_max_y) {
      c.x += sx;
      t_max_x += t_delta_x;
    } else {
      c.y += sy;
      t_max_y += t_delta_y;
    }
    if (map.blocked(c)) {
      return false;
    }
  }
  return true;
}

GridPath breadth_first_search(const GridMap& map, const GridIndex& start, const GridIndex& goal,
                              Connectivity connectivity, const ExpansionObserver& observer) {
  GridPath result;
  result.status = validate(map, start, goal);
  if (result.status != PlanStatus::kFound) {
    return result;
  }
  const GridSpec& spec = map.spec();
  std::vector<std::size_t> parent(spec.cell_count(), kNoParent);
  const std::size_t s = spec.linear_index(start);
  const std::size_t t = spec.linear_index(goal);
  parent[s] = s;
  std::deque<std::size_t> queue{s};
  result.generated = 1;
  while (!queue.empty()) {
    const std::size_t n = queue.front();
    queue.pop_front();
    ++result.expanded;
    const GridIndex c = spec.from_linear(n);
    if (observer) {
      observer(c);
    }
    if (n == t) {
      result.cells = reconstruct(spec, parent, t);
      result.cost = path_length(spec, result.cells);
      return result;
    }
    for (const GridIndex& off : GridMap::offsets(connectivity)) {
      const GridIndex nb{c.x + off.x, c.y + off.y};
      if (map.step_cost(c, nb) == kInf) {
        continue;
      }
      const std::size_t ni = spec.linear_index(nb);
      if (parent[ni] == kNoParent) {
        parent[ni] = n;
        queue.push_back(ni);
        ++result.generated;
      }
    }
  }
  result.status = PlanStatus::kNoPath;
  return result;
}

GridPath dijkstra(const GridMap& map, const GridIndex& start, const GridIndex& goal,
                  Connectivity connectivity, const ExpansionObserver& observer) {
  return best_first(map, start, goal, connectivity, Heuristic::kZero, 1.0, false, observer);
}

GridPath astar(const GridMap& map, const GridIndex& start, const GridIndex& goal,
               const GridSearchConfig& config, const ExpansionObserver& observer) {
  detail::require(config.heuristic_weight >= 1.0, "astar: heuristic_weight must be >= 1");
  return best_first(map, start, goal, config.connectivity, config.heuristic,
                    config.heuristic_weight, false, observer);
}

GridPath theta_star(const GridMap& map, const GridIndex& start, const GridIndex& goal,
                    const GridSearchConfig& config, const ExpansionObserver& observer) {
  detail::require(config.heuristic_weight >= 1.0, "theta_star: heuristic_weight must be >= 1");
  // Any-angle paths can be shorter than octile distances, so only the Euclidean heuristic is
  // admissible here; the configured heuristic is intentionally ignored.
  return best_first(map, start, goal, config.connectivity, Heuristic::kEuclidean,
                    config.heuristic_weight, true, observer);
}

}  // namespace roboticscore::planning
