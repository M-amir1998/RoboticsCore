#include "roboticscore/planning/grid/dstar_lite.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "roboticscore/core/error.hpp"

namespace roboticscore::planning {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Strict lexicographic order used for heap ordering (a strict weak ordering).
bool key_less(const std::pair<double, double>& a, const std::pair<double, double>& b) {
  return a.first < b.first || (a.first == b.first && a.second < b.second);
}

// Floating-point keys: keys of different vertices are sums of step costs and heuristics
// accumulated along different paths, so mathematically equal keys can differ by a few ulps.
// The heap must use a strict order, which may then place an inconsistent vertex whose key
// ties with key(start) behind a consistent one. Two tolerant tests keep the search correct:
//  - termination is conservative: the search continues while the top key's first component
//    is within tolerance of key(start), so every tie is processed (extra expansions are always
//    safe in D* Lite; stopping early is not);
//  - a popped vertex is only re-keyed if its key grew by more than the tolerance.
double key_tolerance(double value) {
  return 1e-9 * std::max(1.0, std::abs(value));
}

bool continue_search(const std::pair<double, double>& top, const std::pair<double, double>& start) {
  if (!std::isfinite(start.first)) {
    return key_less(top, start);
  }
  return top.first <= start.first + key_tolerance(start.first);
}

bool needs_rekey(const std::pair<double, double>& k_old, const std::pair<double, double>& k_new) {
  if (!std::isfinite(k_new.first)) {
    return key_less(k_old, k_new);
  }
  if (k_old.first < k_new.first - key_tolerance(k_new.first)) return true;
  if (k_old.first > k_new.first + key_tolerance(k_new.first)) return false;
  return k_old.second < k_new.second - key_tolerance(k_new.second);
}

}  // namespace

bool DStarLite::LowerPriority::operator()(const QueueEntry& a, const QueueEntry& b) const {
  if (a.key != b.key) {
    return key_less(b.key, a.key);
  }
  return a.order > b.order;
}

DStarLite::DStarLite(GridMap map, const GridIndex& start, const GridIndex& goal,
                     Connectivity connectivity)
    : map_(std::move(map)),
      start_(start),
      goal_(goal),
      last_(start),
      connectivity_(connectivity),
      heuristic_(connectivity == Connectivity::kEight ? Heuristic::kOctile
                                                      : Heuristic::kManhattan) {
  const auto& spec = map_.spec();
  detail::require(spec.in_bounds(start) && spec.in_bounds(goal),
                  "DStarLite: start and goal must lie inside the map");
  const std::size_t n = spec.cell_count();
  g_.assign(n, kInf);
  rhs_.assign(n, kInf);
  in_queue_.assign(n, 0);
  queued_key_.assign(n, {kInf, kInf});
  const std::size_t gi = spec.linear_index(goal_);
  rhs_[gi] = 0.0;
  push(gi);
}

double DStarLite::h(std::size_t a, std::size_t b) const {
  return heuristic_distance(heuristic_, map_.spec().from_linear(a), map_.spec().from_linear(b));
}

double DStarLite::cost(std::size_t a, std::size_t b) const {
  return map_.step_cost(map_.spec().from_linear(a), map_.spec().from_linear(b));
}

template <typename F>
void DStarLite::for_each_neighbor(std::size_t u, F&& f) const {
  const auto& spec = map_.spec();
  const GridIndex c = spec.from_linear(u);
  for (const GridIndex& off : GridMap::offsets(connectivity_)) {
    const GridIndex nb{c.x + off.x, c.y + off.y};
    if (spec.in_bounds(nb)) {
      f(spec.linear_index(nb));
    }
  }
}

DStarLite::Key DStarLite::calculate_key(std::size_t s) const {
  const double m = std::min(g_[s], rhs_[s]);
  return {m + h(map_.spec().linear_index(start_), s) + km_, m};
}

void DStarLite::push(std::size_t u) {
  const Key k = calculate_key(u);
  in_queue_[u] = 1;
  queued_key_[u] = k;
  queue_.push({k, order_++, u});
}

// Pops stale entries and returns the smallest valid key (or {inf, inf} if empty).
bool DStarLite::top_key(Key& key) {
  while (!queue_.empty()) {
    const QueueEntry& e = queue_.top();
    if (in_queue_[e.node] != 0 && queued_key_[e.node] == e.key) {
      key = e.key;
      return true;
    }
    queue_.pop();
  }
  key = {kInf, kInf};
  return false;
}

void DStarLite::update_vertex(std::size_t u) {
  const std::size_t goal_index = map_.spec().linear_index(goal_);
  if (u != goal_index) {
    double best = kInf;
    for_each_neighbor(u, [&](std::size_t s) { best = std::min(best, cost(u, s) + g_[s]); });
    rhs_[u] = best;
  }
  in_queue_[u] = 0;  // lazily removes any queued entry
  if (g_[u] != rhs_[u]) {
    push(u);
  }
}

PlanStatus DStarLite::compute_shortest_path(const ExpansionObserver& observer) {
  const auto& spec = map_.spec();
  const std::size_t s_start = spec.linear_index(start_);
  last_expanded_ = 0;
  Key top{};
  while (top_key(top)
         && (continue_search(top, calculate_key(s_start)) || rhs_[s_start] > g_[s_start])) {
    const std::size_t u = queue_.top().node;
    const Key k_old = top;
    const Key k_new = calculate_key(u);
    if (needs_rekey(k_old, k_new)) {
      push(u);  // re-key; the old entry becomes stale
      continue;
    }
    in_queue_[u] = 0;
    queue_.pop();
    ++last_expanded_;
    if (observer) {
      observer(spec.from_linear(u));
    }
    if (g_[u] > rhs_[u]) {
      g_[u] = rhs_[u];
      for_each_neighbor(u, [&](std::size_t s) { update_vertex(s); });
    } else {
      g_[u] = kInf;
      update_vertex(u);
      for_each_neighbor(u, [&](std::size_t s) { update_vertex(s); });
    }
  }
  total_expanded_ += last_expanded_;
  // The termination test only requires rhs(start) <= g(start): the start itself may remain
  // locally overconsistent (g = inf), and its cost-to-goal is rhs(start) = min_s c + g(s).
  return rhs_[s_start] < kInf ? PlanStatus::kFound : PlanStatus::kNoPath;
}

void DStarLite::move_start(const GridIndex& cell) {
  detail::require(map_.spec().in_bounds(cell), "DStarLite::move_start: cell outside the map");
  start_ = cell;
}

std::size_t DStarLite::update_cells(const std::vector<CellChange>& changes) {
  const auto& spec = map_.spec();
  std::vector<std::size_t> touched;
  std::size_t changed = 0;
  for (const CellChange& ch : changes) {
    if (!spec.in_bounds(ch.cell) || map_.blocked(ch.cell) == ch.blocked) {
      continue;
    }
    if (changed == 0) {
      // Robot may have moved since the last search: shift keys lazily via km.
      km_ += h(spec.linear_index(last_), spec.linear_index(start_));
      last_ = start_;
    }
    ++changed;
    map_.set_blocked(ch.cell, ch.blocked);
    // Every edge whose cost can change has both endpoints in the 3x3 block around the cell
    // (including diagonals that would cut its corner).
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const GridIndex n{ch.cell.x + dx, ch.cell.y + dy};
        if (spec.in_bounds(n)) {
          touched.push_back(spec.linear_index(n));
        }
      }
    }
  }
  std::sort(touched.begin(), touched.end());
  touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
  for (const std::size_t u : touched) {
    update_vertex(u);
  }
  return changed;
}

std::vector<GridIndex> DStarLite::path() const {
  const auto& spec = map_.spec();
  std::vector<GridIndex> cells;
  std::size_t u = spec.linear_index(start_);
  const std::size_t goal_index = spec.linear_index(goal_);
  if (rhs_[u] == kInf) {
    return cells;
  }
  cells.push_back(start_);
  std::vector<std::uint8_t> visited(spec.cell_count(), 0);
  visited[u] = 1;
  for (std::size_t steps = 0; u != goal_index && steps < spec.cell_count(); ++steps) {
    double best = kInf;
    std::size_t next = u;
    for_each_neighbor(u, [&](std::size_t s) {
      const double c = cost(u, s) + g_[s];
      if (c < best) {
        best = c;
        next = s;
      }
    });
    if (best == kInf || visited[next] != 0) {
      return {};  // no path, or inconsistent g-values (cannot happen after a full search)
    }
    visited[next] = 1;
    u = next;
    cells.push_back(spec.from_linear(u));
  }
  return cells;
}

GridIndex DStarLite::next_step() const {
  const auto p = path();
  return p.size() >= 2 ? p[1] : start_;
}

double DStarLite::g(const GridIndex& c) const {
  return g_.at(map_.spec().linear_index(c));
}

double DStarLite::rhs(const GridIndex& c) const {
  return rhs_.at(map_.spec().linear_index(c));
}

}  // namespace roboticscore::planning
