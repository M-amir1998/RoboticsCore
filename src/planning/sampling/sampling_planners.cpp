#include "roboticscore/planning/sampling/sampling_planners.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <queue>
#include <unordered_set>
#include <utility>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/spatial/kd_tree.hpp"

namespace roboticscore::planning {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

void validate(const SamplingConfig& c) {
  detail::require(c.step_size > 0.0, "sampling planner: step_size must be positive");
  detail::require(c.goal_bias >= 0.0 && c.goal_bias <= 1.0,
                  "sampling planner: goal_bias must lie in [0, 1]");
  detail::require(c.goal_tolerance >= 0.0, "sampling planner: goal_tolerance must be >= 0");
  detail::require(c.max_iterations > 0, "sampling planner: max_iterations must be positive");
  detail::require(c.robot_radius >= 0.0, "sampling planner: robot_radius must be >= 0");
  detail::require(c.rewire_factor > 0.0, "sampling planner: rewire_factor must be positive");
  detail::require(c.prm_samples > 0 && c.prm_neighbors > 0,
                  "sampling planner: PRM sample and neighbour counts must be positive");
}

// Tree with incremental nearest-neighbour index and child lists for cost propagation.
class Tree {
 public:
  std::size_t add(const Vec2& p, std::ptrdiff_t parent, double cost) {
    nodes_.push_back({p, parent, cost});
    kd_.insert(p);
    children_.emplace_back();
    if (parent >= 0) {
      children_[static_cast<std::size_t>(parent)].push_back(nodes_.size() - 1);
    }
    return nodes_.size() - 1;
  }

  // Moves `child` under `new_parent` with cost `new_cost` and shifts all descendants.
  void reparent(std::size_t child, std::size_t new_parent, double new_cost) {
    auto& siblings = children_[static_cast<std::size_t>(nodes_[child].parent)];
    siblings.erase(std::find(siblings.begin(), siblings.end(), child));
    children_[new_parent].push_back(child);
    nodes_[child].parent = static_cast<std::ptrdiff_t>(new_parent);
    const double delta = new_cost - nodes_[child].cost;
    std::vector<std::size_t> stack{child};
    while (!stack.empty()) {
      const std::size_t n = stack.back();
      stack.pop_back();
      nodes_[n].cost += delta;
      stack.insert(stack.end(), children_[n].begin(), children_[n].end());
    }
  }

  [[nodiscard]] std::vector<Vec2> path_to(std::size_t n) const {
    std::vector<Vec2> path;
    for (std::ptrdiff_t i = static_cast<std::ptrdiff_t>(n); i >= 0;
         i = nodes_[static_cast<std::size_t>(i)].parent) {
      path.push_back(nodes_[static_cast<std::size_t>(i)].point);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  [[nodiscard]] const std::vector<TreeNode>& nodes() const noexcept { return nodes_; }
  [[nodiscard]] const TreeNode& node(std::size_t i) const { return nodes_[i]; }
  [[nodiscard]] const spatial::KdTree2D& index() const noexcept { return kd_; }
  [[nodiscard]] std::size_t size() const noexcept { return nodes_.size(); }

 private:
  std::vector<TreeNode> nodes_;
  spatial::KdTree2D kd_;
  std::vector<std::vector<std::size_t>> children_;
};

// Shared sampling / collision-checking context.
class Context {
 public:
  Context(const geometry::Environment2D& env, const SamplingConfig& config)
      : env_(env), config_(config), rng_(config.seed) {
    const auto& b = env.bounds();
    lo_ = b.min + Vec2::Constant(config.robot_radius);
    hi_ = b.max - Vec2::Constant(config.robot_radius);
  }

  bool segment_free(const Vec2& a, const Vec2& b) {
    ++checks;
    return env_.segment_free(a, b, config_.robot_radius);
  }
  bool point_free(const Vec2& p) {
    ++checks;
    return env_.point_free(p, config_.robot_radius);
  }
  Vec2 uniform() {
    ++samples;
    return {rng_.uniform(lo_.x(), hi_.x()), rng_.uniform(lo_.y(), hi_.y())};
  }
  Vec2 biased(const Vec2& goal) {
    if (rng_.uniform01() < config_.goal_bias) {
      ++samples;
      return goal;
    }
    return uniform();
  }
  [[nodiscard]] bool inside(const Vec2& p) const {
    return p.x() >= lo_.x() && p.x() <= hi_.x() && p.y() >= lo_.y() && p.y() <= hi_.y();
  }
  math::Random& rng() { return rng_; }
  [[nodiscard]] Vec2 steer(const Vec2& from, const Vec2& to) const {
    const Vec2 d = to - from;
    const double n = d.norm();
    return n <= config_.step_size ? to : Vec2(from + d * (config_.step_size / n));
  }
  [[nodiscard]] double area() const { return (hi_ - lo_).prod(); }

  std::size_t checks = 0;
  std::size_t samples = 0;

 private:
  const geometry::Environment2D& env_;
  const SamplingConfig& config_;
  math::Random rng_;
  Vec2 lo_;
  Vec2 hi_;
};

void notify(const SamplingObserver& obs, SamplingEvent e) {
  if (obs) {
    obs(e);
  }
}

// Returns true (and fills result) when start and goal are trivially connected or invalid.
bool handle_trivial(Context& ctx, const Vec2& start, const Vec2& goal, const SamplingConfig& config,
                    SamplingResult& result) {
  if (!ctx.point_free(start) || !ctx.point_free(goal)) {
    return true;  // found = false
  }
  if ((goal - start).norm() <= config.goal_tolerance && ctx.segment_free(start, goal)) {
    result.found = true;
    result.path = {start, goal};
    result.cost = (goal - start).norm();
    result.tree = {{start, -1, 0.0}};
    result.cost_history.emplace_back(0, result.cost);
    return true;
  }
  return false;
}

void finalize_stats(const Context& ctx, SamplingResult& r) {
  r.collision_checks = ctx.checks;
  r.samples = ctx.samples;
}

SamplingResult rrt_star_impl(const geometry::Environment2D& env, const Vec2& start,
                             const Vec2& goal, const SamplingConfig& config, bool informed,
                             const SamplingObserver& observer) {
  validate(config);
  Context ctx(env, config);
  SamplingResult result;
  if (handle_trivial(ctx, start, goal, config, result)) {
    finalize_stats(ctx, result);
    return result;
  }
  Tree tree;
  tree.add(start, -1, 0.0);
  // gamma* = 2 (1 + 1/d)^(1/d) (mu(X_free) / zeta_d)^(1/d) with d = 2, zeta_2 = pi; the
  // workspace area is used as an upper bound of mu(X_free).
  const double gamma =
      config.rewire_factor * 2.0 * std::sqrt(1.5) * std::sqrt(ctx.area() / std::numbers::pi);
  std::vector<std::size_t> goal_parents;
  double best_cost = kInf;
  std::ptrdiff_t best_parent = -1;

  for (int it = 1; it <= config.max_iterations; ++it) {
    result.iterations = static_cast<std::size_t>(it);
    Vec2 x_rand;
    if (informed && best_cost < kInf) {
      // Rejection only for the workspace bounds; the ellipse is sampled exactly.
      int tries = 0;
      do {
        ++ctx.samples;
        x_rand = sample_informed_ellipse(start, goal, best_cost, ctx.rng().uniform01(),
                                         ctx.rng().uniform01());
      } while (!ctx.inside(x_rand) && ++tries < 100);
    } else {
      x_rand = ctx.biased(goal);
    }
    const auto nn = tree.index().nearest(x_rand);
    const Vec2 x_nearest = tree.node(nn->id).point;
    const Vec2 x_new = ctx.steer(x_nearest, x_rand);
    if ((x_new - x_nearest).squaredNorm() == 0.0 || !ctx.segment_free(x_nearest, x_new)) {
      continue;
    }
    const auto n = static_cast<double>(tree.size() + 1);
    const double radius = std::min(gamma * std::sqrt(std::log(n) / n), config.step_size);
    const auto near = tree.index().radius_search(x_new, radius);

    // Choose parent; remember collision results for rewiring (-1 unknown, 0 blocked, 1 free).
    std::vector<int> free_cache(near.size(), -1);
    std::size_t parent = nn->id;
    double cost = tree.node(nn->id).cost + (x_new - x_nearest).norm();
    for (std::size_t k = 0; k < near.size(); ++k) {
      const std::size_t j = near[k].id;
      if (j == nn->id) {
        free_cache[k] = 1;
        continue;
      }
      const double c = tree.node(j).cost + std::sqrt(near[k].distance_squared);
      if (c < cost) {
        free_cache[k] = ctx.segment_free(tree.node(j).point, x_new) ? 1 : 0;
        if (free_cache[k] == 1) {
          parent = j;
          cost = c;
        }
      }
    }
    const std::size_t id = tree.add(x_new, static_cast<std::ptrdiff_t>(parent), cost);
    notify(observer, {SamplingEvent::Type::kNodeAdded, result.iterations, id,
                      static_cast<std::ptrdiff_t>(parent), -1, cost, 0});

    // Rewire neighbours through the new node when that is cheaper.
    for (std::size_t k = 0; k < near.size(); ++k) {
      const std::size_t j = near[k].id;
      if (j == parent) {
        continue;
      }
      const double c = cost + std::sqrt(near[k].distance_squared);
      if (c + 1e-12 >= tree.node(j).cost) {
        continue;
      }
      if (free_cache[k] == -1) {
        free_cache[k] = ctx.segment_free(x_new, tree.node(j).point) ? 1 : 0;
      }
      if (free_cache[k] == 1) {
        const std::ptrdiff_t old_parent = tree.node(j).parent;
        tree.reparent(j, id, c);
        notify(observer, {SamplingEvent::Type::kRewired, result.iterations, j,
                          static_cast<std::ptrdiff_t>(id), old_parent, c, 0});
      }
    }

    if ((x_new - goal).norm() <= config.goal_tolerance && ctx.segment_free(x_new, goal)) {
      goal_parents.push_back(id);
    }
    // Costs of goal parents can drop through rewiring, so re-evaluate all of them.
    for (const std::size_t gp : goal_parents) {
      const double c = tree.node(gp).cost + (tree.node(gp).point - goal).norm();
      if (c < best_cost - 1e-12) {
        if (best_cost == kInf) {
          result.first_solution_iteration = result.iterations;
        }
        best_cost = c;
        best_parent = static_cast<std::ptrdiff_t>(gp);
        result.cost_history.emplace_back(result.iterations, best_cost);
        notify(observer, {SamplingEvent::Type::kSolutionImproved, result.iterations, gp, -1, -1,
                          best_cost, 0});
      }
    }
    if (config.stop_at_first_solution && best_parent >= 0) {
      break;
    }
  }
  result.tree = tree.nodes();
  if (best_parent >= 0) {
    result.found = true;
    result.path = tree.path_to(static_cast<std::size_t>(best_parent));
    result.path.push_back(goal);
    result.cost = polyline_length(result.path);
  }
  finalize_stats(ctx, result);
  return result;
}

}  // namespace

double polyline_length(const std::vector<Vec2>& path) {
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    len += (path[i] - path[i - 1]).norm();
  }
  return len;
}

Vec2 sample_informed_ellipse(const Vec2& a, const Vec2& b, double c_best, double u_radius,
                             double u_angle) {
  const double c_min = (b - a).norm();
  detail::require(c_best >= c_min, "sample_informed_ellipse: c_best must be >= |b - a|");
  const double r1 = 0.5 * c_best;
  const double r2 = 0.5 * std::sqrt(std::max(0.0, c_best * c_best - c_min * c_min));
  // Uniform point in the unit disc (sqrt for area-uniform radius), scaled and rotated.
  const double rho = std::sqrt(u_radius);
  const double theta = 2.0 * std::numbers::pi * u_angle;
  const Vec2 local(r1 * rho * std::cos(theta), r2 * rho * std::sin(theta));
  const double phi = std::atan2(b.y() - a.y(), b.x() - a.x());
  const double c = std::cos(phi);
  const double s = std::sin(phi);
  return 0.5 * (a + b) + Vec2(c * local.x() - s * local.y(), s * local.x() + c * local.y());
}

SamplingResult rrt(const geometry::Environment2D& env, const Vec2& start, const Vec2& goal,
                   const SamplingConfig& config, const SamplingObserver& observer) {
  validate(config);
  Context ctx(env, config);
  SamplingResult result;
  if (handle_trivial(ctx, start, goal, config, result)) {
    finalize_stats(ctx, result);
    return result;
  }
  Tree tree;
  tree.add(start, -1, 0.0);
  for (int it = 1; it <= config.max_iterations; ++it) {
    result.iterations = static_cast<std::size_t>(it);
    const Vec2 x_rand = ctx.biased(goal);
    const auto nn = tree.index().nearest(x_rand);
    const Vec2 x_near = tree.node(nn->id).point;
    const Vec2 x_new = ctx.steer(x_near, x_rand);
    if ((x_new - x_near).squaredNorm() == 0.0 || !ctx.segment_free(x_near, x_new)) {
      continue;
    }
    const double cost = tree.node(nn->id).cost + (x_new - x_near).norm();
    const std::size_t id = tree.add(x_new, static_cast<std::ptrdiff_t>(nn->id), cost);
    notify(observer, {SamplingEvent::Type::kNodeAdded, result.iterations, id,
                      static_cast<std::ptrdiff_t>(nn->id), -1, cost, 0});
    if ((x_new - goal).norm() <= config.goal_tolerance && ctx.segment_free(x_new, goal)) {
      result.found = true;
      result.first_solution_iteration = result.iterations;
      result.path = tree.path_to(id);
      result.path.push_back(goal);
      result.cost = polyline_length(result.path);
      result.cost_history.emplace_back(result.iterations, result.cost);
      notify(observer, {SamplingEvent::Type::kSolutionImproved, result.iterations, id, -1, -1,
                        result.cost, 0});
      break;
    }
  }
  result.tree = tree.nodes();
  finalize_stats(ctx, result);
  return result;
}

SamplingResult rrt_connect(const geometry::Environment2D& env, const Vec2& start, const Vec2& goal,
                           const SamplingConfig& config, const SamplingObserver& observer) {
  validate(config);
  Context ctx(env, config);
  SamplingResult result;
  if (handle_trivial(ctx, start, goal, config, result)) {
    finalize_stats(ctx, result);
    return result;
  }
  Tree trees[2];
  trees[0].add(start, -1, 0.0);
  trees[1].add(goal, -1, 0.0);
  int a = 0;  // tree being extended towards the sample

  enum class Status { kTrapped, kAdvanced, kReached };
  const auto extend = [&](int t, const Vec2& target, std::size_t& new_id) {
    const auto nn = trees[t].index().nearest(target);
    const Vec2 x_near = trees[t].node(nn->id).point;
    const Vec2 x_new = ctx.steer(x_near, target);
    if ((x_new - x_near).squaredNorm() == 0.0) {
      new_id = nn->id;
      return Status::kReached;
    }
    if (!ctx.segment_free(x_near, x_new)) {
      return Status::kTrapped;
    }
    const double cost = trees[t].node(nn->id).cost + (x_new - x_near).norm();
    new_id = trees[t].add(x_new, static_cast<std::ptrdiff_t>(nn->id), cost);
    notify(observer, {SamplingEvent::Type::kNodeAdded, result.iterations, new_id,
                      static_cast<std::ptrdiff_t>(nn->id), -1, cost, t});
    return (x_new - target).squaredNorm() == 0.0 ? Status::kReached : Status::kAdvanced;
  };

  for (int it = 1; it <= config.max_iterations; ++it) {
    result.iterations = static_cast<std::size_t>(it);
    const Vec2 x_rand = ctx.uniform();
    std::size_t a_new = 0;
    if (extend(a, x_rand, a_new) != Status::kTrapped) {
      const Vec2 target = trees[a].node(a_new).point;
      const int b = 1 - a;
      std::size_t b_new = 0;
      Status s = Status::kAdvanced;
      while (s == Status::kAdvanced) {
        s = extend(b, target, b_new);
      }
      if (s == Status::kReached) {
        // Join the two half-paths; tree 0 is always rooted at the start.
        std::vector<Vec2> from_start = trees[0].path_to(a == 0 ? a_new : b_new);
        std::vector<Vec2> from_goal = trees[1].path_to(a == 0 ? b_new : a_new);
        from_goal.pop_back();  // the meeting point is already the last start-tree vertex
        std::reverse(from_goal.begin(), from_goal.end());
        from_start.insert(from_start.end(), from_goal.begin(), from_goal.end());
        result.found = true;
        result.first_solution_iteration = result.iterations;
        result.path = std::move(from_start);
        result.cost = polyline_length(result.path);
        result.cost_history.emplace_back(result.iterations, result.cost);
        break;
      }
    }
    a = 1 - a;
  }
  result.tree = trees[0].nodes();
  result.goal_tree = trees[1].nodes();
  finalize_stats(ctx, result);
  return result;
}

SamplingResult rrt_star(const geometry::Environment2D& env, const Vec2& start, const Vec2& goal,
                        const SamplingConfig& config, const SamplingObserver& observer) {
  return rrt_star_impl(env, start, goal, config, false, observer);
}

SamplingResult informed_rrt_star(const geometry::Environment2D& env, const Vec2& start,
                                 const Vec2& goal, const SamplingConfig& config,
                                 const SamplingObserver& observer) {
  return rrt_star_impl(env, start, goal, config, true, observer);
}

SamplingResult prm(const geometry::Environment2D& env, const Vec2& start, const Vec2& goal,
                   const SamplingConfig& config) {
  validate(config);
  Context ctx(env, config);
  SamplingResult result;
  if (handle_trivial(ctx, start, goal, config, result)) {
    finalize_stats(ctx, result);
    return result;
  }
  // Milestones: start (0), goal (1), then collision-free uniform samples.
  std::vector<Vec2> pts = {start, goal};
  const int max_attempts = 100 * config.prm_samples;
  for (int attempt = 0;
       attempt < max_attempts && static_cast<int>(pts.size()) < config.prm_samples + 2; ++attempt) {
    const Vec2 p = ctx.uniform();
    if (ctx.point_free(p)) {
      pts.push_back(p);
    }
  }
  const spatial::KdTree2D kd(pts);
  std::vector<std::vector<std::pair<std::size_t, double>>> adj(pts.size());
  std::unordered_set<std::uint64_t> tested;  // undirected pairs already checked
  for (std::size_t i = 0; i < pts.size(); ++i) {
    for (const auto& nb :
         kd.k_nearest(pts[i], static_cast<std::size_t>(config.prm_neighbors) + 1)) {
      const std::size_t j = nb.id;
      if (j == i) continue;
      const std::size_t lo = std::min(i, j);
      const std::size_t hi = std::max(i, j);
      if (!tested.insert(static_cast<std::uint64_t>(lo) * pts.size() + hi).second) continue;
      if (ctx.segment_free(pts[lo], pts[hi])) {
        const double w = std::sqrt(nb.distance_squared);
        adj[lo].emplace_back(hi, w);
        adj[hi].emplace_back(lo, w);
        result.roadmap_edges.emplace_back(lo, hi);
      }
    }
  }
  result.iterations = pts.size();
  // Dijkstra on the roadmap from the start milestone.
  std::vector<double> dist(pts.size(), kInf);
  std::vector<std::ptrdiff_t> parent(pts.size(), -1);
  using Item = std::pair<double, std::size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
  dist[0] = 0.0;
  open.emplace(0.0, 0);
  while (!open.empty()) {
    const auto [d, u] = open.top();
    open.pop();
    if (d > dist[u]) continue;
    if (u == 1) break;
    for (const auto& [v, w] : adj[u]) {
      if (d + w < dist[v]) {
        dist[v] = d + w;
        parent[v] = static_cast<std::ptrdiff_t>(u);
        open.emplace(dist[v], v);
      }
    }
  }
  result.tree.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    result.tree.push_back({pts[i], parent[i], dist[i]});
  }
  if (dist[1] < kInf) {
    result.found = true;
    for (std::ptrdiff_t i = 1; i >= 0; i = parent[static_cast<std::size_t>(i)]) {
      result.path.push_back(pts[static_cast<std::size_t>(i)]);
    }
    std::reverse(result.path.begin(), result.path.end());
    result.cost = dist[1];
    result.cost_history.emplace_back(result.iterations, result.cost);
  }
  finalize_stats(ctx, result);
  return result;
}

}  // namespace roboticscore::planning
