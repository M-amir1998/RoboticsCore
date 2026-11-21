#include "roboticscore/planning/kinodynamic/hybrid_astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <queue>

#include "roboticscore/core/error.hpp"
#include "roboticscore/geometry/grid.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/models/unicycle.hpp"

namespace roboticscore::planning {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

struct SearchNode {
  geometry::Pose2 pose;
  double g = 0.0;
  std::ptrdiff_t parent = -1;
  bool forward = true;
  double steer = 0.0;
};

struct OpenItem {
  double f;
  std::uint64_t order;
  std::size_t node;
};

struct LowerPriority {
  bool operator()(const OpenItem& a, const OpenItem& b) const {
    return a.f != b.f ? a.f > b.f : a.order > b.order;
  }
};

// Obstacle-aware holonomic distance-to-goal on a 2D grid (8-connected Dijkstra).
class HolonomicHeuristic {
 public:
  HolonomicHeuristic(const geometry::Environment2D& env, double resolution, double inflation,
                     const Vec2& goal)
      : spec_(std::max(1, static_cast<int>(std::ceil(env.bounds().size().x() / resolution))),
              std::max(1, static_cast<int>(std::ceil(env.bounds().size().y() / resolution))),
              resolution, env.bounds().min),
        dist_(spec_.cell_count(), kInf) {
    std::vector<std::uint8_t> blocked(spec_.cell_count(), 0);
    for (int y = 0; y < spec_.height(); ++y) {
      for (int x = 0; x < spec_.width(); ++x) {
        const geometry::GridIndex c{x, y};
        if (env.clearance(spec_.cell_center(c)) <= inflation) {
          blocked[spec_.linear_index(c)] = 1;
        }
      }
    }
    const auto g = spec_.world_to_cell_checked(goal);
    if (!g) {
      return;
    }
    blocked[spec_.linear_index(*g)] = 0;
    using Item = std::pair<double, std::size_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
    dist_[spec_.linear_index(*g)] = 0.0;
    open.emplace(0.0, spec_.linear_index(*g));
    constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    while (!open.empty()) {
      const auto [d, u] = open.top();
      open.pop();
      if (d > dist_[u]) continue;
      const geometry::GridIndex c = spec_.from_linear(u);
      for (int k = 0; k < 8; ++k) {
        const geometry::GridIndex n{c.x + kDx[k], c.y + kDy[k]};
        if (!spec_.in_bounds(n) || blocked[spec_.linear_index(n)] != 0) continue;
        const double step = (k < 4 ? 1.0 : std::numbers::sqrt2) * resolution;
        const std::size_t ni = spec_.linear_index(n);
        if (d + step < dist_[ni]) {
          dist_[ni] = d + step;
          open.emplace(dist_[ni], ni);
        }
      }
    }
  }

  [[nodiscard]] double operator()(const Vec2& p) const {
    const auto c = spec_.world_to_cell_checked(p);
    return c ? dist_[spec_.linear_index(*c)] : kInf;
  }

 private:
  geometry::GridSpec spec_;
  std::vector<double> dist_;
};

}  // namespace

HybridAStar::HybridAStar(const geometry::Environment2D& env, const models::BicycleParams& vehicle,
                         const models::VehicleFootprint& footprint, HybridAStarConfig config)
    : env_(env), vehicle_(vehicle), footprint_(footprint), config_(config) {
  detail::require(config.xy_resolution > 0.0 && config.yaw_resolution > 0.0,
                  "HybridAStar: resolutions must be positive");
  detail::require(config.step_length > config.xy_resolution,
                  "HybridAStar: step_length must exceed xy_resolution to leave the cell");
  detail::require(config.steering_samples >= 3 && config.steering_samples % 2 == 1,
                  "HybridAStar: steering_samples must be an odd number >= 3");
  detail::require(config.reverse_penalty >= 1.0 && config.gear_switch_penalty >= 0.0
                      && config.steer_penalty >= 0.0 && config.steer_change_penalty >= 0.0,
                  "HybridAStar: penalties must keep the cost >= driven length");
  detail::require(config.collision_step > 0.0, "HybridAStar: collision_step must be positive");
  detail::require(config.analytic_expansion_interval > 0,
                  "HybridAStar: analytic_expansion_interval must be positive");
}

bool HybridAStar::pose_free(const geometry::Pose2& pose) const {
  // Fast acceptance with the bounding disc, exact polygon test otherwise.
  if (env_.clearance(footprint_.center(pose)) > footprint_.bounding_radius()) {
    return true;
  }
  return env_.polygon_free(footprint_.polygon(pose));
}

HybridAStarResult HybridAStar::plan(const geometry::Pose2& start,
                                    const geometry::Pose2& goal) const {
  HybridAStarResult result;
  if (!pose_free(start) || !pose_free(goal)) {
    return result;
  }
  const double radius = vehicle_.min_turning_radius();
  const auto& bounds = env_.bounds();
  const int nx = static_cast<int>(std::ceil(bounds.size().x() / config_.xy_resolution));
  const int ny = static_cast<int>(std::ceil(bounds.size().y() / config_.xy_resolution));
  const int nyaw = static_cast<int>(std::ceil(math::kTwoPi / config_.yaw_resolution));
  const auto cell_of = [&](const geometry::Pose2& p) -> std::size_t {
    const int ix = std::clamp(
        static_cast<int>(std::floor((p.x() - bounds.min.x()) / config_.xy_resolution)), 0, nx - 1);
    const int iy = std::clamp(
        static_cast<int>(std::floor((p.y() - bounds.min.y()) / config_.xy_resolution)), 0, ny - 1);
    const int iyaw =
        static_cast<int>(std::floor(math::wrap_to_2pi(p.theta()) / config_.yaw_resolution)) % nyaw;
    return (static_cast<std::size_t>(iyaw) * static_cast<std::size_t>(ny)
            + static_cast<std::size_t>(iy))
               * static_cast<std::size_t>(nx)
           + static_cast<std::size_t>(ix);
  };

  // A disc of radius min(width/2, rear overhang) around the rear axle is always inside the
  // body, so blocking cells closer than that to obstacles never removes feasible states.
  const double inflation = 0.95 * std::min(0.5 * footprint_.width, footprint_.rear_overhang);
  const HolonomicHeuristic holonomic(env_, config_.xy_resolution, inflation, goal.translation());
  const auto heuristic = [&](const geometry::Pose2& p) {
    return std::max(reeds_shepp_shortest(p, goal, radius).length(), holonomic(p.translation()));
  };

  std::vector<double> steers;
  steers.reserve(static_cast<std::size_t>(config_.steering_samples));
  const double max_steer = vehicle_.params().max_steer;
  for (int i = 0; i < config_.steering_samples; ++i) {
    steers.push_back(-max_steer + 2.0 * max_steer * i / (config_.steering_samples - 1));
  }
  const int sub_steps =
      std::max(1, static_cast<int>(std::ceil(config_.step_length / config_.collision_step)));
  // Simulates a primitive; returns false on collision. Fills `poses` (excluding the start).
  const auto simulate = [&](const geometry::Pose2& from, bool forward, double steer,
                            std::vector<DirectedPose>* poses) {
    const double kappa = vehicle_.curvature(steer);
    const double ds = (forward ? 1.0 : -1.0) * config_.step_length / sub_steps;
    geometry::Pose2 p = from;
    for (int k = 0; k < sub_steps; ++k) {
      p = models::Unicycle::step(p, {ds, ds * kappa}, 1.0);
      if (!pose_free(p)) {
        return false;
      }
      if (poses != nullptr) {
        poses->push_back({p, forward, kappa});
      }
    }
    return true;
  };

  std::vector<SearchNode> nodes;
  std::vector<double> best_g(
      static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nyaw),
      kInf);
  std::vector<std::uint8_t> closed(best_g.size(), 0);
  std::priority_queue<OpenItem, std::vector<OpenItem>, LowerPriority> open;
  std::uint64_t order = 0;

  const double h0 = heuristic(start);
  if (!std::isfinite(h0)) {
    return result;
  }
  nodes.push_back({start, 0.0, -1, true, 0.0});
  best_g[cell_of(start)] = 0.0;
  open.push({h0, order++, 0});

  std::ptrdiff_t goal_node = -1;
  std::vector<DirectedPose> final_shot;
  while (!open.empty() && result.expanded < config_.max_expansions) {
    const OpenItem item = open.top();
    open.pop();
    const SearchNode current = nodes[item.node];
    const std::size_t cell = cell_of(current.pose);
    if (closed[cell] != 0) continue;
    closed[cell] = 1;
    ++result.expanded;
    result.expanded_poses.push_back(current.pose);

    // Analytic expansion: a collision-free Reeds-Shepp shot completes the search.
    const ReedsSheppPath shot = reeds_shepp_shortest(current.pose, goal, radius);
    if (result.expanded % static_cast<std::size_t>(config_.analytic_expansion_interval) == 0
        || shot.length() < 3.0 * config_.step_length) {
      auto samples = sample_reeds_shepp(current.pose, shot, config_.collision_step);
      const bool free = std::all_of(samples.begin(), samples.end(),
                                    [&](const DirectedPose& d) { return pose_free(d.pose); });
      if (free) {
        goal_node = static_cast<std::ptrdiff_t>(item.node);
        final_shot = std::move(samples);
        result.analytic_expansion = true;
        break;
      }
    }

    for (const bool forward : {true, false}) {
      if (!forward && !config_.allow_reverse) continue;
      for (const double steer : steers) {
        std::vector<DirectedPose> poses;
        if (!simulate(current.pose, forward, steer, &poses)) continue;
        const geometry::Pose2& end = poses.back().pose;
        const std::size_t nc = cell_of(end);
        if (closed[nc] != 0) continue;
        const double l = config_.step_length;
        double cost = l * (forward ? 1.0 : config_.reverse_penalty)
                      + config_.steer_penalty * std::abs(steer) * l
                      + config_.steer_change_penalty * std::abs(steer - current.steer);
        if (current.parent >= 0 && forward != current.forward) {
          cost += config_.gear_switch_penalty;
        }
        const double g = current.g + cost;
        if (g >= best_g[nc]) continue;
        const double h = heuristic(end);
        if (!std::isfinite(h)) continue;
        best_g[nc] = g;
        nodes.push_back({end, g, static_cast<std::ptrdiff_t>(item.node), forward, steer});
        open.push({g + h, order++, nodes.size() - 1});
      }
    }
  }
  if (goal_node < 0) {
    return result;
  }

  // Reconstruct by re-simulating each primitive from its parent (deterministic).
  std::vector<std::size_t> chain;
  for (std::ptrdiff_t n = goal_node; n >= 0; n = nodes[static_cast<std::size_t>(n)].parent) {
    chain.push_back(static_cast<std::size_t>(n));
  }
  std::reverse(chain.begin(), chain.end());
  result.path.push_back({start, chain.size() > 1 ? nodes[chain[1]].forward : true, 0.0});
  for (std::size_t i = 1; i < chain.size(); ++i) {
    const SearchNode& n = nodes[chain[i]];
    (void)simulate(nodes[chain[i - 1]].pose, n.forward, n.steer, &result.path);
  }
  result.cost = nodes[static_cast<std::size_t>(goal_node)].g;
  for (std::size_t i = 1; i < final_shot.size(); ++i) {
    result.path.push_back(final_shot[i]);
  }
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    const double d =
        (result.path[i].pose.translation() - result.path[i - 1].pose.translation()).norm();
    result.length += d;
    if (result.path[i].forward != result.path[i - 1].forward) {
      ++result.gear_switches;
    }
  }
  // Account for the Reeds-Shepp segment in the search cost with the same weights.
  for (std::size_t i = 1; i < final_shot.size(); ++i) {
    const double d =
        (final_shot[i].pose.translation() - final_shot[i - 1].pose.translation()).norm();
    result.cost += d * (final_shot[i].forward ? 1.0 : config_.reverse_penalty);
  }
  result.found = true;
  return result;
}

}  // namespace roboticscore::planning
