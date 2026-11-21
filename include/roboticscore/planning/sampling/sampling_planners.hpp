#pragma once

/// @file sampling_planners.hpp
/// @brief Sampling-based motion planners in a continuous 2D workspace.
///
///   PRM              Kavraki et al., IEEE T-RA 1996 (multi-query roadmap; here k-nearest PRM)
///   RRT              LaValle, TR 98-11, 1998 (single-query tree, probabilistically complete)
///   RRT-Connect      Kuffner & LaValle, ICRA 2000 (bidirectional, greedy CONNECT heuristic)
///   RRT*             Karaman & Frazzoli, IJRR 2011 (asymptotically optimal: choose-parent and
///                    rewiring within a shrinking radius)
///   Informed RRT*    Gammell, Srinivasa & Barfoot, IROS 2014 (after the first solution,
///                    samples only the ellipse of states that can improve it)
///
/// All planners treat the robot as a disc of radius robot_radius, use the exact segment
/// collision checker of geometry::Environment2D and a k-d tree for neighbour queries, and are
/// deterministic for a fixed seed.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/environment.hpp"

namespace roboticscore::planning {

/// Parameters shared by the sampling-based planners.
struct SamplingConfig {
  double step_size = 1.0;       ///< Maximum edge length produced by steer() [m].
  double goal_bias = 0.05;      ///< Probability of sampling the goal (RRT family).
  double goal_tolerance = 0.5;  ///< A node this close to the goal may connect to it [m].
  int max_iterations = 5000;    ///< Sampling iterations.
  double robot_radius = 0.0;    ///< Collision checks inflate obstacles by this radius [m].
  std::uint64_t seed = 1;
  /// RRT*: stop at the first solution instead of continuing to optimise.
  bool stop_at_first_solution = false;
  /// RRT*: multiplier on the theoretical lower bound gamma* of the rewiring radius
  /// (values > 1 keep asymptotic optimality; Karaman & Frazzoli 2011, Thm. 38).
  double rewire_factor = 1.1;
  /// PRM: number of milestones and neighbours per milestone.
  int prm_samples = 500;
  int prm_neighbors = 10;
};

/// A tree or roadmap vertex.
struct TreeNode {
  Vec2 point = Vec2::Zero();
  std::ptrdiff_t parent = -1;  ///< -1 for roots.
  double cost = 0.0;           ///< Cost-to-come along the tree [m].
};

/// Planner output and statistics.
struct SamplingResult {
  bool found = false;
  std::vector<Vec2> path;  ///< Start to goal.
  double cost = 0.0;       ///< Path length [m].
  std::size_t iterations = 0;
  std::size_t samples = 0;           ///< Random states drawn.
  std::size_t collision_checks = 0;  ///< Segment/point collision queries.
  std::size_t first_solution_iteration = 0;
  std::vector<TreeNode> tree;       ///< Start tree (RRT family) or roadmap vertices (PRM).
  std::vector<TreeNode> goal_tree;  ///< RRT-Connect only: tree rooted at the goal.
  std::vector<std::pair<std::size_t, std::size_t>> roadmap_edges;  ///< PRM only.
  /// (iteration, best cost) every time the solution improves.
  std::vector<std::pair<std::size_t, double>> cost_history;
};

/// Planner events reported to an observer (for visualisation and tests).
struct SamplingEvent {
  enum class Type { kNodeAdded, kRewired, kSolutionImproved };
  Type type = Type::kNodeAdded;
  std::size_t iteration = 0;
  std::size_t node = 0;            ///< Node added or rewired.
  std::ptrdiff_t parent = -1;      ///< New parent.
  std::ptrdiff_t old_parent = -1;  ///< Previous parent (kRewired).
  double cost = 0.0;               ///< Node cost, or best solution cost.
  int tree = 0;                    ///< 0 = start tree, 1 = goal tree (RRT-Connect).
};
using SamplingObserver = std::function<void(const SamplingEvent&)>;

/// @throws std::invalid_argument for invalid configuration values.
[[nodiscard]] SamplingResult rrt(const geometry::Environment2D& env, const Vec2& start,
                                 const Vec2& goal, const SamplingConfig& config,
                                 const SamplingObserver& observer = {});

[[nodiscard]] SamplingResult rrt_connect(const geometry::Environment2D& env, const Vec2& start,
                                         const Vec2& goal, const SamplingConfig& config,
                                         const SamplingObserver& observer = {});

[[nodiscard]] SamplingResult rrt_star(const geometry::Environment2D& env, const Vec2& start,
                                      const Vec2& goal, const SamplingConfig& config,
                                      const SamplingObserver& observer = {});

[[nodiscard]] SamplingResult informed_rrt_star(const geometry::Environment2D& env,
                                               const Vec2& start, const Vec2& goal,
                                               const SamplingConfig& config,
                                               const SamplingObserver& observer = {});

[[nodiscard]] SamplingResult prm(const geometry::Environment2D& env, const Vec2& start,
                                 const Vec2& goal, const SamplingConfig& config);

/// Uniform sample from the ellipse {x : |x - a| + |x - b| <= c_best} (c_best > |b - a|).
/// Exposed for testing the informed sampling region.
[[nodiscard]] Vec2 sample_informed_ellipse(const Vec2& a, const Vec2& b, double c_best,
                                           double u_radius, double u_angle);

/// Total length of a polyline.
[[nodiscard]] double polyline_length(const std::vector<Vec2>& path);

}  // namespace roboticscore::planning
