#pragma once

/// @file smoothing.hpp
/// @brief Post-processing of geometric paths: resampling, shortcutting and gradient-based
///        smoothing that never introduces collisions.
///
/// Gradient smoothing minimises, over the interior waypoints x_i (endpoints fixed),
///   J = w_data  sum |x_i - y_i|^2                       (stay near the original path y)
///     + w_smooth sum |x_{i-1} - 2 x_i + x_{i+1}|^2       (discrete curvature)
///     + w_obst   sum max(0, margin - c(x_i))^2           (clearance c from the workspace)
/// by gradient descent. A waypoint update is rejected if it would make either adjacent
/// segment collide, so a collision-free input always yields a collision-free output.

#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/environment.hpp"
#include "roboticscore/math/random.hpp"

namespace roboticscore::planning {

/// Polyline resampled at (at most) @p spacing metres, preserving all original vertices.
/// @throws std::invalid_argument if spacing is not positive.
[[nodiscard]] std::vector<Vec2> resample_polyline(const std::vector<Vec2>& path, double spacing);

/// Random shortcutting: repeatedly replaces the section between two random vertices by a
/// straight segment when that segment is collision free. Endpoints are preserved and the
/// path length never increases.
[[nodiscard]] std::vector<Vec2> shortcut_path(const std::vector<Vec2>& path,
                                              const geometry::Environment2D& env,
                                              double robot_radius, int iterations,
                                              math::Random& rng);

/// Gradient smoothing parameters.
struct GradientSmoothingParams {
  double weight_data = 0.1;
  double weight_smooth = 1.0;
  double weight_obstacle = 2.0;
  double obstacle_margin = 1.0;    ///< Clearance below which the obstacle term is active [m].
  double robot_radius = 0.0;       ///< Collision checks inflate obstacles by this radius [m].
  double step = 0.05;              ///< Gradient-descent step size.
  double max_displacement = 0.02;  ///< Per-iteration move limit for one waypoint [m].
  int iterations = 300;
};

/// Gradient-based smoothing (see file comment).
/// @throws std::invalid_argument for invalid parameters.
[[nodiscard]] std::vector<Vec2> gradient_smooth(const std::vector<Vec2>& path,
                                                const geometry::Environment2D& env,
                                                const GradientSmoothingParams& params = {});

/// Sum of squared second differences, a discrete smoothness measure used by tests and
/// benchmarks (lower is smoother).
[[nodiscard]] double polyline_roughness(const std::vector<Vec2>& path);

}  // namespace roboticscore::planning
