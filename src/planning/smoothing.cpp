#include "roboticscore/planning/smoothing.hpp"

#include <algorithm>
#include <cmath>

#include "roboticscore/core/error.hpp"

namespace roboticscore::planning {
namespace {

Vec2 clearance_gradient(const geometry::Environment2D& env, const Vec2& p) {
  constexpr double h = 1e-4;
  return {(env.clearance(p + Vec2(h, 0)) - env.clearance(p - Vec2(h, 0))) / (2 * h),
          (env.clearance(p + Vec2(0, h)) - env.clearance(p - Vec2(0, h))) / (2 * h)};
}

}  // namespace

std::vector<Vec2> resample_polyline(const std::vector<Vec2>& path, double spacing) {
  detail::require(spacing > 0.0, "resample_polyline: spacing must be positive");
  if (path.size() < 2) {
    return path;
  }
  std::vector<Vec2> out{path.front()};
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Vec2 d = path[i] - path[i - 1];
    const int n = std::max(1, static_cast<int>(std::ceil(d.norm() / spacing)));
    for (int k = 1; k <= n; ++k) {
      out.push_back(path[i - 1] + d * (static_cast<double>(k) / n));
    }
  }
  return out;
}

std::vector<Vec2> shortcut_path(const std::vector<Vec2>& path, const geometry::Environment2D& env,
                                double robot_radius, int iterations, math::Random& rng) {
  std::vector<Vec2> out = path;
  for (int it = 0; it < iterations && out.size() > 2; ++it) {
    std::size_t i = rng.uniform_index(out.size());
    std::size_t j = rng.uniform_index(out.size());
    if (i > j) std::swap(i, j);
    if (j < i + 2) continue;
    if (env.segment_free(out[i], out[j], robot_radius)) {
      out.erase(out.begin() + static_cast<std::ptrdiff_t>(i) + 1,
                out.begin() + static_cast<std::ptrdiff_t>(j));
    }
  }
  return out;
}

std::vector<Vec2> gradient_smooth(const std::vector<Vec2>& path, const geometry::Environment2D& env,
                                  const GradientSmoothingParams& p) {
  detail::require(p.weight_data >= 0.0 && p.weight_smooth >= 0.0 && p.weight_obstacle >= 0.0,
                  "gradient_smooth: weights must be non-negative");
  detail::require(p.step > 0.0 && p.iterations >= 0 && p.max_displacement > 0.0,
                  "gradient_smooth: invalid step/iterations/displacement");
  detail::require(p.obstacle_margin >= 0.0 && p.robot_radius >= 0.0,
                  "gradient_smooth: margin and radius must be non-negative");
  std::vector<Vec2> x = path;
  const std::size_t n = x.size();
  if (n < 3) {
    return x;
  }
  std::vector<Vec2> second(n, Vec2::Zero());
  for (int it = 0; it < p.iterations; ++it) {
    for (std::size_t i = 1; i + 1 < n; ++i) {
      second[i] = x[i - 1] - 2.0 * x[i] + x[i + 1];
    }
    for (std::size_t i = 1; i + 1 < n; ++i) {
      Vec2 grad = 2.0 * p.weight_data * (x[i] - path[i]);
      // d/dx_i of sum_j |D_j|^2 with D_j = x_{j-1} - 2 x_j + x_{j+1}.
      Vec2 smooth = -4.0 * second[i];
      if (i >= 2) smooth += 2.0 * second[i - 1];
      if (i + 2 < n) smooth += 2.0 * second[i + 1];
      grad += p.weight_smooth * smooth;
      const double c = env.clearance(x[i]) - p.robot_radius;
      if (c < p.obstacle_margin) {
        grad += -2.0 * p.weight_obstacle * (p.obstacle_margin - c) * clearance_gradient(env, x[i]);
      }
      Vec2 delta = -p.step * grad;
      if (delta.norm() > p.max_displacement) {
        delta *= p.max_displacement / delta.norm();  // keeps the descent stable near obstacles
      }
      const Vec2 candidate = x[i] + delta;
      if (env.segment_free(x[i - 1], candidate, p.robot_radius)
          && env.segment_free(candidate, x[i + 1], p.robot_radius)) {
        x[i] = candidate;
      }
    }
  }
  return x;
}

double polyline_roughness(const std::vector<Vec2>& path) {
  double r = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    r += (path[i - 1] - 2.0 * path[i] + path[i + 1]).squaredNorm();
  }
  return r;
}

}  // namespace roboticscore::planning
