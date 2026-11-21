#include "roboticscore/planning/local/velocity_obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include "roboticscore/core/error.hpp"

namespace roboticscore::planning {

double time_to_collision(const Vec2& rel_pos, const Vec2& rel_vel, double radius) {
  // Solve |rel_pos - rel_vel t| = radius for the smallest t >= 0 (agent closes in at rel_vel).
  const double c = rel_pos.squaredNorm() - radius * radius;
  if (c <= 0.0) {
    return 0.0;
  }
  const double a = rel_vel.squaredNorm();
  const double b = -2.0 * rel_pos.dot(rel_vel);
  if (a <= 1e-15) {
    return std::numeric_limits<double>::infinity();
  }
  const double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double t = (-b - std::sqrt(disc)) / (2.0 * a);
  return t >= 0.0 ? t : std::numeric_limits<double>::infinity();
}

VelocitySelection select_velocity(const Vec2& position, const Vec2& velocity, double radius,
                                  const Vec2& preferred_velocity,
                                  std::span<const VoNeighbor> neighbors,
                                  const VelocityObstacleConfig& c) {
  detail::require(c.max_speed > 0.0 && c.time_horizon > 0.0,
                  "select_velocity: max_speed and time_horizon must be positive");
  detail::require(c.speed_samples >= 1 && c.heading_samples >= 4,
                  "select_velocity: too few samples");
  detail::require(radius >= 0.0 && c.safety_margin >= 0.0 && c.penalty_weight >= 0.0,
                  "select_velocity: radius, margin and weight must be non-negative");

  const auto earliest_collision = [&](const Vec2& v) {
    double ttc = std::numeric_limits<double>::infinity();
    for (const VoNeighbor& nb : neighbors) {
      // RVO tests 2v - v_A against the VO; plain VO tests v itself.
      const Vec2 effective = nb.reciprocal ? Vec2(2.0 * v - velocity) : v;
      ttc = std::min(ttc, time_to_collision(nb.position - position, effective - nb.velocity,
                                            radius + nb.radius + c.safety_margin));
    }
    return ttc;
  };

  Vec2 pref = preferred_velocity;
  if (pref.norm() > c.max_speed) {
    pref *= c.max_speed / pref.norm();
  }
  // Unobstructed: keep the preferred velocity exactly.
  if (const double ttc = earliest_collision(pref); ttc > c.time_horizon) {
    return {pref, true, ttc};
  }
  // In conflict: aim slightly to the side to break symmetric deadlocks.
  const double cb = std::cos(c.side_bias);
  const double sb = std::sin(c.side_bias);
  pref = Vec2(cb * pref.x() - sb * pref.y(), sb * pref.x() + cb * pref.y());

  // Candidate set: the biased preferred velocity, zero, and a polar grid.
  std::vector<Vec2> candidates;
  candidates.push_back(pref);
  candidates.emplace_back(Vec2::Zero());
  for (int i = 1; i <= c.speed_samples; ++i) {
    const double s = c.max_speed * i / c.speed_samples;
    for (int j = 0; j < c.heading_samples; ++j) {
      const double a = 2.0 * std::numbers::pi * j / c.heading_samples;
      candidates.emplace_back(s * std::cos(a), s * std::sin(a));
    }
  }

  VelocitySelection best;
  double best_free_dist = std::numeric_limits<double>::infinity();
  double best_penalty = std::numeric_limits<double>::infinity();
  VelocitySelection best_penalised;
  for (const Vec2& v : candidates) {
    const double ttc = earliest_collision(v);
    const double dist = (v - pref).norm();
    if (ttc > c.time_horizon) {
      if (dist < best_free_dist) {
        best_free_dist = dist;
        best = {v, true, ttc};
      }
    } else {
      const double penalty = c.penalty_weight / std::max(ttc, 1e-6) + dist;
      if (penalty < best_penalty) {
        best_penalty = penalty;
        best_penalised = {v, false, ttc};
      }
    }
  }
  return best.collision_free ? best : best_penalised;
}

}  // namespace roboticscore::planning
