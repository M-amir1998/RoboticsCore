#include "roboticscore/planning/local/potential_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "roboticscore/core/error.hpp"

namespace roboticscore::planning {
namespace {

Vec2 attractive_force(const Vec2& p, const Vec2& goal, const PotentialFieldConfig& c) {
  const Vec2 d = goal - p;
  const double dist = d.norm();
  if (dist <= c.conic_threshold) {
    return c.attractive_gain * d;
  }
  return c.attractive_gain * c.conic_threshold * d / dist;
}

}  // namespace

double potential(const Vec2& p, const Vec2& goal, const geometry::Environment2D& env,
                 const PotentialFieldConfig& c) {
  const double dist = (goal - p).norm();
  const double u_att = dist <= c.conic_threshold
                           ? 0.5 * c.attractive_gain * dist * dist
                           : c.attractive_gain * c.conic_threshold * dist
                                 - 0.5 * c.attractive_gain * c.conic_threshold * c.conic_threshold;
  const double rho = env.clearance(p);
  double u_rep = 0.0;
  if (rho <= 0.0) {
    u_rep = std::numeric_limits<double>::infinity();
  } else if (rho <= c.influence_distance) {
    const double k = 1.0 / rho - 1.0 / c.influence_distance;
    u_rep = 0.5 * c.repulsive_gain * k * k;
  }
  return u_att + u_rep;
}

Vec2 potential_force(const Vec2& p, const Vec2& goal, const geometry::Environment2D& env,
                     const PotentialFieldConfig& c) {
  Vec2 f = attractive_force(p, goal, c);
  const double rho = env.clearance(p);
  if (rho > 0.0 && rho <= c.influence_distance) {
    constexpr double h = 1e-5;
    const Vec2 grad_rho((env.clearance(p + Vec2(h, 0)) - env.clearance(p - Vec2(h, 0))) / (2 * h),
                        (env.clearance(p + Vec2(0, h)) - env.clearance(p - Vec2(0, h))) / (2 * h));
    // -grad U_rep = k_rep (1/rho - 1/rho_0) (1/rho^2) grad rho
    f += c.repulsive_gain * (1.0 / rho - 1.0 / c.influence_distance) / (rho * rho) * grad_rho;
  }
  return f;
}

PotentialFieldResult potential_field_plan(const geometry::Environment2D& env, const Vec2& start,
                                          const Vec2& goal, const PotentialFieldConfig& c) {
  detail::require(c.attractive_gain > 0.0 && c.repulsive_gain >= 0.0,
                  "potential_field_plan: gains must be positive");
  detail::require(c.influence_distance > 0.0 && c.conic_threshold > 0.0,
                  "potential_field_plan: distances must be positive");
  detail::require(c.step_size > 0.0 && c.max_iterations > 0 && c.stall_window > 0,
                  "potential_field_plan: invalid step or iteration settings");
  PotentialFieldResult result;
  result.path.push_back(start);
  Vec2 p = start;
  for (int it = 1; it <= c.max_iterations; ++it) {
    result.iterations = static_cast<std::size_t>(it);
    if ((goal - p).norm() <= c.goal_tolerance) {
      result.status = PotentialFieldStatus::kReachedGoal;
      return result;
    }
    const Vec2 f = potential_force(p, goal, env, c);
    const double fn = f.norm();
    if (fn < 1e-9) {
      result.status = PotentialFieldStatus::kLocalMinimum;  // exact equilibrium
      return result;
    }
    p += c.step_size * f / fn;
    if (env.clearance(p) <= 0.0) {
      result.status = PotentialFieldStatus::kCollision;
      result.path.push_back(p);
      return result;
    }
    result.path.push_back(p);
    // Oscillation around an equilibrium: little net progress over the stall window.
    if (it >= c.stall_window) {
      const Vec2& past =
          result.path[result.path.size() - 1 - static_cast<std::size_t>(c.stall_window)];
      if ((p - past).norm() < c.stall_distance) {
        result.status = PotentialFieldStatus::kLocalMinimum;
        return result;
      }
    }
  }
  result.status = PotentialFieldStatus::kMaxIterations;
  return result;
}

}  // namespace roboticscore::planning
