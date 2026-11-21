#include "roboticscore/planning/local/dwa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"

namespace roboticscore::planning {
namespace {

void validate(const DwaConfig& c) {
  detail::require(c.max_speed > c.min_speed, "DWA: max_speed must exceed min_speed");
  detail::require(c.max_yaw_rate > 0.0 && c.max_accel > 0.0 && c.max_yaw_accel > 0.0,
                  "DWA: limits must be positive");
  detail::require(c.control_period > 0.0 && c.predict_time > 0.0 && c.rollout_dt > 0.0,
                  "DWA: time parameters must be positive");
  detail::require(c.speed_samples >= 2 && c.yaw_rate_samples >= 2,
                  "DWA: at least two samples per dimension are required");
  detail::require(c.robot_radius >= 0.0 && c.clearance_cap > 0.0,
                  "DWA: robot radius must be >= 0 and clearance cap > 0");
  detail::require(c.curvature_speed_floor > 0.0, "DWA: curvature_speed_floor must be positive");
}

// Clearance at time t from now, with moving obstacles extrapolated at constant velocity.
double clearance_at(const Vec2& p, const geometry::Environment2D& env,
                    std::span<const MovingObstacle> dynamic, double t = 0.0) {
  double c = env.clearance(p);
  for (const auto& o : dynamic) {
    c = std::min(c, (p - (o.shape.center + t * o.velocity)).norm() - o.shape.radius);
  }
  return c;
}

// Free distance along the arc of curvature `kappa` from `pose` in the driving direction,
// capped at `cap` (Fox et al.'s dist(v, omega)).
double arc_free_distance(const geometry::Pose2& pose, double kappa, double direction, double cap,
                         double radius, const geometry::Environment2D& env,
                         std::span<const MovingObstacle> dynamic) {
  constexpr double kStep = 0.05;
  const int steps = static_cast<int>(std::floor(cap / kStep + 1e-9));
  for (int i = 1; i <= steps; ++i) {
    const double d = direction * kStep * i;
    const geometry::Pose2 p = models::Unicycle::step(pose, {d, d * kappa}, 1.0);
    if (clearance_at(p.translation(), env, dynamic) - radius <= 0.0) {
      return kStep * (i - 1);
    }
  }
  return cap;
}

}  // namespace

DwaResult dwa_plan(const geometry::Pose2& pose, const models::UnicycleInput& velocity,
                   const Vec2& goal, const geometry::Environment2D& env,
                   std::span<const MovingObstacle> dynamic_obstacles, const DwaConfig& c) {
  validate(c);
  // Dynamic window intersected with the velocity limits.
  const double v_lo = std::max(c.min_speed, velocity.v - c.max_accel * c.control_period);
  const double v_hi = std::min(c.max_speed, velocity.v + c.max_accel * c.control_period);
  const double w_lo =
      std::max(-c.max_yaw_rate, velocity.omega - c.max_yaw_accel * c.control_period);
  const double w_hi = std::min(c.max_yaw_rate, velocity.omega + c.max_yaw_accel * c.control_period);

  DwaResult result;
  const int steps = std::max(1, static_cast<int>(std::ceil(c.predict_time / c.rollout_dt)));
  for (int i = 0; i < c.speed_samples; ++i) {
    const double v = v_lo + (v_hi - v_lo) * i / (c.speed_samples - 1);
    for (int j = 0; j < c.yaw_rate_samples; ++j) {
      const double w = w_lo + (w_hi - w_lo) * j / (c.yaw_rate_samples - 1);
      DwaCandidate cand;
      cand.command = {v, w};
      cand.rollout.reserve(static_cast<std::size_t>(steps) + 1);
      cand.rollout.push_back(pose);
      double min_clear = clearance_at(pose.translation(), env, dynamic_obstacles) - c.robot_radius;
      geometry::Pose2 p = pose;
      for (int k = 0; k < steps; ++k) {
        p = models::Unicycle::step(p, cand.command, c.rollout_dt);
        cand.rollout.push_back(p);
        min_clear = std::min(
            min_clear, clearance_at(p.translation(), env, dynamic_obstacles, (k + 1) * c.rollout_dt)
                           - c.robot_radius);
      }
      cand.clearance = min_clear;
      // Curvature of the commanded arc; a stationary command looks straight ahead.
      const double kappa = w / std::max(std::abs(v), c.curvature_speed_floor);
      cand.free_distance = arc_free_distance(pose, kappa, v < 0.0 ? -1.0 : 1.0, c.clearance_cap,
                                             c.robot_radius, env, dynamic_obstacles);
      // Admissible: no contact during the rollout and able to brake on the free arc.
      const double brake = v * v / (2.0 * c.max_accel);
      cand.admissible = min_clear > 0.0 && brake <= cand.free_distance + 1e-12;
      const Vec2 to_goal = goal - p.translation();
      cand.heading =
          to_goal.norm() < 1e-9
              ? 0.0
              : std::abs(math::angle_difference(std::atan2(to_goal.y(), to_goal.x()), p.theta()));
      result.candidates.push_back(std::move(cand));
    }
  }

  double best = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < result.candidates.size(); ++i) {
    auto& cand = result.candidates[i];
    cand.score = c.heading_weight * (1.0 - cand.heading / math::kPi)
                 + c.clearance_weight * cand.free_distance / c.clearance_cap
                 + c.velocity_weight * cand.command.v / c.max_speed;
    if (cand.admissible && cand.score > best) {
      best = cand.score;
      result.best = i;
      result.valid = true;
    }
  }
  if (result.valid) {
    result.command = result.candidates[result.best].command;
  } else if (c.escape_when_blocked) {
    // Nothing is admissible (e.g. an obstacle is approaching the robot): rather than freezing
    // in place, execute the rollout that keeps the largest worst-case clearance.
    double best_clear = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
      if (result.candidates[i].clearance > best_clear) {
        best_clear = result.candidates[i].clearance;
        result.best = i;
      }
    }
    result.command = result.candidates[result.best].command;
  }
  return result;
}

}  // namespace roboticscore::planning
