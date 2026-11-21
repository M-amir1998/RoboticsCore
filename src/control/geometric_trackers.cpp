#include "roboticscore/control/geometric_trackers.hpp"

#include <algorithm>
#include <cmath>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"

namespace roboticscore::control {

PurePursuitTracker::PurePursuitTracker(const models::BicycleParams& vehicle,
                                       const PurePursuitConfig& config)
    : PathTracker(vehicle, config.speed_pid), config_(config) {
  detail::require(config.lookahead_min > 0.0 && config.lookahead_max >= config.lookahead_min,
                  "PurePursuitTracker: invalid lookahead bounds");
  detail::require(config.lookahead_gain >= 0.0, "PurePursuitTracker: gain must be >= 0");
}

VehicleCommand PurePursuitTracker::compute(const models::BicycleState& state,
                                           const trajectory::ReferencePath& path, double dt) {
  detail::require(dt > 0.0, "PurePursuitTracker::compute: dt must be positive");
  const Vec2 rear(state(0), state(1));
  const double yaw = state(2);
  const double v = state(3);
  const auto proj = track_point(rear, yaw, path);
  lookahead_ = std::clamp(config_.lookahead_gain * std::abs(v) + config_.lookahead_offset,
                          config_.lookahead_min, config_.lookahead_max);
  status_.target = path.at(proj.s + lookahead_).position;
  const Vec2 d = status_.target - rear;
  const double ld = std::max(d.norm(), 1e-6);
  const double alpha = math::angle_difference(std::atan2(d.y(), d.x()), yaw);
  VehicleCommand cmd;
  cmd.steering = clamp_steering(std::atan2(2.0 * vehicle_.wheelbase * std::sin(alpha), ld));
  cmd.acceleration = speed_control(v, dt);
  return cmd;
}

StanleyTracker::StanleyTracker(const models::BicycleParams& vehicle, const StanleyConfig& config)
    : PathTracker(vehicle, config.speed_pid), config_(config) {
  detail::require(config.gain >= 0.0 && config.softening > 0.0,
                  "StanleyTracker: gain must be >= 0 and softening > 0");
}

VehicleCommand StanleyTracker::compute(const models::BicycleState& state,
                                       const trajectory::ReferencePath& path, double dt) {
  detail::require(dt > 0.0, "StanleyTracker::compute: dt must be positive");
  const double yaw = state(2);
  const double v = state(3);
  const Vec2 front =
      Vec2(state(0), state(1)) + vehicle_.wheelbase * Vec2(std::cos(yaw), std::sin(yaw));
  const auto proj = track_point(front, yaw, path);
  status_.target = proj.point;
  const double heading_term = -status_.heading_error;  // path heading - yaw
  const double cross_term =
      std::atan2(-config_.gain * proj.cross_track, config_.softening + std::abs(v));
  double delta = heading_term + cross_term;
  if (config_.curvature_feedforward) {
    delta += std::atan(vehicle_.wheelbase * proj.curvature);
  }
  VehicleCommand cmd;
  cmd.steering = clamp_steering(math::wrap_to_pi(delta));
  cmd.acceleration = speed_control(v, dt);
  return cmd;
}

}  // namespace roboticscore::control
