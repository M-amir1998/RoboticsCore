#include "roboticscore/control/path_tracking.hpp"

#include <algorithm>
#include <cmath>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"

namespace roboticscore::control {

PathTracker::PathTracker(const models::BicycleParams& vehicle, const PidConfig& speed_pid)
    : vehicle_(vehicle), speed_pid_(speed_pid) {
  detail::require(vehicle.wheelbase > 0.0 && vehicle.max_steer > 0.0,
                  "PathTracker: wheelbase and steering limit must be positive");
}

void PathTracker::reset() {
  speed_pid_.reset();
  status_ = {};
  hint_ = -1;
}

trajectory::PathProjection PathTracker::track_point(const Vec2& point, double yaw,
                                                    const trajectory::ReferencePath& path) {
  const trajectory::PathProjection proj = path.project(point, hint_);
  hint_ = static_cast<std::ptrdiff_t>(proj.index);
  status_.cross_track_error = proj.cross_track;
  status_.heading_error = math::angle_difference(yaw, proj.heading);
  status_.path_s = proj.s;
  // Preview the reference speed slightly ahead: rest-to-rest profiles are exactly zero at the
  // start, which would otherwise keep the vehicle from ever moving.
  status_.reference_speed = path.at(proj.s + kSpeedPreview).speed;
  status_.finished = proj.s >= path.length() - kFinishTolerance;
  status_.prediction.clear();
  return proj;
}

double PathTracker::speed_control(double v, double dt) {
  return speed_pid_.update(status_.reference_speed, v, dt);
}

double PathTracker::clamp_steering(double delta) const {
  return std::clamp(delta, -vehicle_.max_steer, vehicle_.max_steer);
}

}  // namespace roboticscore::control
