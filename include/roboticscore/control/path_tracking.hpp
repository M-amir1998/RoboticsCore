#pragma once

/// @file path_tracking.hpp
/// @brief Common interface of the car-like path-tracking controllers.
///
/// Conventions shared by all trackers:
///   - Vehicle state is the kinematic-bicycle state (x, y, yaw, v) at the rear axle.
///   - The reference path stores the vehicle yaw as heading, the steering curvature
///     tan(delta) / L as curvature, and a signed reference speed (negative = reverse).
///   - Lateral errors are positive when the vehicle is left of the path; heading errors are
///     yaw - path heading.
///   - Pure Pursuit, Stanley and LQR control steering and use a PID speed loop; MPC controls
///     acceleration and steering jointly.

#include <cstddef>
#include <memory>
#include <string_view>
#include <vector>

#include "roboticscore/control/pid.hpp"
#include "roboticscore/core/types.hpp"
#include "roboticscore/models/bicycle.hpp"
#include "roboticscore/trajectory/reference_path.hpp"

namespace roboticscore::control {

/// Actuator command for a car-like vehicle.
struct VehicleCommand {
  double acceleration = 0.0;  ///< [m/s^2]
  double steering = 0.0;      ///< Front-wheel angle [rad]
};

/// Diagnostics of the last control step.
struct TrackingStatus {
  double cross_track_error = 0.0;
  double heading_error = 0.0;
  double reference_speed = 0.0;
  double path_s = 0.0;           ///< Arc length of the projection.
  Vec2 target = Vec2::Zero();    ///< Lookahead / reference point used.
  std::vector<Vec2> prediction;  ///< Predicted positions (MPC only).
  bool finished = false;         ///< Projection reached the end of the path.
};

/// Base class of the path trackers (runtime polymorphism lets benchmarks treat all
/// controllers uniformly).
class PathTracker {
 public:
  PathTracker(const models::BicycleParams& vehicle, const PidConfig& speed_pid);
  virtual ~PathTracker() = default;
  PathTracker(const PathTracker&) = default;
  PathTracker& operator=(const PathTracker&) = default;
  PathTracker(PathTracker&&) = default;
  PathTracker& operator=(PathTracker&&) = default;

  /// Computes the command for the current state.
  /// @throws std::invalid_argument if dt is not positive.
  virtual VehicleCommand compute(const models::BicycleState& state,
                                 const trajectory::ReferencePath& path, double dt) = 0;

  /// Clears internal state (projection hint, integrators, warm starts).
  virtual void reset();

  [[nodiscard]] virtual std::string_view name() const = 0;
  [[nodiscard]] const TrackingStatus& status() const noexcept { return status_; }
  [[nodiscard]] const models::BicycleParams& vehicle() const noexcept { return vehicle_; }

 protected:
  /// Projects @p point onto the path using the stored hint and fills the common status
  /// fields for a vehicle with yaw @p yaw.
  trajectory::PathProjection track_point(const Vec2& point, double yaw,
                                         const trajectory::ReferencePath& path);
  /// PID speed loop towards the reference speed at the current projection.
  double speed_control(double v, double dt);
  double clamp_steering(double delta) const;

  models::BicycleParams vehicle_;
  PidController speed_pid_;
  TrackingStatus status_;
  std::ptrdiff_t hint_ = -1;
};

}  // namespace roboticscore::control
