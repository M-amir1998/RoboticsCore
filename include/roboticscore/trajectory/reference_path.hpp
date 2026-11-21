#pragma once

/// @file reference_path.hpp
/// @brief Densely sampled reference path with arc length, heading, curvature and driving
///        direction; the common input of all path-tracking controllers.

#include <cstddef>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/se2.hpp"

namespace roboticscore::trajectory {

/// One sample of a reference path.
struct PathPoint {
  double s = 0.0;  ///< Arc length from the start [m].
  Vec2 position = Vec2::Zero();
  double heading = 0.0;    ///< Tangent direction of travel [rad].
  double curvature = 0.0;  ///< Signed curvature [1/m].
  double speed = 0.0;      ///< Reference speed (signed: negative when reversing) [m/s].
};

/// Projection of a point onto the path.
struct PathProjection {
  std::size_t index = 0;      ///< Segment start index.
  double s = 0.0;             ///< Arc length of the projection.
  Vec2 point = Vec2::Zero();  ///< Closest point on the path.
  double cross_track = 0.0;  ///< Signed lateral error, positive when the query is left of the path.
  double heading = 0.0;      ///< Path heading at the projection.
  double curvature = 0.0;
};

/// Sampled path with O(log n) arc-length lookup and windowed nearest-point search.
class ReferencePath {
 public:
  ReferencePath() = default;

  /// @throws std::invalid_argument for fewer than two points or decreasing arc length.
  explicit ReferencePath(std::vector<PathPoint> points);

  /// Samples a cubic spline through @p waypoints every @p spacing metres; speed is set to
  /// @p speed everywhere.
  /// @throws std::invalid_argument for invalid input.
  [[nodiscard]] static ReferencePath from_waypoints(const std::vector<Vec2>& waypoints,
                                                    double spacing, double speed = 1.0);

  [[nodiscard]] const std::vector<PathPoint>& points() const noexcept { return points_; }
  [[nodiscard]] std::size_t size() const noexcept { return points_.size(); }
  [[nodiscard]] const PathPoint& operator[](std::size_t i) const { return points_[i]; }
  [[nodiscard]] double length() const { return points_.back().s; }

  /// Linear interpolation of position, heading (on the circle), curvature and speed at s
  /// (clamped to the path).
  [[nodiscard]] PathPoint at(double s) const;

  /// Closest point on the polyline. If @p hint is given, only segments within
  /// [hint - window, hint + window] are searched, which keeps tracking O(window) per step and
  /// avoids jumping to another part of a self-intersecting path.
  [[nodiscard]] PathProjection project(const Vec2& p, std::ptrdiff_t hint = -1,
                                       std::size_t window = 50) const;

  /// Replaces every speed with a curvature- and acceleration-limited profile (see
  /// time_parameterization.hpp); returns the total traversal time.
  double apply_speed_profile(double v_max, double a_max, double a_lat_max, double v_start = 0.0,
                             double v_end = 0.0);

 private:
  std::vector<PathPoint> points_;
};

}  // namespace roboticscore::trajectory
