#pragma once

/// @file tracks.hpp
/// @brief Reference tracks for controller benchmarks and trajectory quality metrics.

#include <string>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/trajectory/reference_path.hpp"

namespace roboticscore::trajectory {

enum class TrackType { kStraight, kCircle, kSCurve, kSharpTurns, kVariableCurvature };

/// Benchmark track sampled every @p spacing metres with constant reference speed @p speed.
/// All tracks start at the origin heading along +x.
[[nodiscard]] ReferencePath make_track(TrackType type, double speed, double spacing = 0.25);

/// Names usable with track_from_name(): straight, circle, s_curve, sharp_turns,
/// variable_curvature.
[[nodiscard]] std::vector<std::string> track_names();

/// @throws std::invalid_argument for an unknown name.
[[nodiscard]] TrackType track_from_name(const std::string& name);

/// Summary statistics of a time-stamped planar trajectory, computed with finite differences.
struct TrajectoryMetrics {
  double length = 0.0;
  double duration = 0.0;
  double mean_speed = 0.0;
  double max_speed = 0.0;
  double max_abs_acceleration = 0.0;  ///< Tangential, from the speed signal.
  double rms_acceleration = 0.0;
  double max_abs_jerk = 0.0;
  double rms_jerk = 0.0;
  double max_abs_curvature = 0.0;  ///< Discrete (Menger) curvature of consecutive samples.
  double mean_abs_curvature = 0.0;
};

/// @throws std::invalid_argument if sizes differ, fewer than four samples are given or time
///         is not strictly increasing.
[[nodiscard]] TrajectoryMetrics evaluate_trajectory(const std::vector<double>& t,
                                                    const std::vector<Vec2>& positions);

}  // namespace roboticscore::trajectory
