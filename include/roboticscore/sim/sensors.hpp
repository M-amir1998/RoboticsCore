#pragma once

/// @file sensors.hpp
/// @brief Simulated sensors with explicit, documented noise models.
///
/// Every sensor draws from the math::Random it is given; the Simulation class gives each
/// sensor an independent stream derived from the scenario seed so adding a sensor never
/// changes the noise realisation of another.

#include <cstddef>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/models/unicycle.hpp"
#include "roboticscore/sim/world.hpp"

namespace roboticscore::sim {

/// Odometry (velocity) noise of the true robot, in the same parameterisation as
/// localization::VelocityNoise: var(v) = a1 v^2 + a2 w^2 + v_floor, var(w) = a3 v^2 + a4 w^2 +
/// w_floor. Kept separate so simulations can use a different "true" noise than the filter's
/// model.
struct OdometryNoise {
  double alpha1 = 0.0;
  double alpha2 = 0.0;
  double alpha3 = 0.0;
  double alpha4 = 0.0;
  double floor_var_v = 0.0;
  double floor_var_omega = 0.0;

  /// Command @p u corrupted by zero-mean Gaussian noise.
  [[nodiscard]] models::UnicycleInput corrupt(const models::UnicycleInput& u,
                                              math::Random& rng) const;
};

/// Range-bearing observation of landmark @p id.
struct LandmarkObservation {
  std::size_t id = 0;
  double range = 0.0;
  double bearing = 0.0;
  bool outlier = false;  ///< Ground-truth flag for evaluation; filters must not read it.
};

/// Range-bearing landmark sensor with limited range and field of view.
struct RangeBearingSensor {
  double max_range = 10.0;
  double fov = math::kTwoPi;  ///< Total field of view centred on the heading [rad].
  double range_std = 0.1;
  double bearing_std = 0.02;
  /// Probability that an observation is replaced by a uniformly random (range, bearing).
  double outlier_probability = 0.0;

  [[nodiscard]] std::vector<LandmarkObservation> observe(const geometry::Pose2& pose,
                                                         const std::vector<Vec2>& landmarks,
                                                         math::Random& rng) const;
};

/// Absolute position sensor (GPS-like) with isotropic Gaussian noise.
struct PositionSensor {
  double position_std = 0.5;
  [[nodiscard]] Vec2 observe(const geometry::Pose2& pose, math::Random& rng) const;
};

/// Planar LiDAR scan. ranges[i] == max_range with hit[i] == false means no return.
struct LidarScan {
  std::vector<double> angles;  ///< Beam angles relative to the robot heading [rad].
  std::vector<double> ranges;
  std::vector<bool> hit;
};

/// Planar LiDAR: equally spaced beams over the field of view, Gaussian range noise.
struct LidarSensor {
  int beam_count = 180;
  double fov = math::kTwoPi;
  double max_range = 8.0;
  double range_std = 0.02;

  [[nodiscard]] LidarScan scan(const geometry::Pose2& pose, const World& world,
                               math::Random& rng) const;
};

}  // namespace roboticscore::sim
