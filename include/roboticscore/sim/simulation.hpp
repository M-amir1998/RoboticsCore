#pragma once

/// @file simulation.hpp
/// @brief Deterministic fixed-step simulation of a unicycle robot in a World.
///
/// Given the same World, initial pose, configuration and seed, every call sequence produces
/// bit-identical results on every platform (see math::Random for why).

#include <cstdint>
#include <vector>

#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/models/unicycle.hpp"
#include "roboticscore/sim/sensors.hpp"
#include "roboticscore/sim/world.hpp"

namespace roboticscore::sim {

/// Simulation parameters.
struct SimulationConfig {
  double dt = 0.1;
  std::uint64_t seed = 1;
  double robot_radius = 0.3;
  /// Noise between the commanded and the executed velocity (actuation noise).
  OdometryNoise actuation_noise;
  /// Noise between the executed velocity and what odometry reports.
  OdometryNoise odometry_noise;
  RangeBearingSensor landmark_sensor;
  PositionSensor position_sensor;
  LidarSensor lidar;
};

/// Result of one simulation step.
struct StepResult {
  double time = 0.0;
  geometry::Pose2 pose;            ///< Ground-truth pose after the step.
  models::UnicycleInput executed;  ///< Velocity actually applied.
  models::UnicycleInput odometry;  ///< Velocity reported by odometry.
  bool collision = false;          ///< Robot disc overlaps an obstacle.
};

/// Fixed-step simulator with independent noise streams per sensor.
class Simulation {
 public:
  /// @throws std::invalid_argument for non-positive dt or robot radius.
  Simulation(World world, const geometry::Pose2& initial_pose, const SimulationConfig& config);

  /// Applies @p command for one time step and advances dynamic obstacles.
  StepResult step(const models::UnicycleInput& command);

  [[nodiscard]] std::vector<LandmarkObservation> observe_landmarks();
  [[nodiscard]] Vec2 observe_position();
  [[nodiscard]] LidarScan scan_lidar();

  [[nodiscard]] const geometry::Pose2& pose() const noexcept { return pose_; }
  [[nodiscard]] double time() const noexcept { return time_; }
  [[nodiscard]] const World& world() const noexcept { return world_; }
  [[nodiscard]] const SimulationConfig& config() const noexcept { return config_; }
  [[nodiscard]] bool in_collision() const;

 private:
  World world_;
  geometry::Pose2 pose_;
  SimulationConfig config_;
  double time_ = 0.0;
  math::Random actuation_rng_;
  math::Random odometry_rng_;
  math::Random landmark_rng_;
  math::Random position_rng_;
  math::Random lidar_rng_;
};

}  // namespace roboticscore::sim
