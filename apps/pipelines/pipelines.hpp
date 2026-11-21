#pragma once

/// @file pipelines.hpp
/// @brief End-to-end scenarios that combine several RoboticsCore modules. Shared by the demo
///        executable (which serialises them for visualisation) and the integration tests
///        (which assert on their outcomes). Not part of the installed library.

#include <cstdint>
#include <vector>

#include "roboticscore/control/path_tracking.hpp"
#include "roboticscore/geometry/collision.hpp"
#include "roboticscore/geometry/grid.hpp"
#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/planning/kinodynamic/reeds_shepp.hpp"
#include "roboticscore/sim/scenarios.hpp"
#include "roboticscore/trajectory/reference_path.hpp"

namespace roboticscore::pipelines {

// ----------------------------------------------------------------------- mobile robot (hero)

/// One recorded simulation step of the mobile-robot pipeline.
struct HeroFrame {
  double time = 0.0;
  geometry::Pose2 truth;
  Vec3 estimate = Vec3::Zero();
  Mat3 covariance = Mat3::Identity();
  Vec2 target = Vec2::Zero();        ///< Lookahead point handed to DWA.
  std::vector<Vec2> chosen_rollout;  ///< DWA trajectory that was executed.
  std::vector<Vec2> lidar_hits;      ///< World-frame LiDAR returns (subsampled).
  std::vector<geometry::Circle> dynamic_obstacles;
  std::vector<std::size_t> observed_landmarks;
  double clearance = 0.0;  ///< True distance from the robot centre to the nearest obstacle.
};

/// Occupancy probabilities (0..255) at one point in time.
struct MapSnapshot {
  std::size_t frame = 0;
  std::vector<std::uint8_t> probability;
};

/// Result of the integrated mobile-robot run.
struct HeroResult {
  sim::Scenario scenario = sim::open_space_scenario();
  std::vector<Vec2> landmarks;
  geometry::GridSpec map_spec;
  std::vector<Vec2> global_path;
  std::vector<HeroFrame> frames;
  std::vector<MapSnapshot> maps;
  bool reached_goal = false;
  bool collision = false;
  double position_rmse = 0.0;  ///< EKF position error over the run [m].
  double heading_rmse = 0.0;   ///< [rad]
  double map_accuracy = 0.0;   ///< Final map vs ground truth (observed cells).
};

/// Noisy odometry + landmarks -> EKF localisation, LiDAR -> occupancy mapping,
/// Theta* global plan -> lookahead carrot -> DWA velocity commands, with moving obstacles.
[[nodiscard]] HeroResult run_hero(std::uint64_t seed);

// --------------------------------------------------------------------------- car parking

/// Result of the parking pipeline.
struct ParkingResult {
  sim::Scenario scenario = sim::open_space_scenario();
  std::vector<planning::DirectedPose> planned;  ///< Hybrid A* output.
  std::vector<geometry::Pose2> expanded;        ///< Hybrid A* expansion order.
  std::vector<models::BicycleState> driven;     ///< Simulated vehicle states.
  std::vector<double> steering;                 ///< Applied steering per step.
  std::vector<std::vector<Vec2>> predictions;   ///< MPC predictions per step.
  models::VehicleFootprint footprint;
  bool planned_ok = false;
  bool parked = false;
  double final_position_error = 0.0;
  double final_heading_error = 0.0;
  double max_cross_track = 0.0;
};

/// Hybrid A* parking plan, split at gear changes, each segment tracked by MPC on a kinematic
/// bicycle simulation.
[[nodiscard]] ParkingResult run_parking();

// ------------------------------------------------------------------------- pose-graph SLAM

/// Result of the loop-closure SLAM pipeline.
struct LoopSlamResult {
  std::vector<geometry::Pose2> truth;
  std::vector<geometry::Pose2> odometry;                 ///< Initial (dead-reckoned) estimate.
  std::vector<std::vector<geometry::Pose2>> iterations;  ///< Estimate after each GN step.
  std::vector<std::pair<std::size_t, std::size_t>> loop_closures;
  std::vector<double> chi2;
  double ate_before = 0.0;
  double ate_after = 0.0;
};

/// Robot drives two laps around a block with drifting odometry; simulated place
/// recognition adds loop closures; Gauss-Newton optimises the pose graph.
[[nodiscard]] LoopSlamResult run_loop_slam(std::uint64_t seed);

// ----------------------------------------------------------------- closed-loop tracking

/// Closed-loop run of a path tracker on the kinematic bicycle.
struct TrackingRun {
  std::vector<models::BicycleState> states;
  std::vector<double> steering;
  std::vector<double> acceleration;
  std::vector<double> cross_track;
  std::vector<double> heading_error;
  std::vector<double> compute_time_us;  ///< Wall-clock time of each compute() call.
  bool finished = false;
};

/// Simulates @p tracker following @p path from @p start with control period @p dt.
[[nodiscard]] TrackingRun run_tracking(control::PathTracker& tracker,
                                       const trajectory::ReferencePath& path,
                                       const models::BicycleParams& vehicle,
                                       const models::BicycleState& start, double dt, int max_steps);

}  // namespace roboticscore::pipelines
