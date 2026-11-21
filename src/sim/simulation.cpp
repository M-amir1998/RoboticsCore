#include "roboticscore/sim/simulation.hpp"

#include <utility>

#include "roboticscore/core/error.hpp"

namespace roboticscore::sim {
namespace {

// Stream identifiers for Random::derive; fixed so adding sensors never reshuffles noise.
constexpr std::uint64_t kActuationStream = 1;
constexpr std::uint64_t kOdometryStream = 2;
constexpr std::uint64_t kLandmarkStream = 3;
constexpr std::uint64_t kPositionStream = 4;
constexpr std::uint64_t kLidarStream = 5;

}  // namespace

Simulation::Simulation(World world, const geometry::Pose2& initial_pose,
                       const SimulationConfig& config)
    : world_(std::move(world)),
      pose_(initial_pose),
      config_(config),
      actuation_rng_(math::Random(config_.seed).derive(kActuationStream)),
      odometry_rng_(math::Random(config_.seed).derive(kOdometryStream)),
      landmark_rng_(math::Random(config_.seed).derive(kLandmarkStream)),
      position_rng_(math::Random(config_.seed).derive(kPositionStream)),
      lidar_rng_(math::Random(config_.seed).derive(kLidarStream)) {
  detail::require(config_.dt > 0.0, "Simulation: dt must be positive");
  detail::require(config_.robot_radius > 0.0, "Simulation: robot radius must be positive");
}

StepResult Simulation::step(const models::UnicycleInput& command) {
  StepResult r;
  r.executed = config_.actuation_noise.corrupt(command, actuation_rng_);
  r.odometry = config_.odometry_noise.corrupt(r.executed, odometry_rng_);
  pose_ = models::Unicycle::step(pose_, r.executed, config_.dt);
  world_.advance(config_.dt);
  time_ += config_.dt;
  r.time = time_;
  r.pose = pose_;
  r.collision = in_collision();
  return r;
}

std::vector<LandmarkObservation> Simulation::observe_landmarks() {
  return config_.landmark_sensor.observe(pose_, world_.landmarks, landmark_rng_);
}

Vec2 Simulation::observe_position() {
  return config_.position_sensor.observe(pose_, position_rng_);
}

LidarScan Simulation::scan_lidar() {
  return config_.lidar.scan(pose_, world_, lidar_rng_);
}

bool Simulation::in_collision() const {
  return !world_.point_free(pose_.translation(), config_.robot_radius);
}

}  // namespace roboticscore::sim
