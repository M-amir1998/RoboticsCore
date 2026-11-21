#pragma once

/// @file fastslam.hpp
/// @brief FastSLAM 1.0 with known data association (Montemerlo, Thrun, Koller & Wegbreit,
///        AAAI 2002).
///
/// The posterior over robot path and map factors into a particle filter over paths and,
/// conditioned on each path, independent 2x2 EKFs per landmark (Rao-Blackwellisation).
/// Per step and particle: sample the motion model, update the EKF of each observed landmark
/// and multiply the weight by the measurement likelihood N(innovation; 0, S). New landmarks
/// are initialised with the inverse observation model and leave the weight unchanged.
/// Resampling (systematic) happens when the effective sample size falls below a threshold.
///
/// Complexity per step: O(M K) for M particles and K observations (plus O(M N) map copies on
/// resampling in this array-based implementation; tree-based maps reduce that to O(M log N)).

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/localization/robot_models.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/models/unicycle.hpp"
#include "roboticscore/slam/ekf_slam.hpp"

namespace roboticscore::slam {

/// Landmark estimate inside one particle.
struct LandmarkEstimate {
  bool initialized = false;
  Vec2 mean = Vec2::Zero();
  Mat2 covariance = Mat2::Identity();
};

/// One path hypothesis with its map.
struct FastSlamParticle {
  geometry::Pose2 pose;
  std::vector<LandmarkEstimate> landmarks;  ///< Indexed by landmark id.
  double log_weight = 0.0;
};

/// FastSLAM 1.0 filter.
class FastSlam {
 public:
  /// @throws std::invalid_argument for zero particles or non-positive noise.
  FastSlam(std::size_t particle_count, const geometry::Pose2& initial_pose,
           const localization::VelocityNoise& motion_noise, double range_std, double bearing_std,
           std::uint64_t seed, double resample_threshold = 0.5);

  /// Samples every particle's motion from the velocity noise model.
  void predict(const models::UnicycleInput& u, double dt);

  /// Landmark updates and reweighting; resamples if needed. Returns true if it resampled.
  bool update(std::span<const RangeBearingObservation> observations);

  /// Highest-weight particle.
  [[nodiscard]] const FastSlamParticle& best_particle() const;
  /// Weighted mean pose (circular mean for the heading).
  [[nodiscard]] geometry::Pose2 mean_pose() const;
  [[nodiscard]] const std::vector<FastSlamParticle>& particles() const noexcept {
    return particles_;
  }
  [[nodiscard]] double effective_sample_size() const;

 private:
  std::vector<FastSlamParticle> particles_;
  localization::VelocityNoise motion_noise_;
  Mat2 r_;
  math::Random rng_;
  double resample_threshold_;
};

/// Root-mean-square position error between estimated and true trajectories. With @p align the
/// estimate is first rigidly aligned to the truth (least squares), which removes the gauge
/// freedom of SLAM; without it errors are measured in the shared world frame.
/// @throws std::invalid_argument for empty or mismatched trajectories.
[[nodiscard]] double absolute_trajectory_error(const std::vector<geometry::Pose2>& estimate,
                                               const std::vector<geometry::Pose2>& truth,
                                               bool align = true);

/// Relative pose error: RMS translation error of relative motions over @p delta steps,
/// insensitive to accumulated drift elsewhere in the trajectory.
/// @throws std::invalid_argument for mismatched trajectories or delta == 0.
[[nodiscard]] double relative_pose_error(const std::vector<geometry::Pose2>& estimate,
                                         const std::vector<geometry::Pose2>& truth,
                                         std::size_t delta = 1);

}  // namespace roboticscore::slam
