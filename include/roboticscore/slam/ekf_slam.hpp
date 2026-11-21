#pragma once

/// @file ekf_slam.hpp
/// @brief EKF-SLAM with point landmarks and known data association (Smith, Self & Cheeseman
///        1990; Thrun, Burgard & Fox, "Probabilistic Robotics", ch. 10).
///
/// The joint state is (x, y, theta, m_1x, m_1y, ..., m_Nx, m_Ny). Landmarks are added when first
/// observed (state augmentation) using the inverse range-bearing model, with the full
/// cross-covariance to the robot and to the existing map. Landmark IDs are arbitrary
/// non-negative integers; a lookup table maps them to state indices in order of first sight.
///
/// Prediction only touches the robot block and the robot-map cross-covariances, O(N); an
/// update with one landmark costs O(N^2). EKF-SLAM is therefore practical up to a few hundred
/// landmarks and becomes inconsistent with large heading uncertainty (linearisation errors).

#include <cstddef>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/localization/robot_models.hpp"
#include "roboticscore/models/unicycle.hpp"

namespace roboticscore::slam {

/// A range-bearing observation of landmark @p id.
struct RangeBearingObservation {
  std::size_t id = 0;
  double range = 0.0;
  double bearing = 0.0;
};

/// EKF-SLAM filter.
class EkfSlam {
 public:
  /// @throws std::invalid_argument for non-positive noise or an invalid covariance.
  EkfSlam(const geometry::Pose2& initial_pose, const Mat3& initial_covariance,
          const localization::VelocityNoise& motion_noise, double range_std, double bearing_std);

  /// Propagates the robot with the odometry command @p u for @p dt seconds.
  void predict(const models::UnicycleInput& u, double dt);

  /// Fuses observations; unseen landmarks are initialised. Returns the number of new landmarks.
  std::size_t update(std::span<const RangeBearingObservation> observations);

  [[nodiscard]] geometry::Pose2 pose() const;
  [[nodiscard]] Mat3 pose_covariance() const { return p_.topLeftCorner<3, 3>(); }
  [[nodiscard]] const VecX& state() const noexcept { return x_; }
  [[nodiscard]] const MatX& covariance() const noexcept { return p_; }

  [[nodiscard]] std::size_t landmark_count() const noexcept { return ids_.size(); }
  /// IDs in order of first observation.
  [[nodiscard]] const std::vector<std::size_t>& landmark_ids() const noexcept { return ids_; }
  [[nodiscard]] std::optional<Vec2> landmark(std::size_t id) const;
  [[nodiscard]] std::optional<Mat2> landmark_covariance(std::size_t id) const;

 private:
  void add_landmark(const RangeBearingObservation& z);
  void correct(std::size_t slot, const RangeBearingObservation& z);

  VecX x_;
  MatX p_;
  localization::VelocityNoise motion_noise_;
  Mat2 r_;
  std::vector<std::size_t> ids_;
  std::unordered_map<std::size_t, std::size_t> slot_;  // id -> landmark slot
};

}  // namespace roboticscore::slam
