#include "roboticscore/slam/ekf_slam.hpp"

#include <cmath>

#include "roboticscore/core/error.hpp"
#include "roboticscore/estimation/kalman_filter.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/linear_algebra.hpp"

namespace roboticscore::slam {

EkfSlam::EkfSlam(const geometry::Pose2& initial_pose, const Mat3& initial_covariance,
                 const localization::VelocityNoise& motion_noise, double range_std,
                 double bearing_std)
    : x_(initial_pose.vector()), p_(initial_covariance), motion_noise_(motion_noise) {
  detail::require(range_std > 0.0 && bearing_std > 0.0,
                  "EkfSlam: measurement noise must be positive");
  estimation::validate_covariance(p_, 3, "EkfSlam");
  r_ = Vec2(range_std * range_std, bearing_std * bearing_std).asDiagonal();
}

geometry::Pose2 EkfSlam::pose() const {
  return {x_(0), x_(1), x_(2)};
}

void EkfSlam::predict(const models::UnicycleInput& u, double dt) {
  const Vec3 robot = x_.head<3>();
  const auto j = models::Unicycle::jacobians(robot, u, dt);
  x_.head<3>() = models::Unicycle::step(robot, u, dt);
  const Mat3 q =
      j.wrt_input * motion_noise_.control_covariance(u.v, u.omega) * j.wrt_input.transpose()
      + 1e-12 * Mat3::Identity();
  const Index n = x_.size();
  // Robot block and robot-map cross-covariances only: P_rr = F P_rr F^T + Q, P_rm = F P_rm.
  p_.topLeftCorner<3, 3>() = j.wrt_state * p_.topLeftCorner<3, 3>() * j.wrt_state.transpose() + q;
  if (n > 3) {
    const MatX prm = j.wrt_state * p_.topRightCorner(3, n - 3);
    p_.topRightCorner(3, n - 3) = prm;
    p_.bottomLeftCorner(n - 3, 3) = prm.transpose();
  }
}

std::size_t EkfSlam::update(std::span<const RangeBearingObservation> observations) {
  std::size_t added = 0;
  for (const auto& z : observations) {
    detail::require(z.range >= 0.0, "EkfSlam::update: range must be non-negative");
    const auto it = slot_.find(z.id);
    if (it == slot_.end()) {
      add_landmark(z);
      ++added;
    } else {
      correct(it->second, z);
    }
  }
  return added;
}

void EkfSlam::add_landmark(const RangeBearingObservation& z) {
  const double th = x_(2) + z.bearing;
  const double c = std::cos(th);
  const double s = std::sin(th);
  const Vec2 m(x_(0) + z.range * c, x_(1) + z.range * s);
  // Inverse-observation Jacobians with respect to the robot pose and the measurement.
  Eigen::Matrix<double, 2, 3> g_r;
  g_r << 1.0, 0.0, -z.range * s,  //
      0.0, 1.0, z.range * c;
  Mat2 g_z;
  g_z << c, -z.range * s,  //
      s, z.range * c;
  const Index n = x_.size();
  VecX x_new(n + 2);
  x_new << x_, m;
  MatX p_new = MatX::Zero(n + 2, n + 2);
  p_new.topLeftCorner(n, n) = p_;
  const MatX p_lx = g_r * p_.topRows(3);  // cross-covariance with every existing state
  p_new.block(n, 0, 2, n) = p_lx;
  p_new.block(0, n, n, 2) = p_lx.transpose();
  p_new.block(n, n, 2, 2) =
      g_r * p_.topLeftCorner<3, 3>() * g_r.transpose() + g_z * r_ * g_z.transpose();
  x_ = std::move(x_new);
  p_ = math::symmetrize(p_new);
  slot_.emplace(z.id, ids_.size());
  ids_.push_back(z.id);
}

void EkfSlam::correct(std::size_t slot, const RangeBearingObservation& z) {
  const Index li = 3 + 2 * static_cast<Index>(slot);
  const double dx = x_(li) - x_(0);
  const double dy = x_(li + 1) - x_(1);
  const double q = dx * dx + dy * dy;
  const double r = std::sqrt(q);
  if (r < 1e-9) {
    return;  // the bearing is undefined when the robot sits on the landmark
  }
  MatX h = MatX::Zero(2, x_.size());
  h.block<2, 3>(0, 0) << -dx / r, -dy / r, 0.0,  //
      dy / q, -dx / q, -1.0;
  h.block<2, 2>(0, li) << dx / r, dy / r,  //
      -dy / q, dx / q;
  const Vec2 predicted(r, math::wrap_to_pi(std::atan2(dy, dx) - x_(2)));
  VecX innovation(2);
  innovation << z.range - predicted(0), math::angle_difference(z.bearing, predicted(1));
  (void)estimation::kalman_update(x_, p_, innovation, h, r_);
  x_(2) = math::wrap_to_pi(x_(2));
}

std::optional<Vec2> EkfSlam::landmark(std::size_t id) const {
  const auto it = slot_.find(id);
  if (it == slot_.end()) return std::nullopt;
  return Vec2(x_.segment<2>(3 + 2 * static_cast<Index>(it->second)));
}

std::optional<Mat2> EkfSlam::landmark_covariance(std::size_t id) const {
  const auto it = slot_.find(id);
  if (it == slot_.end()) return std::nullopt;
  const Index li = 3 + 2 * static_cast<Index>(it->second);
  return Mat2(p_.block<2, 2>(li, li));
}

}  // namespace roboticscore::slam
