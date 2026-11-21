#include "roboticscore/slam/fastslam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include "roboticscore/core/error.hpp"
#include "roboticscore/estimation/resampling.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/statistics.hpp"
#include "roboticscore/registration/icp.hpp"

namespace roboticscore::slam {

FastSlam::FastSlam(std::size_t particle_count, const geometry::Pose2& initial_pose,
                   const localization::VelocityNoise& motion_noise, double range_std,
                   double bearing_std, std::uint64_t seed, double resample_threshold)
    : motion_noise_(motion_noise), rng_(seed), resample_threshold_(resample_threshold) {
  detail::require(particle_count > 0, "FastSlam: at least one particle is required");
  detail::require(range_std > 0.0 && bearing_std > 0.0, "FastSlam: noise must be positive");
  detail::require(resample_threshold >= 0.0 && resample_threshold <= 1.0,
                  "FastSlam: resample threshold must lie in [0, 1]");
  r_ = Vec2(range_std * range_std, bearing_std * bearing_std).asDiagonal();
  const double lw = -std::log(static_cast<double>(particle_count));
  particles_.assign(particle_count, FastSlamParticle{initial_pose, {}, lw});
}

void FastSlam::predict(const models::UnicycleInput& u, double dt) {
  const Mat2 m = motion_noise_.control_covariance(u.v, u.omega);
  for (auto& p : particles_) {
    const models::UnicycleInput noisy{u.v + rng_.normal(0.0, std::sqrt(m(0, 0))),
                                      u.omega + rng_.normal(0.0, std::sqrt(m(1, 1)))};
    p.pose = models::Unicycle::step(p.pose, noisy, dt);
  }
}

bool FastSlam::update(std::span<const RangeBearingObservation> observations) {
  for (auto& p : particles_) {
    const double x = p.pose.x();
    const double y = p.pose.y();
    const double th = p.pose.theta();
    for (const auto& z : observations) {
      if (z.id >= p.landmarks.size()) {
        p.landmarks.resize(z.id + 1);
      }
      LandmarkEstimate& lm = p.landmarks[z.id];
      const double c = std::cos(th + z.bearing);
      const double s = std::sin(th + z.bearing);
      if (!lm.initialized) {
        lm.mean = Vec2(x + z.range * c, y + z.range * s);
        Mat2 g;  // d landmark / d (range, bearing)
        g << c, -z.range * s, s, z.range * c;
        lm.covariance = g * r_ * g.transpose();
        lm.initialized = true;
        continue;
      }
      const double dx = lm.mean.x() - x;
      const double dy = lm.mean.y() - y;
      const double q = dx * dx + dy * dy;
      const double r = std::sqrt(q);
      if (r < 1e-9) continue;
      Mat2 h;  // d (range, bearing) / d landmark
      h << dx / r, dy / r, -dy / q, dx / q;
      const Vec2 innovation(z.range - r,
                            math::angle_difference(z.bearing, std::atan2(dy, dx) - th));
      const Mat2 s_mat = h * lm.covariance * h.transpose() + r_;
      const Eigen::Matrix<double, 2, 2> k = lm.covariance * h.transpose() * s_mat.inverse();
      lm.mean += k * innovation;
      const Mat2 i_kh = Mat2::Identity() - k * h;
      lm.covariance = i_kh * lm.covariance * i_kh.transpose() + k * r_ * k.transpose();
      p.log_weight += math::gaussian_log_pdf(innovation, s_mat);
    }
  }
  // Normalise in the log domain.
  double max_lw = -std::numeric_limits<double>::infinity();
  for (const auto& p : particles_)
    max_lw = std::max(max_lw, p.log_weight);
  double sum = 0.0;
  for (const auto& p : particles_)
    sum += std::exp(p.log_weight - max_lw);
  const double log_norm = max_lw + std::log(sum);
  for (auto& p : particles_)
    p.log_weight -= log_norm;

  if (effective_sample_size() >= resample_threshold_ * static_cast<double>(particles_.size())) {
    return false;
  }
  std::vector<double> w(particles_.size());
  for (std::size_t i = 0; i < w.size(); ++i)
    w[i] = std::exp(particles_[i].log_weight);
  const auto idx =
      estimation::resample(w, particles_.size(), estimation::ResamplingMethod::kSystematic, rng_);
  std::vector<FastSlamParticle> next;
  next.reserve(particles_.size());
  const double lw = -std::log(static_cast<double>(particles_.size()));
  for (const std::size_t i : idx) {
    next.push_back(particles_[i]);
    next.back().log_weight = lw;
  }
  particles_ = std::move(next);
  return true;
}

double FastSlam::effective_sample_size() const {
  double sq = 0.0;
  for (const auto& p : particles_)
    sq += std::exp(2.0 * p.log_weight);
  return 1.0 / sq;
}

const FastSlamParticle& FastSlam::best_particle() const {
  return *std::max_element(particles_.begin(), particles_.end(), [](const auto& a, const auto& b) {
    return a.log_weight < b.log_weight;
  });
}

geometry::Pose2 FastSlam::mean_pose() const {
  double x = 0.0;
  double y = 0.0;
  double s = 0.0;
  double c = 0.0;
  for (const auto& p : particles_) {
    const double w = std::exp(p.log_weight);
    x += w * p.pose.x();
    y += w * p.pose.y();
    s += w * std::sin(p.pose.theta());
    c += w * std::cos(p.pose.theta());
  }
  return {x, y, std::atan2(s, c)};
}

double absolute_trajectory_error(const std::vector<geometry::Pose2>& estimate,
                                 const std::vector<geometry::Pose2>& truth, bool align) {
  detail::require(!estimate.empty() && estimate.size() == truth.size(),
                  "absolute_trajectory_error: trajectories must be non-empty and equal length");
  std::vector<Vec2> a;
  std::vector<Vec2> b;
  for (std::size_t i = 0; i < estimate.size(); ++i) {
    a.push_back(estimate[i].translation());
    b.push_back(truth[i].translation());
  }
  const geometry::SE2 t = align ? registration::best_fit_transform(a, b) : geometry::SE2();
  double ss = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i)
    ss += (t * a[i] - b[i]).squaredNorm();
  return std::sqrt(ss / static_cast<double>(a.size()));
}

double relative_pose_error(const std::vector<geometry::Pose2>& estimate,
                           const std::vector<geometry::Pose2>& truth, std::size_t delta) {
  detail::require(estimate.size() == truth.size(), "relative_pose_error: length mismatch");
  detail::require(delta > 0 && delta < estimate.size(), "relative_pose_error: invalid delta");
  double ss = 0.0;
  std::size_t n = 0;
  for (std::size_t i = 0; i + delta < estimate.size(); ++i) {
    const geometry::Pose2 de = estimate[i].between(estimate[i + delta]);
    const geometry::Pose2 dt = truth[i].between(truth[i + delta]);
    ss += dt.between(de).translation().squaredNorm();
    ++n;
  }
  return std::sqrt(ss / static_cast<double>(n));
}

}  // namespace roboticscore::slam
