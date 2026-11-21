#include <algorithm>
#include <cmath>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/sim/sensors.hpp"
#include "roboticscore/sim/world.hpp"

namespace roboticscore::sim {

// ----------------------------------------------------------------------------- World

void DynamicObstacle::advance(double dt, const geometry::Aabb& bounds) {
  shape.center += velocity * dt;
  for (int axis = 0; axis < 2; ++axis) {
    const double lo = bounds.min(axis) + shape.radius;
    const double hi = bounds.max(axis) - shape.radius;
    if (shape.center(axis) < lo) {
      shape.center(axis) = 2.0 * lo - shape.center(axis);
      velocity(axis) = std::abs(velocity(axis));
    } else if (shape.center(axis) > hi) {
      shape.center(axis) = 2.0 * hi - shape.center(axis);
      velocity(axis) = -std::abs(velocity(axis));
    }
  }
}

double World::clearance(const Vec2& p) const {
  double best = environment.clearance(p);
  for (const DynamicObstacle& o : dynamic_obstacles) {
    best = std::min(best, geometry::signed_distance(p, o.shape));
  }
  return best;
}

std::optional<double> World::ray_cast(const Vec2& origin, double angle, double max_range) const {
  std::optional<double> best = environment.ray_cast(origin, angle, max_range);
  const Vec2 dir(std::cos(angle), std::sin(angle));
  for (const DynamicObstacle& o : dynamic_obstacles) {
    const auto t = geometry::ray_cast(origin, dir, o.shape);
    if (t && *t <= max_range && (!best || *t < *best)) {
      best = t;
    }
  }
  return best;
}

void World::advance(double dt) {
  for (DynamicObstacle& o : dynamic_obstacles) {
    o.advance(dt, environment.bounds());
  }
}

// ----------------------------------------------------------------------------- sensors

models::UnicycleInput OdometryNoise::corrupt(const models::UnicycleInput& u,
                                             math::Random& rng) const {
  const double v2 = u.v * u.v;
  const double w2 = u.omega * u.omega;
  const double var_v = alpha1 * v2 + alpha2 * w2 + floor_var_v;
  const double var_w = alpha3 * v2 + alpha4 * w2 + floor_var_omega;
  return {u.v + rng.normal(0.0, std::sqrt(var_v)), u.omega + rng.normal(0.0, std::sqrt(var_w))};
}

std::vector<LandmarkObservation> RangeBearingSensor::observe(const geometry::Pose2& pose,
                                                             const std::vector<Vec2>& landmarks,
                                                             math::Random& rng) const {
  detail::require(max_range > 0.0 && fov > 0.0, "RangeBearingSensor: range and fov must be > 0");
  detail::require(outlier_probability >= 0.0 && outlier_probability <= 1.0,
                  "RangeBearingSensor: outlier probability must lie in [0, 1]");
  std::vector<LandmarkObservation> out;
  for (std::size_t i = 0; i < landmarks.size(); ++i) {
    const Vec2 d = landmarks[i] - pose.translation();
    const double range = d.norm();
    const double bearing = math::wrap_to_pi(std::atan2(d.y(), d.x()) - pose.theta());
    if (range > max_range || std::abs(bearing) > 0.5 * fov) {
      continue;
    }
    LandmarkObservation obs;
    obs.id = i;
    // Draw both noise samples unconditionally so the stream does not depend on outliers.
    const double nr = rng.normal(0.0, range_std);
    const double nb = rng.normal(0.0, bearing_std);
    const double u = rng.uniform01();
    if (u < outlier_probability) {
      obs.range = rng.uniform(0.0, max_range);
      obs.bearing = rng.uniform(-0.5 * fov, 0.5 * fov);
      obs.outlier = true;
    } else {
      obs.range = std::max(0.0, range + nr);
      obs.bearing = math::wrap_to_pi(bearing + nb);
    }
    out.push_back(obs);
  }
  return out;
}

Vec2 PositionSensor::observe(const geometry::Pose2& pose, math::Random& rng) const {
  return pose.translation() + Vec2(rng.normal(0.0, position_std), rng.normal(0.0, position_std));
}

LidarScan LidarSensor::scan(const geometry::Pose2& pose, const World& world,
                            math::Random& rng) const {
  detail::require(beam_count > 0, "LidarSensor: beam_count must be positive");
  detail::require(max_range > 0.0, "LidarSensor: max_range must be positive");
  LidarScan scan;
  scan.angles.resize(static_cast<std::size_t>(beam_count));
  scan.ranges.resize(scan.angles.size());
  scan.hit.resize(scan.angles.size());
  // A full circle must not duplicate its first beam at +pi.
  const bool full_circle = fov >= math::kTwoPi - 1e-9;
  const double span = full_circle ? math::kTwoPi : fov;
  const double step =
      (full_circle || beam_count == 1) ? span / beam_count : span / (beam_count - 1);
  for (int i = 0; i < beam_count; ++i) {
    const double rel = -0.5 * span + step * i;
    const auto idx = static_cast<std::size_t>(i);
    scan.angles[idx] = rel;
    const auto hit = world.ray_cast(pose.translation(), pose.theta() + rel, max_range);
    const double noise = rng.normal(0.0, range_std);
    if (hit) {
      scan.ranges[idx] = std::clamp(*hit + noise, 0.0, max_range);
      scan.hit[idx] = true;
    } else {
      scan.ranges[idx] = max_range;
      scan.hit[idx] = false;
    }
  }
  return scan;
}

}  // namespace roboticscore::sim
