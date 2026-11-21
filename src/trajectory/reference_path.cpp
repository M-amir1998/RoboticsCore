#include "roboticscore/trajectory/reference_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/trajectory/spline.hpp"
#include "roboticscore/trajectory/time_parameterization.hpp"

namespace roboticscore::trajectory {

ReferencePath::ReferencePath(std::vector<PathPoint> points) : points_(std::move(points)) {
  detail::require(points_.size() >= 2, "ReferencePath: at least two points are required");
  for (std::size_t i = 1; i < points_.size(); ++i) {
    detail::require(points_[i].s > points_[i - 1].s,
                    "ReferencePath: arc length must be strictly increasing");
  }
}

ReferencePath ReferencePath::from_waypoints(const std::vector<Vec2>& waypoints, double spacing,
                                            double speed) {
  detail::require(spacing > 0.0, "ReferencePath::from_waypoints: spacing must be positive");
  const CubicSpline2D spline(waypoints);
  const int n = std::max(1, static_cast<int>(std::ceil(spline.length() / spacing)));
  std::vector<PathPoint> pts;
  pts.reserve(static_cast<std::size_t>(n) + 1);
  for (int i = 0; i <= n; ++i) {
    const double s = spline.length() * i / n;
    pts.push_back({s, spline.position(s), spline.heading(s), spline.curvature(s), speed});
  }
  // Re-measure s along the sampled polyline so it matches the geometry controllers see.
  for (std::size_t i = 1; i < pts.size(); ++i) {
    pts[i].s = pts[i - 1].s + (pts[i].position - pts[i - 1].position).norm();
  }
  return ReferencePath(std::move(pts));
}

PathPoint ReferencePath::at(double s) const {
  if (s <= points_.front().s) return points_.front();
  if (s >= points_.back().s) return points_.back();
  const auto it = std::upper_bound(points_.begin(), points_.end(), s,
                                   [](double v, const PathPoint& p) { return v < p.s; });
  const PathPoint& b = *it;
  const PathPoint& a = *(it - 1);
  const double t = (s - a.s) / (b.s - a.s);
  return {s, a.position + t * (b.position - a.position),
          math::interpolate_angle(a.heading, b.heading, t),
          a.curvature + t * (b.curvature - a.curvature), a.speed + t * (b.speed - a.speed)};
}

PathProjection ReferencePath::project(const Vec2& p, std::ptrdiff_t hint,
                                      std::size_t window) const {
  std::size_t lo = 0;
  std::size_t hi = points_.size() - 1;
  if (hint >= 0) {
    const auto h = static_cast<std::size_t>(hint);
    lo = h > window ? h - window : 0;
    hi = std::min(points_.size() - 1, h + window);
  }
  PathProjection best;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = lo; i < hi; ++i) {
    const Vec2& a = points_[i].position;
    const Vec2& b = points_[i + 1].position;
    const Vec2 ab = b - a;
    const double len2 = ab.squaredNorm();
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    const Vec2 q = a + t * ab;
    const double d2 = (p - q).squaredNorm();
    if (d2 < best_d2) {
      best_d2 = d2;
      best.index = i;
      best.s = points_[i].s + t * (points_[i + 1].s - points_[i].s);
      best.point = q;
      best.heading = math::interpolate_angle(points_[i].heading, points_[i + 1].heading, t);
      best.curvature = points_[i].curvature + t * (points_[i + 1].curvature - points_[i].curvature);
    }
  }
  const Vec2 tangent(std::cos(best.heading), std::sin(best.heading));
  const Vec2 d = p - best.point;
  const double sign = tangent.x() * d.y() - tangent.y() * d.x() >= 0.0 ? 1.0 : -1.0;
  best.cross_track = sign * std::sqrt(best_d2);
  return best;
}

double ReferencePath::apply_speed_profile(double v_max, double a_max, double a_lat_max,
                                          double v_start, double v_end) {
  std::vector<double> s(points_.size());
  std::vector<double> k(points_.size());
  for (std::size_t i = 0; i < points_.size(); ++i) {
    s[i] = points_[i].s;
    k[i] = points_[i].curvature;
  }
  const SpeedProfile profile = compute_speed_profile(s, k, v_max, a_max, a_lat_max, v_start, v_end);
  for (std::size_t i = 0; i < points_.size(); ++i) {
    const double sign = points_[i].speed < 0.0 ? -1.0 : 1.0;
    points_[i].speed = sign * profile.v[i];
  }
  return profile.duration();
}

}  // namespace roboticscore::trajectory
