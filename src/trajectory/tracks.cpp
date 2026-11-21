#include "roboticscore/trajectory/tracks.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"

namespace roboticscore::trajectory {
namespace {

// Integrates heading for a curvature function kappa(s), producing exact path samples.
ReferencePath integrate_curvature(double length, double spacing, double speed,
                                  double (*kappa)(double)) {
  std::vector<PathPoint> pts;
  const int n = static_cast<int>(std::ceil(length / spacing));
  const double ds = length / n;
  Vec2 p = Vec2::Zero();
  double heading = 0.0;
  for (int i = 0; i <= n; ++i) {
    const double s = ds * i;
    pts.push_back({s, p, math::wrap_to_pi(heading), kappa(s), speed});
    // Midpoint rule for the heading keeps positions second-order accurate.
    const double k_mid = kappa(s + 0.5 * ds);
    const double h_mid = heading + 0.5 * ds * kappa(s);
    p += ds * Vec2(std::cos(h_mid), std::sin(h_mid));
    heading += ds * k_mid;
  }
  return ReferencePath(std::move(pts));
}

double s_curve_curvature(double s) {
  // Two S-bends: heading returns to zero every 40 m, |kappa| <= 0.1 1/m (radius 10 m).
  return 0.1 * std::sin(2.0 * std::numbers::pi * s / 40.0);
}

double variable_curvature(double s) {
  // Alternating turns whose amplitude grows along the track (max |kappa| ~ 0.17 1/m).
  return 0.17 * std::sin(std::numbers::pi * s / 25.0) * (0.25 + 0.75 * s / 100.0);
}

// Straight segments joined by circular arcs of radius r at each corner.
std::vector<Vec2> rounded_polyline(const std::vector<Vec2>& corners, double r, double step) {
  std::vector<Vec2> out{corners.front()};
  for (std::size_t i = 1; i + 1 < corners.size(); ++i) {
    const Vec2 in = (corners[i] - corners[i - 1]).normalized();
    const Vec2 outd = (corners[i + 1] - corners[i]).normalized();
    const double turn =
        math::angle_difference(std::atan2(outd.y(), outd.x()), std::atan2(in.y(), in.x()));
    const double cut = r * std::tan(0.5 * std::abs(turn));
    const Vec2 a = corners[i] - cut * in;
    const Vec2 normal = turn > 0.0 ? Vec2(-in.y(), in.x()) : Vec2(in.y(), -in.x());
    const Vec2 center = a + r * normal;
    // Straight part up to the arc start.
    const Vec2 from = out.back();
    const int n_line = std::max(1, static_cast<int>(std::ceil((a - from).norm() / step)));
    for (int k = 1; k <= n_line; ++k)
      out.emplace_back(from + (a - from) * (double(k) / n_line));
    const double start_angle = std::atan2(a.y() - center.y(), a.x() - center.x());
    const int n_arc = std::max(2, static_cast<int>(std::ceil(r * std::abs(turn) / step)));
    for (int k = 1; k <= n_arc; ++k) {
      const double ang = start_angle + turn * k / n_arc;
      out.emplace_back(center + r * Vec2(std::cos(ang), std::sin(ang)));
    }
  }
  const Vec2 from = out.back();
  const Vec2& to = corners.back();
  const int n_line = std::max(1, static_cast<int>(std::ceil((to - from).norm() / step)));
  for (int k = 1; k <= n_line; ++k)
    out.emplace_back(from + (to - from) * (double(k) / n_line));
  return out;
}

}  // namespace

ReferencePath make_track(TrackType type, double speed, double spacing) {
  detail::require(spacing > 0.0, "make_track: spacing must be positive");
  switch (type) {
    case TrackType::kStraight: {
      std::vector<Vec2> w;
      for (int i = 0; i <= 12; ++i)
        w.emplace_back(5.0 * i, 0.0);
      return ReferencePath::from_waypoints(w, spacing, speed);
    }
    case TrackType::kCircle: {
      // Full loop of radius 15 m, counter-clockwise, starting at the origin heading +x.
      std::vector<PathPoint> pts;
      const double r = 15.0;
      const double length = 2.0 * std::numbers::pi * r;
      const int n = static_cast<int>(std::ceil(length / spacing));
      for (int i = 0; i <= n; ++i) {
        const double s = length * i / n;
        const double th = s / r;
        pts.push_back({s, Vec2(r * std::sin(th), r - r * std::cos(th)), math::wrap_to_pi(th),
                       1.0 / r, speed});
      }
      return ReferencePath(std::move(pts));
    }
    case TrackType::kSCurve:
      return integrate_curvature(80.0, spacing, speed, &s_curve_curvature);
    case TrackType::kSharpTurns: {
      const auto dense =
          rounded_polyline({{0, 0}, {25, 0}, {25, 20}, {50, 20}, {50, 0}, {75, 0}}, 4.0, 0.5);
      return ReferencePath::from_waypoints(dense, spacing, speed);
    }
    case TrackType::kVariableCurvature:
      return integrate_curvature(100.0, spacing, speed, &variable_curvature);
  }
  throw std::invalid_argument("make_track: unknown track type");
}

std::vector<std::string> track_names() {
  return {"straight", "circle", "s_curve", "sharp_turns", "variable_curvature"};
}

TrackType track_from_name(const std::string& name) {
  if (name == "straight") return TrackType::kStraight;
  if (name == "circle") return TrackType::kCircle;
  if (name == "s_curve") return TrackType::kSCurve;
  if (name == "sharp_turns") return TrackType::kSharpTurns;
  if (name == "variable_curvature") return TrackType::kVariableCurvature;
  throw std::invalid_argument("track_from_name: unknown track '" + name + "'");
}

TrajectoryMetrics evaluate_trajectory(const std::vector<double>& t,
                                      const std::vector<Vec2>& positions) {
  detail::require(t.size() == positions.size(), "evaluate_trajectory: size mismatch");
  detail::require(t.size() >= 4, "evaluate_trajectory: at least four samples are required");
  for (std::size_t i = 1; i < t.size(); ++i) {
    detail::require(t[i] > t[i - 1], "evaluate_trajectory: time must be strictly increasing");
  }
  const std::size_t n = t.size();
  TrajectoryMetrics m;
  m.duration = t.back() - t.front();
  std::vector<double> speed(n - 1);
  std::vector<double> tm(n - 1);  // segment mid-times
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double d = (positions[i + 1] - positions[i]).norm();
    m.length += d;
    speed[i] = d / (t[i + 1] - t[i]);
    tm[i] = 0.5 * (t[i] + t[i + 1]);
    m.max_speed = std::max(m.max_speed, speed[i]);
  }
  m.mean_speed = m.length / m.duration;
  std::vector<double> acc(n - 2);
  std::vector<double> ta(n - 2);
  double acc_ss = 0.0;
  for (std::size_t i = 0; i + 1 < speed.size(); ++i) {
    acc[i] = (speed[i + 1] - speed[i]) / (tm[i + 1] - tm[i]);
    ta[i] = 0.5 * (tm[i] + tm[i + 1]);
    m.max_abs_acceleration = std::max(m.max_abs_acceleration, std::abs(acc[i]));
    acc_ss += acc[i] * acc[i];
  }
  m.rms_acceleration = std::sqrt(acc_ss / static_cast<double>(acc.size()));
  double jerk_ss = 0.0;
  for (std::size_t i = 0; i + 1 < acc.size(); ++i) {
    const double j = (acc[i + 1] - acc[i]) / (ta[i + 1] - ta[i]);
    m.max_abs_jerk = std::max(m.max_abs_jerk, std::abs(j));
    jerk_ss += j * j;
  }
  m.rms_jerk = std::sqrt(jerk_ss / static_cast<double>(acc.size() - 1));
  double k_sum = 0.0;
  std::size_t k_count = 0;
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const Vec2& a = positions[i - 1];
    const Vec2& b = positions[i];
    const Vec2& c = positions[i + 1];
    const double ab = (b - a).norm();
    const double bc = (c - b).norm();
    const double ca = (a - c).norm();
    if (ab < 1e-12 || bc < 1e-12 || ca < 1e-12) continue;
    const double cross = (b - a).x() * (c - a).y() - (b - a).y() * (c - a).x();
    const double k = 2.0 * std::abs(cross) / (ab * bc * ca);  // Menger curvature
    m.max_abs_curvature = std::max(m.max_abs_curvature, k);
    k_sum += k;
    ++k_count;
  }
  m.mean_abs_curvature = k_count > 0 ? k_sum / static_cast<double>(k_count) : 0.0;
  return m;
}

}  // namespace roboticscore::trajectory
