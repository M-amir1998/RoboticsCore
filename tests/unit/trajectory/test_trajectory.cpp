#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/math/angles.hpp"
#include "roboticscore/trajectory/polynomial.hpp"
#include "roboticscore/trajectory/reference_path.hpp"
#include "roboticscore/trajectory/spline.hpp"
#include "roboticscore/trajectory/time_parameterization.hpp"
#include "roboticscore/trajectory/tracks.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::trajectory {
namespace {

TEST(Polynomial, CubicBoundaryConditions) {
  const Polynomial p = cubic_trajectory(1.0, 0.5, 4.0, -1.0, 2.0);
  EXPECT_NEAR(p.position(0.0), 1.0, 1e-12);
  EXPECT_NEAR(p.velocity(0.0), 0.5, 1e-12);
  EXPECT_NEAR(p.position(2.0), 4.0, 1e-12);
  EXPECT_NEAR(p.velocity(2.0), -1.0, 1e-12);
  EXPECT_THROW((void)cubic_trajectory(0, 0, 1, 0, 0.0), std::invalid_argument);
}

TEST(Polynomial, QuinticBoundaryConditionsAndMinimumJerkSymmetry) {
  const Polynomial p = quintic_trajectory(0.0, 1.0, -0.5, 3.0, 0.2, 0.3, 1.7);
  EXPECT_NEAR(p.position(0.0), 0.0, 1e-12);
  EXPECT_NEAR(p.velocity(0.0), 1.0, 1e-12);
  EXPECT_NEAR(p.acceleration(0.0), -0.5, 1e-12);
  EXPECT_NEAR(p.position(1.7), 3.0, 1e-10);
  EXPECT_NEAR(p.velocity(1.7), 0.2, 1e-10);
  EXPECT_NEAR(p.acceleration(1.7), 0.3, 1e-10);
  // Rest-to-rest minimum-jerk profile: 10 tau^3 - 15 tau^4 + 6 tau^5.
  const Polynomial mj = quintic_trajectory(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0);
  for (const double tau : {0.25, 0.5, 0.8}) {
    EXPECT_NEAR(mj.position(2.0 * tau),
                10 * std::pow(tau, 3) - 15 * std::pow(tau, 4) + 6 * std::pow(tau, 5), 1e-12);
  }
  EXPECT_NEAR(mj.position(1.0), 0.5, 1e-12);
  EXPECT_NEAR(linear_trajectory(2.0, 6.0, 4.0).velocity(1.3), 1.0, 1e-15);
}

TEST(TrapezoidalProfile, TrapezoidalAndTriangularCases) {
  const TrapezoidalProfile trap(0.0, 10.0, 2.0, 1.0);
  EXPECT_NEAR(trap.duration(), 7.0, 1e-12);  // 2 s accel + 3 s cruise + 2 s decel
  EXPECT_NEAR(trap.position(trap.duration()), 10.0, 1e-12);
  EXPECT_NEAR(trap.velocity(3.0), 2.0, 1e-12);
  const TrapezoidalProfile tri(5.0, 4.0, 2.0, 1.0);  // backwards, too short to cruise
  EXPECT_NEAR(tri.peak_velocity(), 1.0, 1e-12);
  EXPECT_NEAR(tri.duration(), 2.0, 1e-12);
  EXPECT_NEAR(tri.position(1.0), 4.5, 1e-12);
  EXPECT_LT(tri.velocity(0.5), 0.0);
  for (double t = 0.0; t <= trap.duration(); t += 0.01) {
    ASSERT_LE(std::abs(trap.velocity(t)), 2.0 + 1e-12);
    ASSERT_LE(std::abs(trap.acceleration(t)), 1.0 + 1e-12);
  }
}

TEST(CubicSpline, InterpolatesAndNaturalEnds) {
  const std::vector<double> x = {0.0, 1.0, 2.5, 4.0};
  const std::vector<double> y = {1.0, -1.0, 2.0, 0.5};
  const CubicSpline s(x, y);
  for (std::size_t i = 0; i < x.size(); ++i) {
    EXPECT_NEAR(s(x[i]), y[i], 1e-12);
  }
  EXPECT_NEAR(s.derivative(0.0, 2), 0.0, 1e-12);
  EXPECT_NEAR(s.derivative(4.0, 2), 0.0, 1e-12);
  // C1 and C2 continuity at an interior knot.
  EXPECT_NEAR(s.derivative(1.0 - 1e-9, 1), s.derivative(1.0 + 1e-9, 1), 1e-6);
  EXPECT_NEAR(s.derivative(1.0 - 1e-9, 2), s.derivative(1.0 + 1e-9, 2), 1e-6);
  EXPECT_THROW(CubicSpline({0.0, 0.0}, {1.0, 2.0}), std::invalid_argument);
}

TEST(CubicSpline, ClampedSplineReproducesCubic) {
  const auto f = [](double t) { return 0.3 * t * t * t - t * t + 2.0 * t - 1.0; };
  const auto df = [](double t) { return 0.9 * t * t - 2.0 * t + 2.0; };
  std::vector<double> x;
  std::vector<double> y;
  for (int i = 0; i <= 6; ++i) {
    x.push_back(0.7 * i);
    y.push_back(f(0.7 * i));
  }
  const CubicSpline s(x, y, df(0.0), df(4.2));
  for (double t = 0.0; t <= 4.2; t += 0.13) {
    EXPECT_NEAR(s(t), f(t), 1e-10);
  }
}

TEST(CubicSpline2D, CircleHasConstantCurvature) {
  std::vector<Vec2> w;
  const double r = 10.0;
  for (int i = 0; i <= 36; ++i) {
    const double a = 2.0 * std::numbers::pi * i / 36.0;
    w.emplace_back(r * std::cos(a), r * std::sin(a));
  }
  const CubicSpline2D spline(w);
  EXPECT_NEAR(spline.length(), 2.0 * std::numbers::pi * r, 0.1);
  for (double s = 0.2 * spline.length(); s < 0.8 * spline.length(); s += 1.0) {
    EXPECT_NEAR(spline.curvature(s), 1.0 / r, 0.002);
    EXPECT_NEAR(spline.position(s).norm(), r, 0.01);
  }
}

TEST(BSpline, ClampedEndpointsDerivativeAndConvexHull) {
  const std::vector<Vec2> ctrl = {{0, 0}, {1, 2}, {3, 3}, {4, 0}, {6, 1}};
  const BSpline b(ctrl, 3);
  EXPECT_MATRIX_NEAR(b.evaluate(0.0), ctrl.front(), 1e-12);
  EXPECT_MATRIX_NEAR(b.evaluate(1.0), ctrl.back(), 1e-12);
  for (double u = 0.05; u < 1.0; u += 0.1) {
    const Vec2 p = b.evaluate(u);
    EXPECT_GE(p.x(), -1e-12);
    EXPECT_LE(p.x(), 6.0 + 1e-12);
    EXPECT_GE(p.y(), -1e-12);
    EXPECT_LE(p.y(), 3.0 + 1e-12);
    const Vec2 fd = (b.evaluate(u + 1e-6) - b.evaluate(u - 1e-6)) / 2e-6;
    EXPECT_MATRIX_NEAR(b.derivative(u), fd, 1e-5);
  }
  // Degree 1 is the control polygon.
  const BSpline line({{0, 0}, {2, 0}, {2, 2}}, 1);
  EXPECT_MATRIX_NEAR(line.evaluate(0.25), Vec2(1, 0), 1e-12);
  EXPECT_THROW(BSpline({{0, 0}, {1, 1}}, 3), std::invalid_argument);
}

TEST(Bezier, EvaluationDerivativeAndSplit) {
  const BezierCurve c({{0, 0}, {1, 3}, {3, 3}, {4, 0}});
  EXPECT_MATRIX_NEAR(c.evaluate(0.0), Vec2(0, 0), 0.0);
  EXPECT_MATRIX_NEAR(c.evaluate(1.0), Vec2(4, 0), 1e-15);
  EXPECT_MATRIX_NEAR(c.evaluate(0.5), Vec2(2, 2.25), 1e-15);
  const BezierCurve d = c.derivative();
  EXPECT_EQ(d.degree(), 2);
  for (const double t : {0.1, 0.4, 0.9}) {
    const Vec2 fd = (c.evaluate(t + 1e-6) - c.evaluate(t - 1e-6)) / 2e-6;
    EXPECT_MATRIX_NEAR(d.evaluate(t), fd, 1e-6);
  }
  const auto [left, right] = c.split(0.3);
  EXPECT_MATRIX_NEAR(left.evaluate(0.5), c.evaluate(0.15), 1e-12);
  EXPECT_MATRIX_NEAR(right.evaluate(0.5), c.evaluate(0.65), 1e-12);
}

TEST(SpeedProfile, RespectsAllLimits) {
  std::vector<double> s;
  std::vector<double> k;
  for (int i = 0; i <= 200; ++i) {
    s.push_back(0.5 * i);
    k.push_back(i > 80 && i < 120 ? 0.2 : 0.0);  // a tight bend in the middle
  }
  const SpeedProfile p = compute_speed_profile(s, k, 5.0, 1.0, 2.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(p.v.front(), 0.0);
  EXPECT_DOUBLE_EQ(p.v.back(), 0.0);
  for (std::size_t i = 0; i < s.size(); ++i) {
    ASSERT_LE(p.v[i], 5.0 + 1e-12);
    ASSERT_LE(p.v[i] * p.v[i] * std::abs(k[i]), 2.0 + 1e-9);
    if (i + 1 < s.size()) {
      ASSERT_LE(std::abs(p.a[i]), 1.0 + 1e-9);
      ASSERT_GT(p.t[i + 1], p.t[i]);
    }
  }
}

TEST(ReferencePath, ProjectionAndInterpolation) {
  const ReferencePath path = ReferencePath::from_waypoints({{0, 0}, {10, 0}}, 0.5, 2.0);
  EXPECT_NEAR(path.length(), 10.0, 1e-9);
  const PathProjection left = path.project({4.2, 1.5});
  EXPECT_NEAR(left.s, 4.2, 1e-9);
  EXPECT_NEAR(left.cross_track, 1.5, 1e-9);
  EXPECT_NEAR(path.project({4.2, -0.5}).cross_track, -0.5, 1e-9);
  EXPECT_MATRIX_NEAR(path.at(7.25).position, Vec2(7.25, 0), 1e-9);
  EXPECT_DOUBLE_EQ(path.at(100.0).s, path.length());
}

TEST(Tracks, AllTracksAreWellFormed) {
  for (const auto& name : track_names()) {
    const ReferencePath p = make_track(track_from_name(name), 3.0);
    SCOPED_TRACE(name);
    EXPECT_MATRIX_NEAR(p[0].position, Vec2::Zero(), 1e-9);
    EXPECT_NEAR(p[0].heading, 0.0, 0.05);
    EXPECT_GT(p.length(), 50.0);
    for (std::size_t i = 1; i < p.size(); ++i) {
      ASSERT_LT((p[i].position - p[i - 1].position).norm(), 0.5);
    }
  }
  const ReferencePath circle = make_track(TrackType::kCircle, 3.0);
  EXPECT_NEAR(circle[circle.size() / 2].curvature, 1.0 / 15.0, 1e-12);
  EXPECT_THROW((void)track_from_name("oval"), std::invalid_argument);
}

TEST(TrajectoryMetrics, StraightLineAndCircle) {
  std::vector<double> t;
  std::vector<Vec2> line;
  std::vector<Vec2> circle;
  for (int i = 0; i <= 100; ++i) {
    t.push_back(0.1 * i);
    line.emplace_back(2.0 * 0.1 * i, 1.0);
    const double a = 0.02 * i;
    circle.emplace_back(5.0 * std::cos(a), 5.0 * std::sin(a));
  }
  const TrajectoryMetrics ml = evaluate_trajectory(t, line);
  EXPECT_NEAR(ml.length, 20.0, 1e-9);
  EXPECT_NEAR(ml.mean_speed, 2.0, 1e-9);
  EXPECT_NEAR(ml.max_abs_acceleration, 0.0, 1e-9);
  EXPECT_NEAR(ml.max_abs_jerk, 0.0, 1e-6);
  EXPECT_NEAR(ml.max_abs_curvature, 0.0, 1e-9);
  const TrajectoryMetrics mc = evaluate_trajectory(t, circle);
  EXPECT_NEAR(mc.mean_abs_curvature, 0.2, 1e-6);
  EXPECT_THROW((void)evaluate_trajectory({0, 1, 2}, {{0, 0}, {1, 0}, {2, 0}}),
               std::invalid_argument);
}

}  // namespace
}  // namespace roboticscore::trajectory
