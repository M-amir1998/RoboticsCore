// Numerical behaviour of the Lie-group maps at their delicate points: zero motion, tiny
// rotations around the series/closed-form switch, and headings at the +-pi discontinuity.

#include <cmath>

#include <gtest/gtest.h>

#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/geometry/se3.hpp"
#include "roboticscore/math/angles.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::geometry {
namespace {

using math::kPi;

TEST(LieNumerics, ZeroMotionIsExactIdentity) {
  EXPECT_MATRIX_NEAR(SE2::exp(Vec3::Zero()).matrix(), Mat3::Identity(), 0.0);
  EXPECT_MATRIX_NEAR(SE3::exp(Vec6::Zero()).matrix(), Mat4::Identity(), 0.0);
  EXPECT_MATRIX_NEAR(SE2().log(), Vec3::Zero(), 0.0);
  EXPECT_MATRIX_NEAR(SE3().log(), Vec6::Zero(), 0.0);
}

TEST(LieNumerics, Se2ExpLogAcrossSeriesThreshold) {
  for (const double w : {1e-15, 1e-9, 9.9e-7, 1.0e-6, 1.1e-6, 1e-3}) {
    const Vec3 xi(0.7, -0.4, w);
    EXPECT_MATRIX_NEAR(SE2::exp(xi).log(), xi, 1e-15);
  }
}

TEST(LieNumerics, Se3ExpLogAcrossSeriesThresholds) {
  const Vec3 axis = Vec3(0.3, 0.9, -0.2).normalized();
  for (const double w : {1e-15, 1e-9, 1e-6, 9.9e-3, 1.01e-2, 0.5, 3.0}) {
    Vec6 xi;
    xi << 1.0, -2.0, 0.5, w * axis;
    EXPECT_MATRIX_NEAR(SE3::exp(xi).log(), xi, 1e-13);
  }
}

TEST(LieNumerics, Se2HeadingAtDiscontinuity) {
  // Two poses whose headings straddle +-pi are close on the manifold.
  const SE2 a(0.0, 0.0, kPi - 1e-9);
  const SE2 b(0.0, 0.0, -kPi + 1e-9);
  const Vec3 d = a.between(b).log();
  EXPECT_NEAR(d(2), 2e-9, 1e-15);
  EXPECT_NEAR(d.head<2>().norm(), 0.0, 1e-15);
  // Interpolation passes through pi rather than through 0.
  EXPECT_NEAR(std::abs(SE2::interpolate(a, b, 0.5).theta()), kPi, 1e-12);
}

TEST(LieNumerics, Se2LogOfHalfTurnIsFinite) {
  const SE2 t(1.0, 2.0, kPi);
  const Vec3 xi = t.log();
  EXPECT_TRUE(xi.allFinite());
  EXPECT_MATRIX_NEAR(SE2::exp(xi).matrix(), t.matrix(), 1e-14);
}

TEST(LieNumerics, RepeatedCompositionStaysOrthonormal) {
  // Integrating a small rotation 1e6 times must not let the rotation drift off SO(3).
  const SO3 step = SO3::exp(Vec3(1e-3, -2e-3, 0.5e-3));
  SO3 r;
  for (int i = 0; i < 1000000; ++i) {
    r = r * step;
  }
  const Mat3 m = r.matrix();
  EXPECT_MATRIX_NEAR(m.transpose() * m, Mat3::Identity(), 1e-13);
}

}  // namespace
}  // namespace roboticscore::geometry
