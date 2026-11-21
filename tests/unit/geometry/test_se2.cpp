#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::geometry {
namespace {

using math::kPi;

SE2 random_pose(math::Random& rng) {
  return {rng.uniform(-10, 10), rng.uniform(-10, 10), rng.uniform(-kPi, kPi)};
}

TEST(SO2, CompositionAddsAngles) {
  const SO2 a(0.3);
  const SO2 b(-1.2);
  EXPECT_NEAR((a * b).angle(), -0.9, 1e-15);
  EXPECT_NEAR((a * a.inverse()).angle(), 0.0, 1e-15);
}

TEST(SO2, RotatesPoints) {
  const SO2 r(kPi / 2);
  EXPECT_MATRIX_NEAR(r * Vec2(1, 0), Vec2(0, 1), 1e-15);
  EXPECT_MATRIX_NEAR(r.matrix() * Vec2(2, 3), r * Vec2(2, 3), 1e-15);
}

TEST(SO2, FromMatrixNormalizesAndRejectsZero) {
  Mat2 scaled = 3.0 * SO2(0.7).matrix();
  EXPECT_NEAR(SO2::from_matrix(scaled).angle(), 0.7, 1e-15);
  EXPECT_THROW((void)SO2::from_matrix(Mat2::Zero()), std::invalid_argument);
}

TEST(SO2, HatVeeRoundTrip) {
  EXPECT_DOUBLE_EQ(SO2::vee(SO2::hat(0.42)), 0.42);
  EXPECT_NEAR(SO2(2.5).normalized().angle(), 2.5, 1e-15);
}

TEST(SE2, ComposeKnownPoses) {
  const SE2 a(1.0, 0.0, kPi / 2);
  const SE2 b(1.0, 0.0, 0.0);
  const SE2 c = a * b;  // move 1 m forward in a's frame (which faces +y)
  EXPECT_NEAR(c.x(), 1.0, 1e-15);
  EXPECT_NEAR(c.y(), 1.0, 1e-15);
  EXPECT_NEAR(c.theta(), kPi / 2, 1e-15);
}

TEST(SE2, InverseAndBetween) {
  const SE2 a(1.0, 2.0, 0.5);
  const SE2 identity = a * a.inverse();
  EXPECT_MATRIX_NEAR(identity.matrix(), Mat3::Identity(), 1e-14);
  const SE2 b(-3.0, 4.0, -2.0);
  EXPECT_MATRIX_NEAR((a * a.between(b)).matrix(), b.matrix(), 1e-14);
}

TEST(SE2, MatrixRoundTripAndValidation) {
  const SE2 a(1.0, -2.0, 2.9);
  EXPECT_MATRIX_NEAR(SE2::from_matrix(a.matrix()).vector(), a.vector(), 1e-14);
  Mat3 bad = a.matrix();
  bad(2, 0) = 0.1;
  EXPECT_THROW((void)SE2::from_matrix(bad), std::invalid_argument);
}

TEST(SE2, ExpOfPureRotationAndTranslation) {
  EXPECT_MATRIX_NEAR(SE2::exp(Vec3(1.0, 2.0, 0.0)).vector(), Vec3(1.0, 2.0, 0.0), 1e-15);
  // A unit-speed arc of angle pi/2 with unit radius ends at (1, 1).
  const SE2 arc = SE2::exp(Vec3(kPi / 2, 0.0, kPi / 2));
  EXPECT_MATRIX_NEAR(arc.vector(), Vec3(1.0, 1.0, kPi / 2), 1e-14);
}

TEST(SE2, HatVeeRoundTrip) {
  const Vec3 xi(0.3, -0.2, 1.1);
  EXPECT_MATRIX_NEAR(SE2::vee(SE2::hat(xi)), xi, 1e-15);
}

TEST(SE2, InterpolateEndpointsAndMidpoint) {
  const SE2 a(0.0, 0.0, 0.0);
  const SE2 b(2.0, 0.0, 0.0);
  EXPECT_MATRIX_NEAR(SE2::interpolate(a, b, 0.5).vector(), Vec3(1.0, 0.0, 0.0), 1e-15);
  EXPECT_MATRIX_NEAR(SE2::interpolate(a, b, 1.0).vector(), b.vector(), 1e-15);
}

TEST(SE2Property, ExpLogInverseAndAdjoint) {
  math::Random rng(testing::kPropertySeed);
  for (int i = 0; i < 500; ++i) {
    const SE2 t = random_pose(rng);
    SCOPED_TRACE(i);
    // exp(log(T)) = T
    ASSERT_MATRIX_NEAR(SE2::exp(t.log()).matrix(), t.matrix(), 1e-12);
    // T * T^-1 = I
    ASSERT_MATRIX_NEAR((t * t.inverse()).matrix(), Mat3::Identity(), 1e-12);
    // Matrix representation is a homomorphism.
    const SE2 u = random_pose(rng);
    ASSERT_MATRIX_NEAR((t * u).matrix(), t.matrix() * u.matrix(), 1e-12);
    // Associativity.
    const SE2 w = random_pose(rng);
    ASSERT_MATRIX_NEAR(((t * u) * w).matrix(), (t * (u * w)).matrix(), 1e-12);
    // Adjoint: T exp(xi) T^-1 = exp(Ad xi).
    const Vec3 xi(rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1));
    ASSERT_MATRIX_NEAR((t * SE2::exp(xi) * t.inverse()).matrix(),
                       SE2::exp(t.adjoint() * xi).matrix(), 1e-12);
    // Point transform agrees with the homogeneous matrix.
    const Vec2 p(rng.uniform(-5, 5), rng.uniform(-5, 5));
    ASSERT_MATRIX_NEAR(t * p, (t.matrix() * p.homogeneous()).head<2>(), 1e-12);
  }
}

}  // namespace
}  // namespace roboticscore::geometry
