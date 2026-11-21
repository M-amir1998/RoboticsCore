#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "roboticscore/geometry/se3.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::geometry {
namespace {

using math::kPi;

Vec3 random_vec3(math::Random& rng, double scale) {
  return {rng.uniform(-scale, scale), rng.uniform(-scale, scale), rng.uniform(-scale, scale)};
}

SO3 random_rotation(math::Random& rng) {
  // Uniform rotation: normalised 4D Gaussian is uniform on S^3.
  const VecX g = rng.standard_normal(4);
  return SO3(Eigen::Quaterniond(g(0), g(1), g(2), g(3)));
}

SE3 random_pose(math::Random& rng) {
  return {random_rotation(rng), random_vec3(rng, 10.0)};
}

TEST(SO3, AxisRotations) {
  EXPECT_MATRIX_NEAR(SO3::rot_z(kPi / 2) * Vec3(1, 0, 0), Vec3(0, 1, 0), 1e-15);
  EXPECT_MATRIX_NEAR(SO3::rot_x(kPi / 2) * Vec3(0, 1, 0), Vec3(0, 0, 1), 1e-15);
  EXPECT_MATRIX_NEAR(SO3::rot_y(kPi / 2) * Vec3(0, 0, 1), Vec3(1, 0, 0), 1e-15);
}

TEST(SO3, QuaternionIsNormalized) {
  const SO3 r(Eigen::Quaterniond(2.0, 0.0, 0.0, 2.0));
  EXPECT_NEAR(r.quaternion().norm(), 1.0, 1e-15);
  EXPECT_MATRIX_NEAR(r.matrix(), SO3::rot_z(kPi / 2).matrix(), 1e-15);
  EXPECT_THROW(SO3(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)), std::invalid_argument);
}

TEST(SO3, FromMatrixProjectsOntoRotations) {
  const Mat3 r = SO3::from_euler_zyx(0.3, -0.4, 1.0).matrix();
  Mat3 noisy = r;
  noisy(0, 1) += 1e-3;
  noisy(2, 2) *= 1.01;
  const Mat3 projected = SO3::from_matrix(noisy).matrix();
  EXPECT_MATRIX_NEAR(projected * projected.transpose(), Mat3::Identity(), 1e-14);
  EXPECT_NEAR(projected.determinant(), 1.0, 1e-14);
  EXPECT_MATRIX_NEAR(projected, r, 2e-3);
  Mat3 reflection = Mat3::Identity();
  reflection(2, 2) = -1.0;
  EXPECT_THROW((void)SO3::from_matrix(reflection), std::invalid_argument);
  EXPECT_THROW((void)SO3::from_matrix(Mat3::Zero()), std::invalid_argument);
}

TEST(SO3, EulerRoundTrip) {
  const Vec3 ypr(0.4, -0.7, 2.5);
  const SO3 r = SO3::from_euler_zyx(ypr(0), ypr(1), ypr(2));
  EXPECT_MATRIX_NEAR(r.to_euler_zyx(), ypr, 1e-14);
  // Definition check: R = Rz * Ry * Rx.
  const Mat3 expected = (SO3::rot_z(ypr(0)) * SO3::rot_y(ypr(1)) * SO3::rot_x(ypr(2))).matrix();
  EXPECT_MATRIX_NEAR(r.matrix(), expected, 1e-15);
}

TEST(SO3, EulerGimbalLockReproducesRotation) {
  for (const double pitch : {kPi / 2, -kPi / 2}) {
    const SO3 r = SO3::from_euler_zyx(0.3, pitch, 0.9);
    const Vec3 ypr = r.to_euler_zyx();
    EXPECT_DOUBLE_EQ(ypr(2), 0.0);
    EXPECT_MATRIX_NEAR(SO3::from_euler_zyx(ypr(0), ypr(1), ypr(2)).matrix(), r.matrix(), 1e-12);
  }
}

TEST(SO3, ExpLogKnownValues) {
  const Vec3 omega(0.0, 0.0, kPi / 3);
  EXPECT_MATRIX_NEAR(SO3::exp(omega).matrix(), SO3::rot_z(kPi / 3).matrix(), 1e-15);
  EXPECT_MATRIX_NEAR(SO3::exp(omega).log(), omega, 1e-15);
  EXPECT_MATRIX_NEAR(SO3().log(), Vec3::Zero(), 0.0);
}

TEST(SO3, LogNearPiHasNormPi) {
  const Vec3 axis = Vec3(1.0, 2.0, -0.5).normalized();
  for (const double angle : {kPi - 1e-9, kPi, kPi + 1e-9}) {
    const Vec3 w = SO3::from_axis_angle(axis, angle).log();
    EXPECT_LE(w.norm(), kPi + 1e-12);
    EXPECT_NEAR(std::abs(w.normalized().dot(axis)), 1.0, 1e-9);
    EXPECT_MATRIX_NEAR(SO3::exp(w).matrix(), SO3::from_axis_angle(axis, angle).matrix(), 1e-9);
  }
}

TEST(SO3, SmallAngleSeriesAreContinuous) {
  const Vec3 axis = Vec3(0.2, -1.0, 0.4).normalized();
  for (const double angle : {1e-12, 1e-7, 9.99e-7, 1.01e-6, 1e-4}) {
    const Vec3 w = angle * axis;
    EXPECT_MATRIX_NEAR(SO3::exp(w).log(), w, 1e-18 + 1e-12 * angle);
    const Mat3 jl = SO3::left_jacobian(w);
    EXPECT_MATRIX_NEAR(jl * SO3::left_jacobian_inverse(w), Mat3::Identity(), 1e-14);
  }
}

TEST(SO3, HatIsCrossProduct) {
  const Vec3 a(1, 2, 3);
  const Vec3 b(-4, 0.5, 2);
  EXPECT_MATRIX_NEAR(SO3::hat(a) * b, a.cross(b), 1e-15);
  EXPECT_MATRIX_NEAR(SO3::vee(SO3::hat(a)), a, 0.0);
}

TEST(SE3, MatrixRoundTripAndValidation) {
  const SE3 t(SO3::from_euler_zyx(0.1, 0.2, 0.3), Vec3(1, 2, 3));
  EXPECT_MATRIX_NEAR(SE3::from_matrix(t.matrix()).matrix(), t.matrix(), 1e-14);
  Mat4 bad = t.matrix();
  bad(3, 1) = 1.0;
  EXPECT_THROW((void)SE3::from_matrix(bad), std::invalid_argument);
}

TEST(SE3, HatVeeRoundTrip) {
  Vec6 xi;
  xi << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6;
  EXPECT_MATRIX_NEAR(SE3::vee(SE3::hat(xi)), xi, 0.0);
}

TEST(LieProperty, SO3AndSE3Identities) {
  math::Random rng(testing::kPropertySeed + 1);
  for (int i = 0; i < 500; ++i) {
    SCOPED_TRACE(i);
    const SO3 r = random_rotation(rng);
    // R * R^-1 = I and R^T R = I
    ASSERT_MATRIX_NEAR((r * r.inverse()).matrix(), Mat3::Identity(), 1e-14);
    ASSERT_MATRIX_NEAR(r.matrix().transpose() * r.matrix(), Mat3::Identity(), 1e-14);
    ASSERT_MATRIX_NEAR(SO3::exp(r.log()).matrix(), r.matrix(), 1e-12);
    ASSERT_LE(r.log().norm(), kPi + 1e-12);

    const SE3 t = random_pose(rng);
    const SE3 u = random_pose(rng);
    ASSERT_MATRIX_NEAR((t * t.inverse()).matrix(), Mat4::Identity(), 1e-12);
    ASSERT_MATRIX_NEAR(SE3::exp(t.log()).matrix(), t.matrix(), 1e-10);
    ASSERT_MATRIX_NEAR((t * u).matrix(), t.matrix() * u.matrix(), 1e-12);
    const Vec3 p = random_vec3(rng, 5.0);
    ASSERT_MATRIX_NEAR(t * p, (t.matrix() * p.homogeneous()).head<3>(), 1e-12);

    Vec6 xi;
    xi << random_vec3(rng, 1.0), random_vec3(rng, 1.0);
    ASSERT_MATRIX_NEAR((t * SE3::exp(xi) * t.inverse()).matrix(),
                       SE3::exp(t.adjoint() * xi).matrix(), 1e-10);
    // Geodesic interpolation reaches its endpoints.
    ASSERT_MATRIX_NEAR(SE3::interpolate(t, u, 1.0).matrix(), u.matrix(), 1e-10);
    ASSERT_MATRIX_NEAR(SO3::interpolate(r, r, 0.3).matrix(), r.matrix(), 1e-12);
  }
}

}  // namespace
}  // namespace roboticscore::geometry
