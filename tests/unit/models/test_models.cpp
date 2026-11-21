#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/numerical_diff.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/models/bicycle.hpp"
#include "roboticscore/models/double_integrator.hpp"
#include "roboticscore/models/unicycle.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::models {
namespace {

using math::kPi;

TEST(Unicycle, StraightLineAndZeroMotion) {
  const Vec3 s(1.0, 2.0, kPi / 2);
  EXPECT_MATRIX_NEAR(Unicycle::step(s, {2.0, 0.0}, 0.5), Vec3(1.0, 3.0, kPi / 2), 1e-15);
  EXPECT_MATRIX_NEAR(Unicycle::step(s, {0.0, 0.0}, 0.5), s, 0.0);
  EXPECT_MATRIX_NEAR(Unicycle::step(s, {2.0, 1.0}, 0.0), s, 0.0);
  EXPECT_THROW((void)Unicycle::step(s, {1.0, 0.0}, -0.1), std::invalid_argument);
}

TEST(Unicycle, FullCircleReturnsToStart) {
  // v = 1, omega = 1 -> radius 1, period 2*pi; integrate in many small exact steps.
  Vec3 s = Vec3::Zero();
  const int steps = 1000;
  for (int i = 0; i < steps; ++i) {
    s = Unicycle::step(s, {1.0, 1.0}, 2.0 * kPi / steps);
  }
  EXPECT_MATRIX_NEAR(s, Vec3::Zero(), 1e-10);
  // A quarter circle in one exact step lands on (1, 1, pi/2).
  EXPECT_MATRIX_NEAR(Unicycle::step(Vec3::Zero(), {1.0, 1.0}, kPi / 2), Vec3(1, 1, kPi / 2), 1e-14);
}

TEST(Unicycle, JacobiansMatchNumericalIncludingZeroOmega) {
  math::Random rng(testing::kPropertySeed);
  for (int i = 0; i < 200; ++i) {
    const Vec3 s(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-3, 3));
    const double v = rng.uniform(-2, 2);
    const double omega = (i % 4 == 0) ? 0.0 : rng.uniform(-2, 2) * std::pow(10.0, -(i % 7));
    const double dt = 0.1;
    const UnicycleJacobians j = Unicycle::jacobians(s, {v, omega}, dt);
    const auto wrap_diff = [](const VecX& a, const VecX& b) -> VecX {
      VecX d = a - b;
      d(2) = math::wrap_to_pi(d(2));
      return d;
    };
    const auto fx = [&](const VecX& x) -> VecX {
      return Unicycle::step(Vec3(x), {v, omega}, dt);
    };
    const auto fu = [&](const VecX& u) -> VecX {
      return Unicycle::step(s, {u(0), u(1)}, dt);
    };
    ASSERT_MATRIX_NEAR(j.wrt_state, math::numerical_jacobian(fx, VecX(s), wrap_diff), 1e-8);
    ASSERT_MATRIX_NEAR(j.wrt_input, math::numerical_jacobian(fu, Vec2(v, omega), wrap_diff), 1e-8);
  }
}

TEST(DifferentialDrive, ForwardInverseRoundTrip) {
  const DifferentialDrive dd({0.1, 0.5, 10.0});
  const UnicycleInput u{0.8, 0.6};
  const WheelSpeeds w = dd.inverse(u);
  const UnicycleInput back = dd.forward(w);
  EXPECT_NEAR(back.v, u.v, 1e-15);
  EXPECT_NEAR(back.omega, u.omega, 1e-15);
  // Turning in place: wheels spin in opposite directions.
  const WheelSpeeds spin = dd.inverse({0.0, 1.0});
  EXPECT_NEAR(spin.left, -spin.right, 1e-15);
  EXPECT_THROW(DifferentialDrive({0.0, 0.5, 10.0}), std::invalid_argument);
}

TEST(DifferentialDrive, SaturationPreservesCurvature) {
  const DifferentialDrive dd({0.1, 0.5, 10.0});
  const UnicycleInput fast{3.0, 2.0};
  const UnicycleInput sat = dd.saturate(fast);
  const WheelSpeeds w = dd.inverse(sat);
  EXPECT_NEAR(std::max(std::abs(w.left), std::abs(w.right)), 10.0, 1e-12);
  EXPECT_NEAR(sat.omega / sat.v, fast.omega / fast.v, 1e-12);
  const UnicycleInput slow{0.1, 0.1};
  EXPECT_DOUBLE_EQ(dd.saturate(slow).v, slow.v);
}

TEST(KinematicBicycle, ConstantSteeringFollowsCircleOfExpectedRadius) {
  const KinematicBicycle car({2.5, 0.5, 20.0, 5.0});
  const double delta = 0.3;
  const double radius = 2.5 / std::tan(delta);
  EXPECT_NEAR(1.0 / car.curvature(delta), radius, 1e-12);
  BicycleState x(0.0, 0.0, 0.0, 2.0);
  for (int i = 0; i < 2000; ++i) {
    x = car.step(x, BicycleInput(0.0, delta), 0.005);
    // The turning centre is (0, R); every state stays on the circle.
    ASSERT_NEAR(std::hypot(x(0), x(1) - radius), radius, 1e-6);
  }
  EXPECT_NEAR(car.steering_for_curvature(car.curvature(delta)), delta, 1e-15);
}

TEST(KinematicBicycle, StepClampsInputsAndSpeed) {
  const KinematicBicycle car({2.5, 0.5, 3.0, 2.0});
  const BicycleState x(0, 0, 0, 2.9);
  const BicycleState next = car.step(x, BicycleInput(100.0, 2.0), 1.0);
  EXPECT_DOUBLE_EQ(next(3), 3.0);  // speed limit
  const BicycleState ref = car.step(x, BicycleInput(2.0, 0.5), 1.0);
  EXPECT_MATRIX_NEAR(next, ref, 1e-15);  // inputs clamped to (max_accel, max_steer)
}

TEST(KinematicBicycle, StepPoseIsExactArc) {
  const KinematicBicycle car({2.0, 0.6, 20.0, 5.0});
  const double delta = 0.4;
  const double r = 1.0 / car.curvature(delta);
  // Quarter turn: arc length r*pi/2.
  const geometry::Pose2 p = car.step_pose({0, 0, 0}, 1.0, delta, r * kPi / 2);
  EXPECT_MATRIX_NEAR(p.vector(), Vec3(r, r, kPi / 2), 1e-12);
  EXPECT_NEAR(car.min_turning_radius(), 2.0 / std::tan(0.6), 1e-15);
}

TEST(KinematicBicycle, LinearizationMatchesNumericalJacobians) {
  const KinematicBicycle car({2.7, 0.6, 20.0, 5.0});
  const BicycleState x(1.0, -2.0, 0.7, 4.0);
  const BicycleInput u(0.5, 0.2);
  const double dt = 0.05;
  const BicycleLinearization lin = car.linearize(x, u, dt);
  const auto f_x = [&](const VecX& s) -> VecX {
    return car.derivative(Vec4(s), u);
  };
  const auto f_u = [&](const VecX& in) -> VecX {
    return car.derivative(x, Vec2(in));
  };
  const MatX ac = math::numerical_jacobian(f_x, VecX(x));
  const MatX bc = math::numerical_jacobian(f_u, VecX(u));
  EXPECT_MATRIX_NEAR(lin.a, MatX(Mat4::Identity() + dt * ac), 1e-9);
  EXPECT_MATRIX_NEAR(lin.b, MatX(dt * bc), 1e-9);
  // Affine model is exact (to first order) at the linearisation point.
  const Vec4 euler = x + dt * car.derivative(x, u);
  EXPECT_MATRIX_NEAR(lin.a * x + lin.b * u + lin.c, euler, 1e-12);
}

TEST(Ackermann, WheelAxesMeetOnRearAxle) {
  const AckermannGeometry g{2.7, 1.6};
  for (const double delta : {0.1, 0.4, -0.3}) {
    const Vec2 w = g.wheel_angles(delta);
    // Ackermann condition: cot(outer) - cot(inner) = track / wheelbase.
    const double inner = delta > 0 ? w(0) : w(1);
    const double outer = delta > 0 ? w(1) : w(0);
    EXPECT_GT(std::abs(inner), std::abs(outer));
    EXPECT_NEAR(std::abs(1.0 / std::tan(outer) - 1.0 / std::tan(inner)), 1.6 / 2.7, 1e-12);
  }
  EXPECT_MATRIX_NEAR(g.wheel_angles(0.0), Vec2::Zero(), 0.0);
}

TEST(VehicleFootprint, PolygonPlacement) {
  const VehicleFootprint fp{4.0, 2.0, 1.0};
  const geometry::Pose2 pose(10.0, 5.0, kPi / 2);
  EXPECT_MATRIX_NEAR(fp.center(pose), Vec2(10.0, 6.0), 1e-14);
  const auto poly = fp.polygon(pose);
  EXPECT_NEAR(poly.area(), 8.0, 1e-12);
  EXPECT_NEAR(poly.bounding_box().min.y(), 4.0, 1e-12);  // rear bumper 1 m behind the axle
  EXPECT_NEAR(poly.bounding_box().max.y(), 8.0, 1e-12);
  EXPECT_NEAR(fp.bounding_radius(), std::sqrt(5.0), 1e-15);
}

TEST(DoubleIntegrator, ExactDiscretization) {
  const DoubleIntegrator di(2, 0.1);
  VecX x = VecX::Zero(4);
  const VecX a = Vec2(1.0, -2.0);
  for (int i = 0; i < 10; ++i) {
    x = di.step(x, a);
  }
  // After 1 s of constant acceleration: p = a t^2 / 2, v = a t.
  EXPECT_MATRIX_NEAR(x, Vec4(0.5, -1.0, 1.0, -2.0), 1e-14);
  const MatX q = di.process_noise(0.5);
  EXPECT_MATRIX_NEAR(q, q.transpose(), 0.0);
  EXPECT_GT(q.ldlt().vectorD().minCoeff(), 0.0);
  EXPECT_EQ(di.position_measurement().rows(), 2);
  EXPECT_THROW(DoubleIntegrator(0, 0.1), std::invalid_argument);
}

}  // namespace
}  // namespace roboticscore::models
