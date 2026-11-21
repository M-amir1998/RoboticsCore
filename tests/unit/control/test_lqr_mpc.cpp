#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "roboticscore/control/geometric_trackers.hpp"
#include "roboticscore/control/lqr.hpp"
#include "roboticscore/control/mpc.hpp"
#include "roboticscore/math/linear_algebra.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/models/double_integrator.hpp"
#include "roboticscore/trajectory/tracks.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::control {
namespace {

MatX value_iteration_dare(const MatX& a, const MatX& b, const MatX& q, const MatX& r) {
  MatX p = q;
  for (int i = 0; i < 20000; ++i) {
    const MatX k = (r + b.transpose() * p * b).ldlt().solve(b.transpose() * p * a);
    p = a.transpose() * p * (a - b * k) + q;
  }
  return p;
}

TEST(Dare, ScalarClosedForm) {
  // x+ = a x + b u: P solves b^2 P^2 + (r (1 - a^2) - q b^2) P - q r = 0.
  const double a = 1.2;
  const double b = 0.5;
  const double q = 2.0;
  const double r = 0.7;
  const double qa = b * b;
  const double qb = r * (1 - a * a) - q * b * b;
  const double qc = -q * r;
  const double p_expected = (-qb + std::sqrt(qb * qb - 4 * qa * qc)) / (2 * qa);
  const DareResult res = solve_dare(MatX::Constant(1, 1, a), MatX::Constant(1, 1, b),
                                    MatX::Constant(1, 1, q), MatX::Constant(1, 1, r));
  ASSERT_TRUE(res.converged);
  EXPECT_NEAR(res.p(0, 0), p_expected, 1e-10);
  EXPECT_LT(std::abs(a - b * res.k(0, 0)), 1.0);
}

TEST(Dare, MatchesValueIterationAndStabilises) {
  const models::DoubleIntegrator di(2, 0.1);
  const MatX q = MatX::Identity(4, 4);
  const MatX r = 0.1 * MatX::Identity(2, 2);
  const DareResult res = solve_dare(di.a(), di.b(), q, r);
  ASSERT_TRUE(res.converged);
  EXPECT_LT(res.residual, 1e-10);
  EXPECT_LT(res.iterations, 30);  // quadratic convergence
  EXPECT_MATRIX_NEAR(res.p, value_iteration_dare(di.a(), di.b(), q, r), 1e-6);
  const Eigen::EigenSolver<MatX> eig(di.a() - di.b() * res.k);
  EXPECT_LT(eig.eigenvalues().cwiseAbs().maxCoeff(), 1.0);
}

TEST(DareProperty, RandomStabilisableSystems) {
  math::Random rng(testing::kPropertySeed);
  int solved = 0;
  for (int trial = 0; trial < 30; ++trial) {
    MatX a(3, 3);
    MatX b(3, 1);
    for (Index i = 0; i < 3; ++i) {
      b(i, 0) = rng.normal();
      for (Index j = 0; j < 3; ++j)
        a(i, j) = rng.normal() * 0.8;
    }
    const DareResult res = solve_dare(a, b, MatX::Identity(3, 3), MatX::Identity(1, 1));
    if (!res.converged) continue;  // random (A, B) may be (nearly) uncontrollable
    ++solved;
    EXPECT_LT(res.residual, 1e-8);
    EXPECT_TRUE(math::is_positive_definite(res.p));
  }
  EXPECT_GT(solved, 20);
}

TEST(Dare, ReportsMissingStabilisingSolutionAndBadInput) {
  // Unstable mode that the input cannot reach.
  MatX a(2, 2);
  a << 2.0, 0.0, 0.0, 0.5;
  MatX b(2, 1);
  b << 0.0, 1.0;
  EXPECT_FALSE(solve_dare(a, b, MatX::Identity(2, 2), MatX::Identity(1, 1)).converged);
  EXPECT_THROW((void)solve_dare(a, b, MatX::Identity(2, 2), MatX::Zero(1, 1)),
               std::invalid_argument);
}

TEST(FiniteHorizonLqr, ConvergesToInfiniteHorizonGain) {
  const models::DoubleIntegrator di(1, 0.1);
  const MatX q = MatX::Identity(2, 2);
  const MatX r = MatX::Identity(1, 1);
  const auto gains = finite_horizon_lqr(di.a(), di.b(), q, r, MatX::Zero(2, 2), 500);
  EXPECT_MATRIX_NEAR(gains.front(), solve_dare(di.a(), di.b(), q, r).k, 1e-8);
  EXPECT_MATRIX_NEAR(gains.back(), MatX::Zero(1, 2), 1e-12);  // Qf = 0: last step does nothing
}

// ------------------------------------------------------------------ closed-loop trackers

const models::BicycleParams kCar{2.5, 0.6, 10.0, 3.0};

struct TrackingStats {
  double rmse = 0.0;
  double max_abs_steer = 0.0;
  double max_steer_rate = 0.0;
  double max_speed = 0.0;
  bool finished = false;
};

TrackingStats run(PathTracker& tracker, const trajectory::ReferencePath& path,
                  models::BicycleState x) {
  const models::KinematicBicycle car(kCar);
  const double dt = 0.05;
  TrackingStats st;
  double ss = 0.0;
  int n = 0;
  double prev_steer = 0.0;
  for (int k = 0; k < 4000 && !tracker.status().finished; ++k) {
    const VehicleCommand u = tracker.compute(x, path, dt);
    if (k > 0)
      st.max_steer_rate = std::max(st.max_steer_rate, std::abs(u.steering - prev_steer) / dt);
    prev_steer = u.steering;
    st.max_abs_steer = std::max(st.max_abs_steer, std::abs(u.steering));
    x = car.step(x, models::BicycleInput(u.acceleration, u.steering), dt);
    st.max_speed = std::max(st.max_speed, std::abs(x(3)));
    if (k > 100) {
      ss += tracker.status().cross_track_error * tracker.status().cross_track_error;
      ++n;
    }
  }
  st.finished = tracker.status().finished;
  st.rmse = std::sqrt(ss / std::max(n, 1));
  return st;
}

TEST(LqrTracker, TracksSCurveAndCircle) {
  for (const auto type : {trajectory::TrackType::kSCurve, trajectory::TrackType::kCircle}) {
    const auto path = trajectory::make_track(type, 4.0);
    LqrTracker lqr(kCar);
    const TrackingStats st = run(lqr, path, models::BicycleState(0, 0.5, 0, 0));
    EXPECT_TRUE(st.finished);
    EXPECT_LT(st.rmse, 0.1);
  }
}

TEST(LqrTracker, TracksStraightPathInReverse) {
  // Reverse along +x: the vehicle faces +x but drives backwards towards -x.
  std::vector<trajectory::PathPoint> pts;
  for (int i = 0; i <= 120; ++i) {
    pts.push_back({0.25 * i, Vec2(-0.25 * i, 0.0), 0.0, 0.0, -1.5});
  }
  const trajectory::ReferencePath path(pts);
  LqrTracker lqr(kCar);
  const TrackingStats st = run(lqr, path, models::BicycleState(0, 0.4, 0.05, 0));
  EXPECT_TRUE(st.finished);
  EXPECT_LT(std::abs(lqr.status().cross_track_error), 0.05);
}

TEST(MpcTracker, TracksTrackAndRespectsConstraints) {
  const auto path = trajectory::make_track(trajectory::TrackType::kSCurve, 4.0);
  MpcConfig cfg;
  MpcTracker mpc(kCar, cfg);
  const TrackingStats st = run(mpc, path, models::BicycleState(0, 0.5, 0, 0));
  EXPECT_TRUE(st.finished);
  EXPECT_LT(st.rmse, 0.1);
  EXPECT_LE(st.max_abs_steer, kCar.max_steer + 1e-9);
  // The rate constraint holds up to the QP tolerance.
  EXPECT_LE(st.max_steer_rate, cfg.max_steer_rate * 1.05);
  EXPECT_LE(st.max_speed, kCar.max_speed + 1e-6);
  EXPECT_EQ(mpc.status().prediction.size(), static_cast<std::size_t>(cfg.horizon));
}

TEST(MpcTracker, SpeedConstraintCapsReferenceSpeed) {
  // Reference faster than the vehicle limit: the speed constraint must hold.
  const auto path = trajectory::make_track(trajectory::TrackType::kStraight, 20.0);
  models::BicycleParams slow = kCar;
  slow.max_speed = 5.0;
  MpcConfig cfg;
  cfg.horizon = 10;
  MpcTracker mpc(slow, cfg);
  const models::KinematicBicycle car(slow);
  models::BicycleState x(0, 0, 0, 0);
  int solved = 0;
  const int steps = 80;
  for (int k = 0; k < steps; ++k) {
    const VehicleCommand u = mpc.compute(x, path, 0.05);
    solved += mpc.last_qp_solved() ? 1 : 0;
    x = car.step(x, models::BicycleInput(u.acceleration, u.steering), 0.05);
    ASSERT_LE(x(3), 5.0 + 0.05);
  }
  EXPECT_GT(x(3), 4.5);
  EXPECT_GE(solved, steps * 9 / 10);  // the QP converges within its iteration budget
}

}  // namespace
}  // namespace roboticscore::control
