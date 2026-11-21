#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/core/error.hpp"
#include "roboticscore/estimation/consistency.hpp"
#include "roboticscore/estimation/extended_kalman_filter.hpp"
#include "roboticscore/estimation/kalman_filter.hpp"
#include "roboticscore/localization/robot_models.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/linear_algebra.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/models/double_integrator.hpp"
#include "roboticscore/models/unicycle.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::estimation {
namespace {

MatX scalar(double v) {
  return MatX::Constant(1, 1, v);
}
VecX vec1(double v) {
  return VecX::Constant(1, v);
}

TEST(KalmanFilter, ScalarPredictAndUpdateByHand) {
  KalmanFilter kf(vec1(0.0), scalar(1.0));
  kf.predict(scalar(1.0), scalar(0.5));  // P = 1.5
  EXPECT_DOUBLE_EQ(kf.covariance()(0, 0), 1.5);
  const UpdateResult r = kf.update(vec1(3.0), scalar(1.0), scalar(0.5));  // K = 0.75
  EXPECT_TRUE(r.accepted);
  EXPECT_NEAR(kf.state()(0), 2.25, 1e-15);
  EXPECT_NEAR(kf.covariance()(0, 0), 0.375, 1e-15);
  EXPECT_NEAR(r.stats.nis, 9.0 / 2.0, 1e-15);  // y^2 / S = 9 / 2
}

TEST(KalmanFilter, ControlInputEntersPrediction) {
  const models::DoubleIntegrator di(1, 0.5);
  KalmanFilter kf(VecX::Zero(2), MatX::Identity(2, 2));
  kf.predict(di.a(), di.b(), vec1(2.0), MatX::Zero(2, 2));
  EXPECT_MATRIX_NEAR(kf.state(), Vec2(0.25, 1.0), 1e-15);
}

TEST(KalmanFilter, ZeroMeasurementNoiseSnapsToMeasurement) {
  KalmanFilter kf(Vec2(5.0, 0.0), MatX::Identity(2, 2));
  MatX h(1, 2);
  h << 1.0, 0.0;
  kf.update(vec1(1.0), h, scalar(0.0));
  EXPECT_NEAR(kf.state()(0), 1.0, 1e-12);
  EXPECT_NEAR(kf.covariance()(0, 0), 0.0, 1e-12);
  EXPECT_TRUE(math::is_symmetric(kf.covariance()));
}

TEST(KalmanFilter, GateRejectsOutliers) {
  KalmanFilter kf(vec1(0.0), scalar(1.0));
  const UpdateResult r = kf.update(vec1(100.0), scalar(1.0), scalar(1.0), 9.0);
  EXPECT_FALSE(r.accepted);
  EXPECT_DOUBLE_EQ(kf.state()(0), 0.0);
  EXPECT_DOUBLE_EQ(kf.covariance()(0, 0), 1.0);
}

TEST(KalmanFilter, RejectsInvalidInputs) {
  MatX indefinite(2, 2);
  indefinite << 1.0, 2.0, 2.0, 1.0;
  EXPECT_THROW(KalmanFilter(VecX::Zero(2), indefinite), std::invalid_argument);
  EXPECT_THROW(KalmanFilter(VecX::Zero(2), MatX::Identity(3, 3)), std::invalid_argument);
  KalmanFilter kf(VecX::Zero(2), MatX::Identity(2, 2));
  EXPECT_THROW(kf.predict(MatX::Identity(3, 3), MatX::Zero(3, 3)), std::invalid_argument);
  EXPECT_THROW(kf.update(vec1(1.0), MatX::Identity(2, 2), scalar(1.0)), std::invalid_argument);
  // Zero prior covariance with zero measurement noise: S = 0 is not positive definite.
  KalmanFilter certain(VecX::Zero(1), scalar(0.0));
  EXPECT_THROW(certain.update(vec1(1.0), scalar(1.0), scalar(0.0)), NumericalError);
}

TEST(KalmanFilter, MonteCarloNeesIsConsistent) {
  // A correctly modelled linear system must produce chi-square distributed NEES.
  const models::DoubleIntegrator di(1, 0.1);
  const MatX q = di.process_noise(0.2);
  const MatX h = di.position_measurement();
  const MatX r = scalar(0.05);
  const int runs = 200;
  const int steps = 60;
  math::Random rng(12345);
  double nees_sum = 0.0;
  for (int run = 0; run < runs; ++run) {
    const VecX x0 = Vec2(0.0, 1.0);
    const MatX p0 = MatX::Identity(2, 2) * 0.5;
    VecX truth = rng.multivariate_normal(x0, p0);
    KalmanFilter kf(x0, p0);
    for (int k = 0; k < steps; ++k) {
      truth = di.a() * truth + rng.multivariate_normal(VecX::Zero(2), q);
      kf.predict(di.a(), q);
      const VecX z = h * truth + rng.multivariate_normal(VecX::Zero(1), r);
      kf.update(z, h, r);
    }
    nees_sum += nees(truth - kf.state(), kf.covariance());
  }
  const Chi2Interval bounds = average_chi2_bounds(2, runs, 0.99);
  EXPECT_TRUE(bounds.contains(nees_sum / runs))
      << "average NEES " << nees_sum / runs << " outside [" << bounds.lower << ", " << bounds.upper
      << "]";
}

TEST(ExtendedKalmanFilter, ReducesToKalmanFilterForLinearModels) {
  const models::DoubleIntegrator di(1, 0.2);
  ProcessModel process;
  process.f = [&](const VecX& x, const VecX&, double) -> VecX {
    return di.a() * x;
  };
  process.noise = [&](const VecX&, const VecX&, double) -> MatX {
    return di.process_noise(1.0);
  };
  MeasurementModel meas;
  meas.h = [&](const VecX& x) -> VecX {
    return di.position_measurement() * x;
  };
  meas.noise = scalar(0.1);

  KalmanFilter kf(Vec2(0.0, 0.0), MatX::Identity(2, 2));
  ExtendedKalmanFilter ekf(Vec2(0.0, 0.0), MatX::Identity(2, 2));
  for (int k = 0; k < 20; ++k) {
    kf.predict(di.a(), di.process_noise(1.0));
    ekf.predict(process, VecX(), 0.2);  // numerical Jacobian path
    kf.update(vec1(0.3 * k), di.position_measurement(), scalar(0.1));
    ekf.update(meas, vec1(0.3 * k));
  }
  EXPECT_MATRIX_NEAR(ekf.state(), kf.state(), 1e-7);
  EXPECT_MATRIX_NEAR(ekf.covariance(), kf.covariance(), 1e-7);
}

TEST(ExtendedKalmanFilter, BearingInnovationIsWrapped) {
  // Robot at origin facing +x, landmark slightly below the -x axis: bearing ~ -179 deg.
  const Vec2 landmark(-10.0, -0.1745);
  ExtendedKalmanFilter ekf(Vec3(0, 0, 0), MatX::Identity(3, 3) * 0.01);
  const auto model = localization::range_bearing_model(landmark, 0.1, 0.05);
  // Measured bearing on the other side of the discontinuity (+179 deg).
  const Vec2 z(10.0, math::deg2rad(179.0));
  const UpdateResult r = ekf.update(model, z);
  EXPECT_LT(std::abs(r.stats.innovation(1)), math::deg2rad(3.0));
  EXPECT_LT(r.stats.nis, 50.0);
}

TEST(ExtendedKalmanFilter, AnalyticAndNumericalJacobiansAgree) {
  const auto process = localization::unicycle_process_model({});
  ProcessModel numeric = process;
  numeric.jacobian = nullptr;
  ExtendedKalmanFilter a(Vec3(1, 2, 0.5), MatX::Identity(3, 3) * 0.1);
  ExtendedKalmanFilter b = a;
  const Vec2 u(1.0, 0.3);
  a.predict(process, u, 0.1);
  b.predict(numeric, u, 0.1);
  EXPECT_MATRIX_NEAR(a.covariance(), b.covariance(), 1e-9);

  const auto meas = localization::range_bearing_model({5.0, 5.0}, 0.1, 0.02);
  MeasurementModel meas_numeric = meas;
  meas_numeric.jacobian = nullptr;
  a.update(meas, Vec2(5.5, 0.3));
  b.update(meas_numeric, Vec2(5.5, 0.3));
  EXPECT_MATRIX_NEAR(a.state(), b.state(), 1e-7);
}

TEST(ExtendedKalmanFilter, LandmarkLocalizationConverges) {
  const std::vector<Vec2> landmarks = {{10, 0}, {10, 10}, {0, 10}, {-5, 5}};
  const localization::VelocityNoise noise{0.02, 0.002, 0.002, 0.02, 1e-6, 1e-6};
  const auto process = localization::unicycle_process_model(noise);
  math::Random rng(7);
  Vec3 truth(0.0, 0.0, 0.0);
  ExtendedKalmanFilter ekf(Vec3(0.5, -0.5, 0.2), Vec3(1.0, 1.0, 0.1).asDiagonal());
  ekf.set_normalizer(localization::normalize_pose);
  const double dt = 0.1;
  for (int k = 0; k < 300; ++k) {
    const Vec2 u(1.0, 0.2);
    truth = models::Unicycle::step(truth, {u(0), u(1)}, dt);
    // Odometry reports the command corrupted by the modelled noise.
    const Mat2 m = noise.control_covariance(u(0), u(1));
    const Vec2 u_meas(u(0) + rng.normal(0, std::sqrt(m(0, 0))),
                      u(1) + rng.normal(0, std::sqrt(m(1, 1))));
    ekf.predict(process, u_meas, dt);
    for (const Vec2& lm : landmarks) {
      const Vec2 z = localization::predict_range_bearing(truth, lm)
                     + Vec2(rng.normal(0, 0.1), rng.normal(0, 0.02));
      ekf.update(localization::range_bearing_model(lm, 0.1, 0.02), z);
    }
  }
  const VecX err = localization::pose_residual(truth, ekf.state());
  EXPECT_LT(err.head<2>().norm(), 0.2);
  EXPECT_LT(std::abs(err(2)), 0.05);
  EXPECT_LT(nees(err, ekf.covariance()), 20.0);  // well inside a sane range for 3 dof
}

TEST(Consistency, Chi2BoundsMatchTables) {
  // Single 2-dof sample: 95% interval is [chi2(0.025, 2), chi2(0.975, 2)] = [0.0506, 7.378].
  const Chi2Interval b = average_chi2_bounds(2, 1, 0.95);
  EXPECT_NEAR(b.lower, 0.050636, 1e-5);
  EXPECT_NEAR(b.upper, 7.377759, 1e-5);
  EXPECT_THROW((void)average_chi2_bounds(0, 1), std::invalid_argument);
}

}  // namespace
}  // namespace roboticscore::estimation
