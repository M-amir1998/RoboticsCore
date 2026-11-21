#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/estimation/extended_kalman_filter.hpp"
#include "roboticscore/estimation/kalman_filter.hpp"
#include "roboticscore/estimation/particle_filter.hpp"
#include "roboticscore/estimation/resampling.hpp"
#include "roboticscore/estimation/unscented_kalman_filter.hpp"
#include "roboticscore/localization/robot_models.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/math/statistics.hpp"
#include "roboticscore/models/double_integrator.hpp"
#include "roboticscore/models/unicycle.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::estimation {
namespace {

// ------------------------------------------------------------------ sigma points / UKF

TEST(SigmaPoints, ReproduceMeanAndCovariance) {
  const Vec3 mean(1.0, -2.0, 0.5);
  Mat3 cov;
  cov << 2.0, 0.3, 0.1, 0.3, 1.0, -0.2, 0.1, -0.2, 0.5;
  for (const SigmaPointParams params :
       {SigmaPointParams{}, SigmaPointParams{0.5, 2.0, 1.0}, SigmaPointParams{1e-3, 2.0, 0.0}}) {
    const SigmaPoints sp = merwe_sigma_points(mean, cov, params);
    ASSERT_EQ(sp.points.cols(), 7);
    EXPECT_NEAR(sp.mean_weights.sum(), 1.0, 1e-9);
    const VecX m = sp.points * sp.mean_weights;
    EXPECT_MATRIX_NEAR(m, mean, 1e-8);
    MatX c = MatX::Zero(3, 3);
    for (Index i = 0; i < sp.points.cols(); ++i) {
      const VecX d = sp.points.col(i) - mean;
      c += sp.covariance_weights(i) * d * d.transpose();
    }
    // The first covariance weight carries the extra (1 - alpha^2 + beta) term, which only
    // multiplies the zero-deviation central point, so the covariance is still exact.
    EXPECT_MATRIX_NEAR(c, cov, 1e-6);
  }
  EXPECT_THROW((void)merwe_sigma_points(mean, MatX::Identity(2, 2), {}), std::invalid_argument);
}

TEST(UnscentedKalmanFilter, MatchesKalmanFilterOnLinearSystem) {
  const models::DoubleIntegrator di(2, 0.1);
  ProcessModel process;
  process.f = [&](const VecX& x, const VecX&, double) -> VecX { return di.a() * x; };
  process.noise = [&](const VecX&, const VecX&, double) -> MatX { return di.process_noise(0.3); };
  MeasurementModel meas;
  meas.h = [&](const VecX& x) -> VecX { return di.position_measurement() * x; };
  meas.noise = MatX::Identity(2, 2) * 0.04;

  KalmanFilter kf(VecX::Zero(4), MatX::Identity(4, 4));
  UnscentedKalmanFilter ukf(VecX::Zero(4), MatX::Identity(4, 4));
  math::Random rng(3);
  for (int k = 0; k < 30; ++k) {
    kf.predict(di.a(), di.process_noise(0.3));
    ukf.predict(process, VecX(), 0.1);
    const Vec2 z(rng.normal(), rng.normal());
    kf.update(z, di.position_measurement(), meas.noise);
    ukf.update(meas, z);
  }
  EXPECT_MATRIX_NEAR(ukf.state(), kf.state(), 1e-9);
  EXPECT_MATRIX_NEAR(ukf.covariance(), kf.covariance(), 1e-9);
}

TEST(UnscentedKalmanFilter, BeatsEkfOnPolarToCartesianBias) {
  // Classic example: a sensor at the origin measures (range, bearing) of a target with a
  // large bearing uncertainty. The true posterior mean of (x, y) is pulled towards the
  // sensor (E[cos(theta)] < 1). Use the "prediction" step as the transform under test.
  const double r = 1.0;
  const double sigma_r = 0.02;
  const double sigma_b = 0.35;
  ProcessModel polar_to_cartesian;
  polar_to_cartesian.f = [](const VecX& x, const VecX&, double) -> VecX {
    return Vec2(x(0) * std::cos(x(1)), x(0) * std::sin(x(1)));
  };
  polar_to_cartesian.jacobian = [](const VecX& x, const VecX&, double) -> MatX {
    Mat2 j;
    j << std::cos(x(1)), -x(0) * std::sin(x(1)), std::sin(x(1)), x(0) * std::cos(x(1));
    return j;
  };
  polar_to_cartesian.noise = [](const VecX&, const VecX&, double) -> MatX { return Mat2::Zero(); };
  const Vec2 mean(r, math::kHalfPi);
  const Mat2 cov = Vec2(sigma_r * sigma_r, sigma_b * sigma_b).asDiagonal();

  // Monte Carlo ground truth.
  math::Random rng(99);
  Vec2 mc = Vec2::Zero();
  const int n = 200000;
  for (int i = 0; i < n; ++i) {
    const VecX s = rng.multivariate_normal(mean, cov);
    mc += polar_to_cartesian.f(s, VecX(), 0.0);
  }
  mc /= n;

  ExtendedKalmanFilter ekf(mean, cov);
  UnscentedKalmanFilter ukf(mean, cov, {1.0, 2.0, 1.0});  // kappa = 3 - n
  ekf.predict(polar_to_cartesian, VecX(), 0.0);
  ukf.predict(polar_to_cartesian, VecX(), 0.0);
  const double ekf_error = (ekf.state() - mc).norm();
  const double ukf_error = (ukf.state() - mc).norm();
  EXPECT_GT(ekf_error, 0.04);  // EKF ignores the curvature: mean stays at (0, 1)
  EXPECT_LT(ukf_error, 0.25 * ekf_error);
}

TEST(UnscentedKalmanFilter, PoseLocalizationWithWrappedHeading) {
  const auto process = localization::unicycle_process_model({});
  UnscentedKalmanFilter ukf(Vec3(0, 0, math::kPi - 0.05), MatX::Identity(3, 3) * 0.01, {},
                            localization::pose_state_space());
  // Heading crosses the +-pi discontinuity during prediction.
  ukf.predict(process, Vec2(1.0, 1.0), 0.1);
  EXPECT_NEAR(ukf.state()(2), -math::kPi + 0.05, 1e-6);
  EXPECT_LT(ukf.covariance()(2, 2), 0.02);  // no spurious 2*pi spread
}

// ------------------------------------------------------------------------ resampling

TEST(Resampling, SystematicCopiesAreFloorOrCeil) {
  math::Random rng(1);
  const std::vector<double> w = {0.5, 0.25, 0.125, 0.125};
  for (int trial = 0; trial < 50; ++trial) {
    const auto idx = resample(w, 8, ResamplingMethod::kSystematic, rng);
    std::vector<int> counts(4, 0);
    for (const auto i : idx) {
      ++counts[i];
    }
    EXPECT_EQ(counts, (std::vector<int>{4, 2, 1, 1}));
  }
}

TEST(Resampling, AllSchemesAreUnbiasedAndSkipZeroWeights) {
  const std::vector<double> w = {0.1, 0.0, 0.6, 0.3};
  for (const auto method : {ResamplingMethod::kMultinomial, ResamplingMethod::kStratified,
                            ResamplingMethod::kSystematic, ResamplingMethod::kResidual}) {
    math::Random rng(17);
    std::vector<double> counts(4, 0.0);
    const int trials = 3000;
    const std::size_t n = 7;
    for (int t = 0; t < trials; ++t) {
      const auto idx = resample(w, n, method, rng);
      ASSERT_EQ(idx.size(), n);
      ASSERT_TRUE(std::is_sorted(idx.begin(), idx.end()));
      for (const auto i : idx) {
        ASSERT_NE(i, 1U) << "zero-weight particle selected";
        counts[i] += 1.0;
      }
    }
    for (std::size_t i = 0; i < 4; ++i) {
      EXPECT_NEAR(counts[i] / (trials * static_cast<double>(n)), w[i], 0.01)
          << "method " << static_cast<int>(method) << " index " << i;
    }
  }
}

TEST(Resampling, RejectsInvalidWeightsAndComputesEss) {
  math::Random rng(1);
  const std::vector<double> zeros = {0.0, 0.0};
  const std::vector<double> negative = {1.0, -0.1};
  const std::vector<double> nan = {1.0, std::numeric_limits<double>::quiet_NaN()};
  EXPECT_THROW((void)resample(zeros, 2, ResamplingMethod::kSystematic, rng), std::invalid_argument);
  EXPECT_THROW((void)resample(negative, 2, ResamplingMethod::kSystematic, rng),
               std::invalid_argument);
  EXPECT_THROW((void)resample(nan, 2, ResamplingMethod::kSystematic, rng), std::invalid_argument);
  const std::vector<double> uniform(10, 3.0);
  EXPECT_NEAR(effective_sample_size(uniform), 10.0, 1e-12);
  const std::vector<double> peaked = {1.0, 0.0, 0.0};
  EXPECT_NEAR(effective_sample_size(peaked), 1.0, 1e-12);
}

// ------------------------------------------------------------------- particle filter

TEST(ParticleFilter, WeightsStayNormalizedAndUnderflowSafe) {
  math::Random rng(testing::kPropertySeed);
  auto pf = ParticleFilter::from_gaussian(VecX::Zero(1), MatX::Identity(1, 1), 500, rng);
  for (int k = 0; k < 20; ++k) {
    // Extremely peaked likelihood: exp() of these values underflows in linear space.
    const auto result =
        pf.update([](const Eigen::Ref<const VecX>& x) { return -1e4 * x(0) * x(0); });
    ASSERT_FALSE(result.degenerate);
    ASSERT_NEAR(pf.weights().sum(), 1.0, 1e-12);
    ASSERT_GE(result.effective_sample_size, 1.0 - 1e-12);
  }
}

TEST(ParticleFilter, DegenerateUpdateKeepsPreviousWeights) {
  math::Random rng(2);
  auto pf = ParticleFilter::from_gaussian(VecX::Zero(2), MatX::Identity(2, 2), 50, rng);
  const VecX before = pf.log_weights();
  const auto result = pf.update(
      [](const Eigen::Ref<const VecX>&) { return -std::numeric_limits<double>::infinity(); });
  EXPECT_TRUE(result.degenerate);
  EXPECT_MATRIX_NEAR(pf.log_weights(), before, 0.0);
}

TEST(ParticleFilter, ApproximatesKalmanPosteriorOnLinearGaussianModel) {
  math::Random rng(5);
  const double q = 0.2;
  const double r = 0.5;
  auto pf = ParticleFilter::from_gaussian(VecX::Zero(1), MatX::Identity(1, 1), 20000, rng);
  KalmanFilter kf(VecX::Zero(1), MatX::Identity(1, 1));
  for (int k = 0; k < 10; ++k) {
    const double z = 0.3 * k;
    pf.predict([&](Eigen::Ref<VecX> x, math::Random& g) { x(0) += g.normal(0.0, std::sqrt(q)); },
               rng);
    pf.update([&](const Eigen::Ref<const VecX>& x) {
      const double e = z - x(0);
      return -0.5 * e * e / r;
    });
    pf.resample_if_needed(0.5, ResamplingMethod::kSystematic, rng);
    kf.predict(MatX::Identity(1, 1), MatX::Constant(1, 1, q));
    kf.update(VecX::Constant(1, z), MatX::Identity(1, 1), MatX::Constant(1, 1, r));
  }
  EXPECT_NEAR(pf.mean()(0), kf.state()(0), 0.03);
  EXPECT_NEAR(pf.covariance()(0, 0), kf.covariance()(0, 0), 0.03);
}

TEST(ParticleFilter, LandmarkLocalizationFromPoorPrior) {
  const std::vector<Vec2> landmarks = {{8, 0}, {8, 8}, {0, 8}, {-4, 4}};
  const double rs = 0.2;
  const double bs = 0.05;
  math::Random rng(21);
  Vec3 truth(1.0, 1.0, 0.3);
  auto pf = ParticleFilter::from_gaussian(Vec3(0, 0, 0), Vec3(4.0, 4.0, 0.5).asDiagonal(), 2000,
                                          rng, localization::pose_state_space());
  for (int k = 0; k < 60; ++k) {
    const Vec2 u(0.8, 0.15);
    truth = models::Unicycle::step(truth, {u(0), u(1)}, 0.1);
    pf.predict(
        [&](Eigen::Ref<VecX> x, math::Random& g) {
          const Vec3 next = models::Unicycle::step(
              Vec3(x), {u(0) + g.normal(0, 0.05), u(1) + g.normal(0, 0.02)}, 0.1);
          x = next;
        },
        rng);
    std::vector<Vec2> zs;
    for (const Vec2& lm : landmarks) {
      zs.push_back(localization::predict_range_bearing(truth, lm)
                   + Vec2(rng.normal(0, rs), rng.normal(0, bs)));
    }
    pf.update([&](const Eigen::Ref<const VecX>& x) {
      double ll = 0.0;
      for (std::size_t i = 0; i < landmarks.size(); ++i) {
        ll += localization::range_bearing_log_likelihood(Vec3(x), landmarks[i], zs[i], rs, bs);
      }
      return ll;
    });
    pf.resample_if_needed(0.5, ResamplingMethod::kSystematic, rng);
  }
  const VecX err = localization::pose_residual(truth, pf.mean());
  EXPECT_LT(err.head<2>().norm(), 0.2);
  EXPECT_LT(std::abs(err(2)), 0.1);
}

}  // namespace
}  // namespace roboticscore::estimation
