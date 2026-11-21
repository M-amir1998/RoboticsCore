// Numerical robustness of filters, controllers and solvers at degenerate inputs: zero noise,
// tiny time steps, near-singular covariances and redundant constraints.

#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "roboticscore/control/lqr.hpp"
#include "roboticscore/estimation/kalman_filter.hpp"
#include "roboticscore/estimation/particle_filter.hpp"
#include "roboticscore/estimation/unscented_kalman_filter.hpp"
#include "roboticscore/localization/robot_models.hpp"
#include "roboticscore/math/linear_algebra.hpp"
#include "roboticscore/models/double_integrator.hpp"
#include "roboticscore/models/unicycle.hpp"
#include "roboticscore/optimization/qp_solver.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore {
namespace {

TEST(FilterNumerics, KalmanCovarianceStaysSymmetricPsdOverLongRuns) {
  const models::DoubleIntegrator di(2, 1e-4);  // tiny time step
  estimation::KalmanFilter kf(VecX::Zero(4), MatX::Identity(4, 4));
  const MatX h = di.position_measurement();
  for (int k = 0; k < 20000; ++k) {
    kf.predict(di.a(), di.process_noise(0.0));  // zero process noise
    if (k % 10 == 0) kf.update(Vec2(0.1, -0.2), h, MatX::Identity(2, 2) * 1e-6);
  }
  EXPECT_TRUE(kf.covariance().allFinite());
  EXPECT_TRUE(math::is_symmetric(kf.covariance(), 1e-12));
  const Eigen::SelfAdjointEigenSolver<MatX> eig(kf.covariance());
  EXPECT_GE(eig.eigenvalues().minCoeff(), -1e-12);
}

TEST(FilterNumerics, UkfToleratesNearSingularCovariance) {
  estimation::UnscentedKalmanFilter ukf(Vec3(1, 2, 0.3), Mat3::Identity() * 1e-14, {},
                                        localization::pose_state_space());
  const auto process = localization::unicycle_process_model({0, 0, 0, 0, 1e-14, 1e-14});
  ukf.predict(process, Vec2(1.0, 0.1), 0.1);
  EXPECT_TRUE(ukf.state().allFinite());
  EXPECT_TRUE(ukf.covariance().allFinite());
}

TEST(FilterNumerics, ParticleFilterWithZeroNoiseCollapsesGracefully) {
  math::Random rng(1);
  estimation::ParticleFilter pf(MatX::Constant(2, 100, 3.0));  // all particles identical
  pf.predict([](Eigen::Ref<VecX>, math::Random&) {}, rng);
  const auto r = pf.update([](const Eigen::Ref<const VecX>&) { return -1e6; });
  EXPECT_FALSE(r.degenerate);
  EXPECT_NEAR(pf.effective_sample_size(), 100.0, 1e-6);
  EXPECT_MATRIX_NEAR(pf.covariance(), MatX::Zero(2, 2), 1e-15);
  pf.resample(estimation::ResamplingMethod::kResidual, rng);
  EXPECT_MATRIX_NEAR(pf.mean(), Vec2(3, 3), 1e-12);
}

TEST(ModelNumerics, UnicycleJacobiansFiniteForTinyDt) {
  for (const double dt : {0.0, 1e-12, 1e-6}) {
    const auto j = models::Unicycle::jacobians(Vec3(1, 2, 3), {2.0, 1e9}, dt);
    EXPECT_TRUE(j.wrt_state.allFinite());
    EXPECT_TRUE(j.wrt_input.allFinite());
  }
}

TEST(ControlNumerics, DareWithNearlySingularInputWeight) {
  const models::DoubleIntegrator di(1, 0.05);
  const auto res =
      control::solve_dare(di.a(), di.b(), MatX::Identity(2, 2), MatX::Constant(1, 1, 1e-8));
  EXPECT_TRUE(res.converged);
  EXPECT_LT(res.residual, 1e-8);
  // Cheap control: the closed loop becomes very fast but stays stable.
  const Eigen::EigenSolver<MatX> eig(di.a() - di.b() * res.k);
  EXPECT_LT(eig.eigenvalues().cwiseAbs().maxCoeff(), 1.0);
}

TEST(OptimizationNumerics, QpWithRedundantConstraints) {
  // x0 + x1 = 1 stated twice, plus duplicated box rows.
  optimization::QpProblem q;
  q.p = MatX::Identity(2, 2);
  q.q = Vec2(-1, -3);
  q.a = (MatX(4, 2) << 1, 1, 1, 1, 1, 0, 1, 0).finished();
  q.l = Vec4(1, 1, 0, 0);
  q.u = Vec4(1, 1, 0.8, 0.8);
  const auto r = optimization::solve_qp(q);
  ASSERT_TRUE(r.solved());
  EXPECT_NEAR(r.x(0) + r.x(1), 1.0, 1e-4);
  EXPECT_MATRIX_NEAR(r.x, Vec2(0.0, 1.0), 1e-3);
}

}  // namespace
}  // namespace roboticscore
