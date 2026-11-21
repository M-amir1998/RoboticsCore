#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/linear_algebra.hpp"
#include "roboticscore/math/numerical_diff.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/sim/sensors.hpp"
#include "roboticscore/slam/ekf_slam.hpp"
#include "roboticscore/slam/fastslam.hpp"
#include "roboticscore/slam/pose_graph.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::slam {
namespace {

using geometry::Pose2;

// Robot drives a circle among landmarks with non-contiguous ids; returns final errors.
struct LandmarkWorld {
  std::vector<Vec2> positions = {{8, 0}, {8, 8}, {0, 10}, {-6, 6}, {-8, -2}, {0, -6}, {5, -5}};
  std::vector<std::size_t> ids = {7, 2, 15, 0, 9, 4, 11};
};

template <typename Filter>
std::vector<Pose2> run_landmark_slam(Filter& filter, const LandmarkWorld& world, int steps,
                                     std::uint64_t seed) {
  math::Random rng(seed);
  sim::OdometryNoise odom{0.02, 0.002, 0.002, 0.02, 1e-6, 1e-6};
  sim::RangeBearingSensor sensor;
  sensor.max_range = 12.0;
  sensor.range_std = 0.1;
  sensor.bearing_std = 0.02;
  Pose2 truth(0, 0, 0);
  std::vector<Pose2> trajectory;
  for (int k = 0; k < steps; ++k) {
    const models::UnicycleInput u{1.0, 0.15};
    truth = models::Unicycle::step(truth, u, 0.1);
    filter.predict(odom.corrupt(u, rng), 0.1);
    std::vector<RangeBearingObservation> zs;
    for (const auto& o : sensor.observe(truth, world.positions, rng)) {
      zs.push_back({world.ids[o.id], o.range, o.bearing});
    }
    filter.update(zs);
    trajectory.push_back(truth);
  }
  return trajectory;
}

TEST(EkfSlam, BuildsMapWithArbitraryLandmarkIds) {
  const LandmarkWorld world;
  EkfSlam slam(Pose2(0, 0, 0), Mat3::Identity() * 1e-6, {0.02, 0.002, 0.002, 0.02, 1e-6, 1e-6}, 0.1,
               0.02);
  const auto truth = run_landmark_slam(slam, world, 500, 3);
  ASSERT_EQ(slam.landmark_count(), world.positions.size());
  EXPECT_EQ(slam.state().size(), 3 + 2 * static_cast<Index>(world.positions.size()));
  for (std::size_t i = 0; i < world.positions.size(); ++i) {
    const auto est = slam.landmark(world.ids[i]);
    ASSERT_TRUE(est.has_value()) << "id " << world.ids[i];
    EXPECT_LT((*est - world.positions[i]).norm(), 0.35) << "id " << world.ids[i];
    EXPECT_LT(slam.landmark_covariance(world.ids[i])->trace(), 0.2);
  }
  EXPECT_FALSE(slam.landmark(99).has_value());
  EXPECT_LT((slam.pose().translation() - truth.back().translation()).norm(), 0.4);
  EXPECT_TRUE(math::is_positive_definite(slam.covariance()));
}

TEST(EkfSlam, LandmarkUncertaintyShrinksWithObservations) {
  EkfSlam slam(Pose2(0, 0, 0), Mat3::Identity() * 1e-6, {}, 0.2, 0.05);
  const std::vector<RangeBearingObservation> z = {{5, 4.0, 0.3}};
  slam.update(z);
  const double first = slam.landmark_covariance(5)->trace();
  for (int i = 0; i < 10; ++i)
    slam.update(z);
  EXPECT_LT(slam.landmark_covariance(5)->trace(), 0.3 * first);
  // Robot-landmark cross-covariance is populated at initialisation.
  EkfSlam uncertain(Pose2(0, 0, 0), Mat3::Identity() * 0.5, {}, 0.2, 0.05);
  uncertain.update(z);
  const double cross = uncertain.covariance().block<3, 2>(0, 3).cwiseAbs().maxCoeff();
  EXPECT_GT(cross, 0.1);
}

TEST(PoseGraph, EdgeJacobiansMatchNumerical) {
  math::Random rng(testing::kPropertySeed);
  for (int i = 0; i < 50; ++i) {
    const Pose2 xi(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-3, 3));
    const Pose2 xj(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-3, 3));
    const Pose2 z(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-3, 3));
    const EdgeLinearization lin = linearize_edge(xi, xj, z);
    const auto wrap = [](const VecX& a, const VecX& b) -> VecX {
      VecX d = a - b;
      d(2) = math::wrap_to_pi(d(2));
      return d;
    };
    const auto fi = [&](const VecX& v) -> VecX {
      return linearize_edge(Pose2(v(0), v(1), v(2)), xj, z).error;
    };
    const auto fj = [&](const VecX& v) -> VecX {
      return linearize_edge(xi, Pose2(v(0), v(1), v(2)), z).error;
    };
    ASSERT_MATRIX_NEAR(lin.jacobian_from,
                       math::numerical_jacobian(fi, VecX(Vec3(xi.x(), xi.y(), xi.theta())), wrap),
                       1e-6);
    ASSERT_MATRIX_NEAR(lin.jacobian_to,
                       math::numerical_jacobian(fj, VecX(Vec3(xj.x(), xj.y(), xj.theta())), wrap),
                       1e-6);
  }
}

// Square loops with drifting odometry and exact-but-noisy loop closures.
struct LoopDataset {
  std::vector<Pose2> truth;
  PoseGraph2D graph;
};

LoopDataset make_loop_dataset(std::uint64_t seed) {
  LoopDataset d;
  math::Random rng(seed);
  Pose2 pose(0, 0, 0);
  Pose2 odom(0, 0, 0);
  d.truth.push_back(pose);
  d.graph.add_node(odom);
  const Mat3 info = Vec3(1.0 / 0.01, 1.0 / 0.01, 1.0 / 0.0004).asDiagonal();
  for (int lap = 0; lap < 2; ++lap) {
    for (int side = 0; side < 4; ++side) {
      for (int k = 0; k < 10; ++k) {
        const Pose2 step(1.0, 0.0, k == 9 ? math::kHalfPi : 0.0);
        pose = pose * step;
        const Pose2 noisy =
            step * Pose2(rng.normal(0, 0.1), rng.normal(0, 0.1), rng.normal(0, 0.02));
        odom = odom * noisy;
        d.truth.push_back(pose);
        const std::size_t id = d.graph.add_node(odom);
        d.graph.add_edge({id - 1, id, noisy, info});
      }
    }
  }
  // Loop closures between the two laps (same place, 40 nodes apart).
  for (std::size_t i = 0; i + 40 < d.truth.size(); i += 5) {
    const Pose2 rel = d.truth[i].between(d.truth[i + 40]);
    const Pose2 meas = rel * Pose2(rng.normal(0, 0.05), rng.normal(0, 0.05), rng.normal(0, 0.01));
    d.graph.add_edge({i, i + 40, meas, info * 4.0});
  }
  return d;
}

TEST(PoseGraph, LoopClosureRemovesDrift) {
  for (const auto method :
       {PoseGraphOptions::Method::kGaussNewton, PoseGraphOptions::Method::kLevenbergMarquardt}) {
    LoopDataset d = make_loop_dataset(5);
    const double before = absolute_trajectory_error(d.graph.poses(), d.truth, false);
    PoseGraphOptions opt;
    opt.method = method;
    opt.record_history = true;
    const PoseGraphResult r = d.graph.optimize(opt);
    EXPECT_TRUE(r.converged);
    EXPECT_LT(r.final_chi2, 0.05 * r.initial_chi2);
    const double after = absolute_trajectory_error(d.graph.poses(), d.truth, false);
    EXPECT_LT(after, 0.6 * before) << "before " << before << " after " << after;
    // The optimum must explain the measurements at least as well as the true trajectory
    // (the maximum-likelihood estimate need not equal the truth under noise).
    PoseGraph2D truth_graph = d.graph;
    for (std::size_t i = 0; i < d.truth.size(); ++i)
      truth_graph.set_pose(i, d.truth[i]);
    EXPECT_LE(d.graph.chi2(), truth_graph.chi2() + 1e-9);
    EXPECT_MATRIX_NEAR(d.graph.poses()[0].vector(), Vec3::Zero(), 0.0);  // fixed anchor
    EXPECT_EQ(r.pose_history.size(), r.chi2_history.size());
  }
}

TEST(PoseGraph, ExactMeasurementsGiveZeroError) {
  PoseGraph2D g;
  const Pose2 a(0, 0, 0);
  const Pose2 b(2, 1, 0.5);
  const Pose2 c(3, -1, -1.0);
  g.add_node(a);
  g.add_node(Pose2(1.5, 1.4, 0.2));  // perturbed initial guesses
  g.add_node(Pose2(3.3, -0.5, -0.8));
  g.add_edge({0, 1, a.between(b), Mat3::Identity()});
  g.add_edge({1, 2, b.between(c), Mat3::Identity()});
  g.add_edge({0, 2, a.between(c), Mat3::Identity()});
  const PoseGraphResult r = g.optimize();
  EXPECT_TRUE(r.converged);
  EXPECT_LT(g.chi2(), 1e-16);
  EXPECT_MATRIX_NEAR(g.poses()[2].vector(), c.vector(), 1e-8);
  EXPECT_THROW(g.add_edge({0, 0, a, Mat3::Identity()}), std::invalid_argument);
  g.set_fixed(0, false);
  EXPECT_THROW((void)g.optimize(), std::invalid_argument);
}

TEST(FastSlam, MapsLandmarksAndTracksPose) {
  const LandmarkWorld world;
  FastSlam slam(60, Pose2(0, 0, 0), {0.02, 0.002, 0.002, 0.02, 1e-6, 1e-6}, 0.1, 0.02, 11);
  const auto truth = run_landmark_slam(slam, world, 400, 3);
  const auto& best = slam.best_particle();
  double ss = 0.0;
  for (std::size_t i = 0; i < world.positions.size(); ++i) {
    const auto& lm = best.landmarks.at(world.ids[i]);
    ASSERT_TRUE(lm.initialized);
    ss += (lm.mean - world.positions[i]).squaredNorm();
  }
  EXPECT_LT(std::sqrt(ss / static_cast<double>(world.positions.size())), 0.5);
  EXPECT_LT((slam.mean_pose().translation() - truth.back().translation()).norm(), 0.6);
  EXPECT_GE(slam.effective_sample_size(), 1.0);
}

TEST(SlamMetrics, AteAndRpe) {
  std::vector<Pose2> truth;
  std::vector<Pose2> est;
  const Pose2 offset(3.0, -2.0, 0.4);
  for (int i = 0; i < 20; ++i) {
    truth.emplace_back(i, 0.1 * i * i, 0.0);
    est.push_back(offset * truth.back());
  }
  EXPECT_NEAR(absolute_trajectory_error(est, truth, true), 0.0, 1e-9);
  EXPECT_GT(absolute_trajectory_error(est, truth, false), 1.0);
  EXPECT_NEAR(relative_pose_error(est, truth, 1), 0.0, 1e-9);  // rigid offsets cancel
  EXPECT_THROW((void)absolute_trajectory_error({}, {}), std::invalid_argument);
}

}  // namespace
}  // namespace roboticscore::slam
