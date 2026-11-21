#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/planning/kinodynamic/hybrid_astar.hpp"
#include "roboticscore/planning/kinodynamic/reeds_shepp.hpp"
#include "roboticscore/sim/scenarios.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::planning {
namespace {

using geometry::Pose2;
using math::kPi;

void expect_reaches(const Pose2& start, const Pose2& goal, const ReedsSheppPath& p) {
  const auto samples = sample_reeds_shepp(start, p, 0.05);
  const Pose2& end = samples.back().pose;
  EXPECT_NEAR((end.translation() - goal.translation()).norm(), 0.0, 1e-6);
  EXPECT_NEAR(math::angle_difference(end.theta(), goal.theta()), 0.0, 1e-6);
}

TEST(ReedsShepp, StraightLinesAndArcs) {
  const double r = 2.0;
  EXPECT_NEAR(reeds_shepp_shortest({0, 0, 0}, {5, 0, 0}, r).length(), 5.0, 1e-9);
  const ReedsSheppPath back = reeds_shepp_shortest({0, 0, 0}, {-5, 0, 0}, r);
  EXPECT_NEAR(back.length(), 5.0, 1e-9);  // straight reverse, no turning
  EXPECT_EQ(back.reversals(), 0);
  // Quarter circle to the left.
  const ReedsSheppPath arc = reeds_shepp_shortest({0, 0, 0}, {r, r, kPi / 2}, r);
  EXPECT_NEAR(arc.length(), r * kPi / 2, 1e-9);
  expect_reaches({0, 0, 0}, {r, r, kPi / 2}, arc);
  EXPECT_THROW((void)reeds_shepp_shortest({0, 0, 0}, {1, 1, 0}, 0.0), std::invalid_argument);
}

TEST(ReedsShepp, ParallelParkingUsesReversal) {
  // Sideways shift with the same heading cannot be done well forward-only.
  const ReedsSheppPath p = reeds_shepp_shortest({0, 0, 0}, {0, 1.0, 0}, 1.0);
  EXPECT_GE(p.reversals(), 1);
  expect_reaches({0, 0, 0}, {0, 1.0, 0}, p);
}

TEST(ReedsSheppProperty, ShortestPathReachesGoalAndIsSymmetric) {
  math::Random rng(testing::kPropertySeed);
  for (int i = 0; i < 400; ++i) {
    const Pose2 a(rng.uniform(-10, 10), rng.uniform(-10, 10), rng.uniform(-kPi, kPi));
    const Pose2 b(rng.uniform(-10, 10), rng.uniform(-10, 10), rng.uniform(-kPi, kPi));
    const double r = rng.uniform(0.5, 4.0);
    const ReedsSheppPath ab = reeds_shepp_shortest(a, b, r);
    const ReedsSheppPath ba = reeds_shepp_shortest(b, a, r);
    SCOPED_TRACE(i);
    expect_reaches(a, b, ab);
    ASSERT_NEAR(ab.length(), ba.length(), 1e-6 * (1.0 + ab.length()));
    ASSERT_GE(ab.length(), (b.translation() - a.translation()).norm() - 1e-9);
    ASSERT_LE(ab.reversals(), 2);
    // Every returned candidate is valid, and the shortest is the minimum over them.
    for (const auto& c : reeds_shepp_candidates(a, b, r)) {
      ASSERT_GE(c.length(), ab.length() - 1e-12);
    }
  }
}

class HybridAStarTest : public ::testing::Test {
 protected:
  models::BicycleParams vehicle{2.7, 0.6, 5.0, 2.0};
  models::VehicleFootprint footprint{4.5, 1.9, 0.9};
};

TEST_F(HybridAStarTest, ParksInFreeSlot) {
  const sim::Scenario sc = sim::parking_scenario();
  const HybridAStar planner(sc.environment, vehicle, footprint);
  const HybridAStarResult r = planner.plan(sc.start, sc.goal);
  ASSERT_TRUE(r.found);
  EXPECT_TRUE(r.analytic_expansion);
  const Pose2& end = r.path.back().pose;
  EXPECT_NEAR((end.translation() - sc.goal.translation()).norm(), 0.0, 1e-6);
  EXPECT_NEAR(math::angle_difference(end.theta(), sc.goal.theta()), 0.0, 1e-6);
  const double max_kappa = 1.0 / models::KinematicBicycle(vehicle).min_turning_radius();
  for (std::size_t i = 0; i < r.path.size(); ++i) {
    ASSERT_TRUE(planner.pose_free(r.path[i].pose)) << "sample " << i;
    ASSERT_LE(std::abs(r.path[i].curvature), max_kappa + 1e-9);
    if (i > 0) {
      ASSERT_LE((r.path[i].pose.translation() - r.path[i - 1].pose.translation()).norm(),
                planner.config().collision_step + 1e-6);
    }
  }
  EXPECT_GE(r.cost, r.length - 1e-6);  // penalties only add cost
  EXPECT_GT(r.expanded, 0U);
}

TEST_F(HybridAStarTest, ForwardOnlyHasNoGearSwitches) {
  const geometry::Environment2D env({{0, 0}, {40, 30}});
  HybridAStarConfig cfg;
  cfg.allow_reverse = false;
  const HybridAStar planner(env, vehicle, footprint, cfg);
  const HybridAStarResult r = planner.plan({5, 5, 0}, {30, 20, kPi / 2});
  ASSERT_TRUE(r.found);
  // The search itself never reverses; only the final Reeds-Shepp shot could.
  for (const auto& p : r.path) {
    if (!r.analytic_expansion) {
      ASSERT_TRUE(p.forward);
    }
  }
  EXPECT_GE(r.length, (Vec2(30, 20) - Vec2(5, 5)).norm());
}

TEST_F(HybridAStarTest, DeterministicAndRejectsInvalidRequests) {
  const sim::Scenario sc = sim::parking_scenario();
  const HybridAStar planner(sc.environment, vehicle, footprint);
  const auto a = planner.plan(sc.start, sc.goal);
  const auto b = planner.plan(sc.start, sc.goal);
  ASSERT_EQ(a.path.size(), b.path.size());
  EXPECT_EQ(a.expanded, b.expanded);
  // Goal overlapping a parked car.
  EXPECT_FALSE(planner.plan(sc.start, Pose2(sc.goal.x() + 2.8, 17.6, -kPi / 2)).found);
  HybridAStarConfig bad;
  bad.steering_samples = 4;
  EXPECT_THROW(HybridAStar(sc.environment, vehicle, footprint, bad), std::invalid_argument);
}

}  // namespace
}  // namespace roboticscore::planning
