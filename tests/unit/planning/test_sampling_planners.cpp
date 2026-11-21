#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/math/random.hpp"
#include "roboticscore/planning/sampling/sampling_planners.hpp"
#include "roboticscore/sim/scenarios.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::planning {
namespace {

void expect_valid_path(const geometry::Environment2D& env, const SamplingResult& r,
                       const Vec2& start, const Vec2& goal, double radius) {
  ASSERT_TRUE(r.found);
  ASSERT_GE(r.path.size(), 2U);
  EXPECT_MATRIX_NEAR(r.path.front(), start, 1e-12);
  EXPECT_MATRIX_NEAR(r.path.back(), goal, 1e-12);
  for (std::size_t i = 1; i < r.path.size(); ++i) {
    EXPECT_TRUE(env.segment_free(r.path[i - 1], r.path[i], radius)) << "segment " << i;
  }
  EXPECT_NEAR(r.cost, polyline_length(r.path), 1e-9);
  EXPECT_GE(r.cost, (goal - start).norm() - 1e-9);
}

// Tree invariant: every node's cost equals its parent's cost plus the edge length.
void expect_consistent_tree(const std::vector<TreeNode>& tree) {
  for (std::size_t i = 0; i < tree.size(); ++i) {
    if (tree[i].parent < 0) {
      EXPECT_DOUBLE_EQ(tree[i].cost, 0.0);
      continue;
    }
    const TreeNode& p = tree[static_cast<std::size_t>(tree[i].parent)];
    ASSERT_NEAR(tree[i].cost, p.cost + (tree[i].point - p.point).norm(), 1e-9) << "node " << i;
  }
}

class SamplingPlannersTest : public ::testing::Test {
 protected:
  sim::Scenario scenario = sim::cluttered_scenario(3, 35);
  Vec2 start = scenario.start.translation();
  Vec2 goal = scenario.goal.translation();
  SamplingConfig config = [] {
    SamplingConfig c;
    c.step_size = 1.0;
    c.robot_radius = 0.3;
    c.max_iterations = 4000;
    c.seed = 7;
    return c;
  }();
};

TEST_F(SamplingPlannersTest, RrtFindsCollisionFreePathDeterministically) {
  const SamplingResult a = rrt(scenario.environment, start, goal, config);
  expect_valid_path(scenario.environment, a, start, goal, config.robot_radius);
  expect_consistent_tree(a.tree);
  const SamplingResult b = rrt(scenario.environment, start, goal, config);
  ASSERT_EQ(a.path.size(), b.path.size());
  for (std::size_t i = 0; i < a.path.size(); ++i) {
    EXPECT_EQ(a.path[i], b.path[i]);
  }
  config.seed = 8;
  EXPECT_NE(rrt(scenario.environment, start, goal, config).tree.size(), a.tree.size());
}

TEST_F(SamplingPlannersTest, RrtConnectFindsPath) {
  const SamplingResult r = rrt_connect(scenario.environment, start, goal, config);
  expect_valid_path(scenario.environment, r, start, goal, config.robot_radius);
  EXPECT_FALSE(r.goal_tree.empty());
  expect_consistent_tree(r.tree);
  expect_consistent_tree(r.goal_tree);
}

TEST_F(SamplingPlannersTest, RrtStarRewiringNeverIncreasesCosts) {
  std::map<std::size_t, double> last_cost;
  int rewires = 0;
  double last_solution = std::numeric_limits<double>::infinity();
  const SamplingResult r =
      rrt_star(scenario.environment, start, goal, config, [&](const SamplingEvent& e) {
        switch (e.type) {
          case SamplingEvent::Type::kNodeAdded:
            last_cost[e.node] = e.cost;
            break;
          case SamplingEvent::Type::kRewired:
            ++rewires;
            ASSERT_LT(e.cost, last_cost[e.node]) << "rewire increased cost of node " << e.node;
            last_cost[e.node] = e.cost;
            break;
          case SamplingEvent::Type::kSolutionImproved:
            ASSERT_LT(e.cost, last_solution);
            last_solution = e.cost;
            break;
        }
      });
  expect_valid_path(scenario.environment, r, start, goal, config.robot_radius);
  expect_consistent_tree(r.tree);
  EXPECT_GT(rewires, 0);
  for (std::size_t i = 1; i < r.cost_history.size(); ++i) {
    EXPECT_LT(r.cost_history[i].second, r.cost_history[i - 1].second);
  }
  EXPECT_NEAR(r.cost_history.back().second, r.cost, 1e-9);
}

TEST_F(SamplingPlannersTest, RrtStarImprovesOnFirstSolution) {
  const SamplingResult r = rrt_star(scenario.environment, start, goal, config);
  ASSERT_TRUE(r.found);
  ASSERT_GE(r.cost_history.size(), 2U);
  EXPECT_LT(r.cost, 0.95 * r.cost_history.front().second);
}

TEST(SamplingPlanners, OptimalPlannersApproachStraightLineInOpenSpace) {
  const geometry::Environment2D env({{0, 0}, {10, 10}});
  const Vec2 s(1, 1);
  const Vec2 g(9, 9);
  SamplingConfig c;
  c.step_size = 1.5;
  c.max_iterations = 3000;
  c.goal_tolerance = 0.5;
  const double straight = (g - s).norm();
  const SamplingResult star = rrt_star(env, s, g, c);
  const SamplingResult informed = informed_rrt_star(env, s, g, c);
  ASSERT_TRUE(star.found);
  ASSERT_TRUE(informed.found);
  EXPECT_LT(star.cost, 1.03 * straight);
  EXPECT_LT(informed.cost, 1.02 * straight);
}

TEST(SamplingPlanners, InformedEllipseSamplesLieInsideEllipse) {
  math::Random rng(1);
  const Vec2 a(1, 2);
  const Vec2 b(6, -1);
  const double c_min = (b - a).norm();
  const double c_best = 1.3 * c_min;
  Vec2 mean = Vec2::Zero();
  const int n = 20000;
  for (int i = 0; i < n; ++i) {
    const Vec2 x = sample_informed_ellipse(a, b, c_best, rng.uniform01(), rng.uniform01());
    ASSERT_LE((x - a).norm() + (x - b).norm(), c_best + 1e-9);
    mean += x;
  }
  EXPECT_MATRIX_NEAR(mean / n, Vec2(0.5 * (a + b)), 0.03);  // uniform => centred
  EXPECT_THROW((void)sample_informed_ellipse(a, b, 0.5 * c_min, 0.1, 0.1), std::invalid_argument);
}

TEST(SamplingPlanners, PrmSolvesNarrowPassage) {
  const sim::Scenario sc = sim::narrow_passage_scenario(1.4);
  SamplingConfig c;
  c.robot_radius = 0.3;
  c.prm_samples = 600;
  c.prm_neighbors = 12;
  c.seed = 2;
  const SamplingResult r = prm(sc.environment, sc.start.translation(), sc.goal.translation(), c);
  expect_valid_path(sc.environment, r, sc.start.translation(), sc.goal.translation(),
                    c.robot_radius);
  EXPECT_FALSE(r.roadmap_edges.empty());
}

TEST(SamplingPlanners, UnreachableGoalAndInvalidEndpoints) {
  geometry::Environment2D env({{0, 0}, {10, 10}});
  env.add_obstacle(geometry::Polygon::from_aabb({{7, 7}, {10, 7.5}}));
  env.add_obstacle(geometry::Polygon::from_aabb({{7, 7}, {7.5, 10}}));
  SamplingConfig c;
  c.max_iterations = 500;
  c.prm_samples = 100;
  const Vec2 s(1, 1);
  const Vec2 enclosed(9, 9);
  EXPECT_FALSE(rrt(env, s, enclosed, c).found);
  EXPECT_FALSE(rrt_connect(env, s, enclosed, c).found);
  EXPECT_FALSE(rrt_star(env, s, enclosed, c).found);
  EXPECT_FALSE(prm(env, s, enclosed, c).found);
  EXPECT_FALSE(rrt(env, Vec2(7.2, 8.0), s, c).found);  // start inside an obstacle
  const SamplingResult same = rrt_star(env, s, s, c);
  ASSERT_TRUE(same.found);
  EXPECT_DOUBLE_EQ(same.cost, 0.0);
  c.step_size = 0.0;
  EXPECT_THROW((void)rrt(env, s, enclosed, c), std::invalid_argument);
}

}  // namespace
}  // namespace roboticscore::planning
