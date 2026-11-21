#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/math/random.hpp"
#include "roboticscore/planning/grid/grid_search.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::planning {
namespace {

constexpr double kSqrt2 = 1.4142135623730951;

// Builds a map from rows written top (high y) to bottom: '#' blocked, '.' free.
GridMap map_from_rows(const std::vector<std::string>& rows) {
  const int h = static_cast<int>(rows.size());
  const int w = static_cast<int>(rows.front().size());
  GridMap map(GridSpec(w, h, 1.0));
  for (int r = 0; r < h; ++r) {
    for (int x = 0; x < w; ++x) {
      if (rows[static_cast<std::size_t>(r)][static_cast<std::size_t>(x)] == '#') {
        map.set_blocked({x, h - 1 - r}, true);
      }
    }
  }
  return map;
}

GridMap random_map(math::Random& rng, int w, int h, double density) {
  GridMap map(GridSpec(w, h, 1.0));
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (rng.uniform01() < density) {
        map.set_blocked({x, y}, true);
      }
    }
  }
  return map;
}

bool path_is_valid(const GridMap& map, const std::vector<GridIndex>& cells) {
  for (std::size_t i = 0; i < cells.size(); ++i) {
    if (map.blocked(cells[i])) return false;
    if (i > 0 && !std::isfinite(map.step_cost(cells[i - 1], cells[i]))) return false;
  }
  return true;
}

TEST(GridMap, StepCostsAndCornerCutting) {
  const GridMap map = map_from_rows({
      "...",
      ".#.",
      "...",
  });
  EXPECT_DOUBLE_EQ(map.step_cost({0, 0}, {1, 0}), 1.0);
  EXPECT_DOUBLE_EQ(map.step_cost({0, 2}, {1, 2}), 1.0);
  // Diagonal (0,0)->(1,1) enters the blocked centre; (0,1)->(1,2) cuts its corner.
  EXPECT_TRUE(std::isinf(map.step_cost({0, 0}, {1, 1})));
  EXPECT_TRUE(std::isinf(map.step_cost({0, 1}, {1, 2})));
  EXPECT_TRUE(map.blocked({-1, 0}));  // out of bounds counts as blocked
}

TEST(GridMap, InflationGrowsObstacles) {
  GridMap map(GridSpec(9, 9, 0.5));
  map.set_blocked({4, 4}, true);
  const GridMap inflated = map.inflated(1.0);  // 2 cells
  EXPECT_TRUE(inflated.blocked({6, 4}));
  EXPECT_TRUE(inflated.blocked({5, 5}));
  EXPECT_FALSE(inflated.blocked({6, 6}));  // sqrt(8) cells > 2
  EXPECT_FALSE(inflated.blocked({7, 4}));
}

TEST(GridSearch, KnownOptimalCostsAroundWall) {
  const GridMap map = map_from_rows({
      "..........",
      "....#.....",
      "....#.....",
      "....#.....",
      "..........",
  });
  const GridIndex s{1, 2};
  const GridIndex g{8, 2};
  const GridPath a = astar(map, s, g);
  const GridPath d = dijkstra(map, s, g);
  ASSERT_TRUE(a.found());
  ASSERT_TRUE(d.found());
  // Optimal 8-connected detour: up to the gap above the wall and back down (no corner cutting).
  const double expected = 3.0 + 4.0 * kSqrt2;
  EXPECT_NEAR(a.cost, expected, 1e-12);
  EXPECT_NEAR(d.cost, expected, 1e-12);
  EXPECT_LE(a.expanded, d.expanded);
  EXPECT_TRUE(path_is_valid(map, a.cells));
  EXPECT_EQ(a.cells.front(), s);
  EXPECT_EQ(a.cells.back(), g);
}

TEST(GridSearch, StartEqualsGoal) {
  const GridMap map(GridSpec(5, 5, 0.1));
  for (const GridPath& p :
       {astar(map, {2, 2}, {2, 2}), dijkstra(map, {2, 2}, {2, 2}),
        breadth_first_search(map, {2, 2}, {2, 2}), theta_star(map, {2, 2}, {2, 2})}) {
    ASSERT_TRUE(p.found());
    EXPECT_EQ(p.cells.size(), 1U);
    EXPECT_DOUBLE_EQ(p.cost, 0.0);
  }
}

TEST(GridSearch, BlockedOrOutOfBoundsEndpoints) {
  GridMap map(GridSpec(5, 5, 1.0));
  map.set_blocked({0, 0}, true);
  EXPECT_EQ(astar(map, {0, 0}, {4, 4}).status, PlanStatus::kInvalidStart);
  EXPECT_EQ(astar(map, {4, 4}, {0, 0}).status, PlanStatus::kInvalidGoal);
  EXPECT_EQ(astar(map, {-1, 0}, {4, 4}).status, PlanStatus::kInvalidStart);
  EXPECT_EQ(breadth_first_search(map, {4, 4}, {9, 9}).status, PlanStatus::kInvalidGoal);
}

TEST(GridSearch, UnreachableGoal) {
  const GridMap map = map_from_rows({
      "..#..",
      "..#..",
      "..#..",
  });
  for (const GridPath& p :
       {astar(map, {0, 1}, {4, 1}), dijkstra(map, {0, 1}, {4, 1}),
        breadth_first_search(map, {0, 1}, {4, 1}), theta_star(map, {0, 1}, {4, 1})}) {
    EXPECT_EQ(p.status, PlanStatus::kNoPath);
    EXPECT_TRUE(p.cells.empty());
  }
}

TEST(GridSearch, FourConnectedManhattanIsOptimal) {
  const GridMap map(GridSpec(10, 10, 1.0));
  const GridSearchConfig cfg{Connectivity::kFour, Heuristic::kManhattan, 1.0};
  const GridPath p = astar(map, {0, 0}, {6, 3}, cfg);
  ASSERT_TRUE(p.found());
  EXPECT_DOUBLE_EQ(p.cost, 9.0);
  EXPECT_EQ(p.cells.size(), 10U);
}

TEST(GridSearch, ResolutionScalesCost) {
  const GridMap map(GridSpec(10, 10, 0.25));
  EXPECT_NEAR(astar(map, {0, 0}, {4, 0}).cost, 1.0, 1e-12);
}

TEST(GridSearch, ThetaStarIsShorterAndHasLineOfSight) {
  math::Random rng(5);
  GridMap map = random_map(rng, 40, 40, 0.2);
  map.set_blocked({1, 1}, false);
  map.set_blocked({38, 38}, false);
  const GridPath a = astar(map, {1, 1}, {38, 38});
  const GridPath t = theta_star(map, {1, 1}, {38, 38});
  ASSERT_EQ(a.found(), t.found());
  if (!a.found()) GTEST_SKIP() << "random map disconnected";
  EXPECT_LE(t.cost, a.cost + 1e-9);
  // Any-angle paths are bounded below by the straight-line distance.
  EXPECT_GE(t.cost, std::hypot(37.0, 37.0) - 1e-9);
  for (std::size_t i = 1; i < t.cells.size(); ++i) {
    EXPECT_TRUE(line_of_sight(map, t.cells[i - 1], t.cells[i]));
  }
}

TEST(LineOfSight, CornerTouchIsConservative) {
  const GridMap map = map_from_rows({
      "...",
      ".#.",
      "...",
  });
  EXPECT_FALSE(line_of_sight(map, {0, 0}, {2, 2}));  // through the blocked centre
  EXPECT_FALSE(line_of_sight(map, {0, 1}, {1, 2}));  // grazes the blocked corner
  EXPECT_TRUE(line_of_sight(map, {0, 0}, {2, 0}));
}

TEST(GridSearchProperty, OptimalityAndConsistencyOnRandomMaps) {
  math::Random rng(testing::kPropertySeed);
  int solved = 0;
  for (int trial = 0; trial < 60; ++trial) {
    GridMap map = random_map(rng, 30, 25, 0.25);
    const GridIndex s{static_cast<int>(rng.uniform_index(30)),
                      static_cast<int>(rng.uniform_index(25))};
    const GridIndex g{static_cast<int>(rng.uniform_index(30)),
                      static_cast<int>(rng.uniform_index(25))};
    map.set_blocked(s, false);
    map.set_blocked(g, false);
    const GridPath d = dijkstra(map, s, g);
    const GridPath a = astar(map, s, g, {Connectivity::kEight, Heuristic::kOctile, 1.0});
    const GridPath e = astar(map, s, g, {Connectivity::kEight, Heuristic::kEuclidean, 1.0});
    const GridPath w = astar(map, s, g, {Connectivity::kEight, Heuristic::kOctile, 2.0});
    const GridPath b = breadth_first_search(map, s, g);
    ASSERT_EQ(d.found(), a.found());
    ASSERT_EQ(d.found(), b.found());
    if (!d.found()) continue;
    ++solved;
    ASSERT_NEAR(a.cost, d.cost, 1e-9);
    ASSERT_NEAR(e.cost, d.cost, 1e-9);
    ASSERT_LE(w.cost, 2.0 * d.cost + 1e-9);  // weighted A* suboptimality bound
    ASSERT_GE(b.cost, d.cost - 1e-9);
    // A* cost is never below the octile lower bound.
    ASSERT_GE(a.cost, heuristic_distance(Heuristic::kOctile, s, g) - 1e-9);
    ASSERT_TRUE(path_is_valid(map, a.cells));
    ASSERT_LE(a.expanded, d.expanded);
  }
  EXPECT_GT(solved, 20);
}

}  // namespace
}  // namespace roboticscore::planning
