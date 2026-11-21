// Golden-value regression tests on deterministic scenarios. Values were recorded from this
// implementation (MSVC 19.50, Release) after checking them for plausibility (A* equals
// Dijkstra, Theta* is shorter, a wall makes D* Lite detour, ...). Tolerances absorb
// floating-point differences between compilers but catch behavioural changes. The scenarios
// avoid random sampling so the goldens do not depend on the C library's math functions.

#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/control/lqr.hpp"
#include "roboticscore/models/double_integrator.hpp"
#include "roboticscore/planning/grid/dstar_lite.hpp"
#include "roboticscore/planning/grid/grid_search.hpp"
#include "roboticscore/planning/kinodynamic/hybrid_astar.hpp"
#include "roboticscore/planning/kinodynamic/reeds_shepp.hpp"
#include "roboticscore/sim/scenarios.hpp"
#include "roboticscore/trajectory/polynomial.hpp"

namespace roboticscore {
namespace {

planning::GridMap maze_map() {
  const auto sc = sim::maze_scenario(7);
  const geometry::GridSpec spec(96, 72, 0.25);
  return planning::GridMap(spec, sim::rasterize(sc.environment, spec, 0.3));
}

TEST(Regression, GridPlannersOnMaze) {
  const auto map = maze_map();
  const planning::GridIndex s{6, 6};
  const planning::GridIndex g{89, 65};
  const auto a = planning::astar(map, s, g);
  const auto d = planning::dijkstra(map, s, g);
  const auto t = planning::theta_star(map, s, g);
  EXPECT_NEAR(a.cost, 32.5814754520, 1e-8);
  EXPECT_NEAR(d.cost, 32.5814754520, 1e-8);
  EXPECT_NEAR(t.cost, 30.5792147644, 1e-8);
  // Expansion counts depend only on integer grid logic and deterministic tie-breaking.
  EXPECT_EQ(a.expanded, 1056U);
  EXPECT_EQ(d.expanded, 1792U);
  EXPECT_EQ(t.expanded, 1062U);
}

TEST(Regression, DStarLiteReplanningCost) {
  const auto sc = sim::cluttered_scenario(3, 25);
  const geometry::GridSpec spec(60, 40, 0.5);
  const planning::GridMap map(spec, sim::rasterize(sc.environment, spec, 0.25));
  planning::DStarLite planner(map, {4, 4}, {55, 35});
  planner.compute_shortest_path();
  EXPECT_NEAR(planner.rhs({4, 4}), 66.7695526217, 1e-8);
  const auto p = planner.path();
  const auto mid = p[p.size() / 2];
  std::vector<planning::CellChange> wall;
  for (int d = -5; d <= 5; ++d) {
    wall.push_back({{mid.x + d, mid.y - d}, true});
    wall.push_back({{mid.x + d + 1, mid.y - d}, true});
  }
  planner.update_cells(wall);
  planner.compute_shortest_path();
  EXPECT_NEAR(planner.rhs({4, 4}), 69.2548339959, 1e-8);
  EXPECT_EQ(planner.last_expanded(), 283U);
}

TEST(Regression, ReedsSheppLengths) {
  EXPECT_NEAR(planning::reeds_shepp_shortest({0, 0, 0}, {3, 4, 1.2}, 2.0).length(), 5.352015075200,
              1e-9);
  EXPECT_NEAR(planning::reeds_shepp_shortest({0, 0, 0}, {-2, 1, -2.5}, 1.5).length(), 3.75, 1e-9);
  EXPECT_NEAR(planning::reeds_shepp_shortest({1, 1, 0.3}, {1.5, 1.2, 0.3}, 3.0).length(),
              0.734545710660, 1e-9);
}

TEST(Regression, HybridAStarParking) {
  const auto sc = sim::parking_scenario();
  const planning::HybridAStar planner(sc.environment, {2.7, 0.6, 3.0, 2.0}, {4.5, 1.9, 0.9});
  const auto r = planner.plan(sc.start, sc.goal);
  ASSERT_TRUE(r.found);
  EXPECT_NEAR(r.length, 20.9225066282, 1e-3 * 20.9);
  EXPECT_NEAR(r.cost, 27.3938266947, 1e-3 * 27.4);
  EXPECT_EQ(r.gear_switches, 1);
  EXPECT_EQ(r.expanded, 135U);
}

TEST(Regression, DareAndQuintic) {
  const models::DoubleIntegrator di(2, 0.1);
  const auto dare =
      control::solve_dare(di.a(), di.b(), MatX::Identity(4, 4), 0.1 * MatX::Identity(2, 2));
  EXPECT_NEAR(dare.p(0, 0), 13.317224441131, 1e-8);
  EXPECT_NEAR(dare.p(0, 2), 3.201562118716, 1e-8);
  EXPECT_NEAR(dare.k(0, 0), 2.585700896660, 1e-8);
  const auto q = trajectory::quintic_trajectory(0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 3.0);
  EXPECT_NEAR(q.position(1.3), 1.297016419753, 1e-10);
  EXPECT_NEAR(q.jerk(2.1), -0.022222222222, 1e-10);
}

}  // namespace
}  // namespace roboticscore
