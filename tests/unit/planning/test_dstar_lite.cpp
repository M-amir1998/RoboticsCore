#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/math/random.hpp"
#include "roboticscore/planning/grid/dstar_lite.hpp"
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

[[maybe_unused]] bool path_is_valid(const GridMap& map, const std::vector<GridIndex>& cells) {
  for (std::size_t i = 0; i < cells.size(); ++i) {
    if (map.blocked(cells[i])) return false;
    if (i > 0 && !std::isfinite(map.step_cost(cells[i - 1], cells[i]))) return false;
  }
  return true;
}

// ------------------------------------------------------------------------------- D* Lite

TEST(DStarLite, InitialPlanMatchesAStar) {
  const GridMap map = map_from_rows({
      "..........",
      "....#.....",
      "....#.....",
      "....#.....",
      "..........",
  });
  DStarLite planner(map, {1, 2}, {8, 2});
  ASSERT_EQ(planner.compute_shortest_path(), PlanStatus::kFound);
  const auto p = planner.path();
  EXPECT_NEAR(path_length(map.spec(), p), astar(map, {1, 2}, {8, 2}).cost, 1e-12);
  EXPECT_NEAR(planner.rhs({1, 2}), 3.0 + 4.0 * kSqrt2, 1e-12);
}

TEST(DStarLite, ReplansAfterBlockingAndUnblocking) {
  GridMap map(GridSpec(20, 20, 1.0));
  DStarLite planner(map, {0, 10}, {19, 10});
  ASSERT_EQ(planner.compute_shortest_path(), PlanStatus::kFound);
  const std::size_t first = planner.last_expanded();
  // Drop a wall across the straight-line path.
  std::vector<CellChange> wall;
  for (int y = 5; y <= 15; ++y)
    wall.push_back({{10, y}, true});
  EXPECT_EQ(planner.update_cells(wall), 11U);
  ASSERT_EQ(planner.compute_shortest_path(), PlanStatus::kFound);
  for (const auto& c : wall)
    map.set_blocked(c.cell, true);
  EXPECT_NEAR(path_length(map.spec(), planner.path()), astar(map, {0, 10}, {19, 10}).cost, 1e-9);
  EXPECT_GT(first, 0U);
  // Re-applying the same changes is a no-op.
  EXPECT_EQ(planner.update_cells(wall), 0U);
  // Enclose the goal completely: no path.
  std::vector<CellChange> box;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx != 0 || dy != 0) box.push_back({{18 + dx, 10 + dy}, true});
    }
  }
  box.push_back({{19, 9}, true});
  box.push_back({{19, 11}, true});
  planner.update_cells(box);
  EXPECT_EQ(planner.compute_shortest_path(), PlanStatus::kNoPath);
  EXPECT_TRUE(planner.path().empty());
}

TEST(DStarLiteProperty, MatchesFreshAStarWhileMovingAndReplanning) {
  math::Random rng(testing::kPropertySeed + 3);
  int checks = 0;
  for (int trial = 0; trial < 25; ++trial) {
    GridMap truth = random_map(rng, 25, 25, 0.15);
    const GridIndex start{0, 0};
    const GridIndex goal{24, 24};
    truth.set_blocked(start, false);
    truth.set_blocked(goal, false);
    DStarLite planner(truth, start, goal);
    if (planner.compute_shortest_path() != PlanStatus::kFound) continue;
    GridIndex robot = start;
    for (int step = 0; step < 40 && !(robot == goal); ++step) {
      robot = planner.next_step();
      planner.move_start(robot);
      // Random environment changes away from the robot and the goal.
      std::vector<CellChange> changes;
      for (int k = 0; k < 6; ++k) {
        const GridIndex c{static_cast<int>(rng.uniform_index(25)),
                          static_cast<int>(rng.uniform_index(25))};
        if (c == robot || c == goal) continue;
        changes.push_back({c, rng.uniform01() < 0.6});
      }
      planner.update_cells(changes);
      for (const auto& ch : changes)
        truth.set_blocked(ch.cell, ch.blocked);
      const PlanStatus st = planner.compute_shortest_path();
      const GridPath ref = astar(truth, robot, goal);
      ASSERT_EQ(st == PlanStatus::kFound, ref.found()) << "trial " << trial << " step " << step;
      if (!ref.found()) break;
      ASSERT_NEAR(path_length(truth.spec(), planner.path()), ref.cost, 1e-9);
      ++checks;
    }
  }
  EXPECT_GT(checks, 100);
}

TEST(DStarLite, RejectsOutOfBoundsEndpoints) {
  const GridMap map(GridSpec(5, 5, 1.0));
  EXPECT_THROW(DStarLite(map, {-1, 0}, {4, 4}), std::invalid_argument);
  DStarLite planner(map, {0, 0}, {4, 4});
  EXPECT_THROW(planner.move_start({7, 7}), std::invalid_argument);
}

}  // namespace
}  // namespace roboticscore::planning
