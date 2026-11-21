#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/geometry/environment.hpp"
#include "roboticscore/mapping/occupancy_grid.hpp"
#include "roboticscore/mapping/raycast.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/sim/scenarios.hpp"
#include "roboticscore/sim/sensors.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::mapping {
namespace {

// ----------------------------------------------------------------------------- Bresenham

TEST(Bresenham, SingleCellAndAxisAligned) {
  EXPECT_EQ(bresenham_line({3, 4}, {3, 4}), (std::vector<GridIndex>{{3, 4}}));
  EXPECT_EQ(bresenham_line({0, 0}, {3, 0}),
            (std::vector<GridIndex>{{0, 0}, {1, 0}, {2, 0}, {3, 0}}));
  EXPECT_EQ(bresenham_line({0, 0}, {0, -2}), (std::vector<GridIndex>{{0, 0}, {0, -1}, {0, -2}}));
  EXPECT_EQ(bresenham_line({0, 0}, {2, 2}), (std::vector<GridIndex>{{0, 0}, {1, 1}, {2, 2}}));
}

TEST(BresenhamProperty, AllOctantsAreEightConnectedAndMinimal) {
  math::Random rng(testing::kPropertySeed);
  for (int i = 0; i < 500; ++i) {
    const GridIndex a{static_cast<int>(rng.uniform_index(41)) - 20,
                      static_cast<int>(rng.uniform_index(41)) - 20};
    const GridIndex b{static_cast<int>(rng.uniform_index(41)) - 20,
                      static_cast<int>(rng.uniform_index(41)) - 20};
    const auto cells = bresenham_line(a, b);
    ASSERT_EQ(cells.front(), a);
    ASSERT_EQ(cells.back(), b);
    ASSERT_EQ(cells.size(),
              static_cast<std::size_t>(std::max(std::abs(b.x - a.x), std::abs(b.y - a.y))) + 1);
    for (std::size_t k = 1; k < cells.size(); ++k) {
      ASSERT_LE(std::abs(cells[k].x - cells[k - 1].x), 1);
      ASSERT_LE(std::abs(cells[k].y - cells[k - 1].y), 1);
    }
    // Every cell lies within half a cell of the ideal line (perpendicular distance).
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    for (const auto& c : cells) {
      if (len == 0.0) break;
      const double dist = std::abs((b.x - a.x) * (a.y - c.y) - (a.x - c.x) * (b.y - a.y)) / len;
      ASSERT_LE(dist, 0.7072);
    }
  }
}

// ---------------------------------------------------------------------- grid traversal

TEST(TraverseGrid, CoversEveryCellTheSegmentCrosses) {
  const GridSpec spec(20, 20, 0.5, Vec2(-5, -5));
  math::Random rng(3);
  for (int i = 0; i < 300; ++i) {
    const Vec2 a(rng.uniform(-6, 6), rng.uniform(-6, 6));
    const Vec2 b(rng.uniform(-6, 6), rng.uniform(-6, 6));
    const auto cells = traverse_grid(spec, a, b);
    std::set<std::pair<int, int>> visited;
    for (const auto& c : cells) {
      ASSERT_TRUE(spec.in_bounds(c));
      visited.insert({c.x, c.y});
    }
    // Dense sampling must never find a crossed in-bounds cell that was not reported.
    for (int k = 0; k <= 4000; ++k) {
      const Vec2 p = a + (b - a) * (k / 4000.0);
      const auto c = spec.world_to_cell(p);
      if (spec.in_bounds(c)) {
        ASSERT_TRUE(visited.count({c.x, c.y}))
            << "segment " << i << " missed cell " << c.x << "," << c.y;
      }
    }
  }
}

TEST(TraverseGrid, BoundaryConditions) {
  const GridSpec spec(4, 4, 1.0);
  // Entirely outside.
  EXPECT_TRUE(traverse_grid(spec, {-2, -2}, {-1, 5}).empty());
  // Starts outside, ends inside: clipped at the boundary.
  const auto in = traverse_grid(spec, {-3, 0.5}, {1.5, 0.5});
  EXPECT_EQ(in, (std::vector<GridIndex>{{0, 0}, {1, 0}}));
  // Exact diagonal through cell corners reports both edge neighbours.
  const auto diag = traverse_grid(spec, {0.0, 0.0}, {2.0, 2.0});
  EXPECT_EQ(diag.front(), (GridIndex{0, 0}));
  EXPECT_NE(std::find(diag.begin(), diag.end(), GridIndex{1, 0}), diag.end());
  EXPECT_NE(std::find(diag.begin(), diag.end(), GridIndex{0, 1}), diag.end());
  // Ending exactly on the outer boundary stays in bounds.
  const auto edge = traverse_grid(spec, {0.5, 0.5}, {4.0, 0.5});
  EXPECT_EQ(edge.back(), (GridIndex{3, 0}));
  // Zero-length segment.
  EXPECT_EQ(traverse_grid(spec, {1.5, 1.5}, {1.5, 1.5}), (std::vector<GridIndex>{{1, 1}}));
}

// ------------------------------------------------------------------------ occupancy grid

TEST(OccupancyGrid, LogOddsConversionAndClamping) {
  EXPECT_NEAR(probability_to_log_odds(0.5), 0.0, 1e-15);
  EXPECT_NEAR(log_odds_to_probability(probability_to_log_odds(0.83)), 0.83, 1e-14);
  EXPECT_NEAR(log_odds_to_probability(-800.0), 0.0, 1e-300);  // no overflow
  EXPECT_NEAR(log_odds_to_probability(800.0), 1.0, 1e-15);
  EXPECT_THROW((void)probability_to_log_odds(1.0), std::invalid_argument);

  OccupancyGrid grid(GridSpec(2, 2, 1.0));
  for (int i = 0; i < 100; ++i) {
    grid.add_log_odds({0, 0}, 1.0);
  }
  EXPECT_DOUBLE_EQ(grid.log_odds({0, 0}), 4.0);
  EXPECT_EQ(grid.state({0, 0}), CellState::kOccupied);
  EXPECT_EQ(grid.state({1, 1}), CellState::kUnknown);
  EXPECT_FALSE(grid.observed({1, 1}));
  EXPECT_THROW((void)grid.log_odds({5, 5}), std::out_of_range);
  EXPECT_THROW(OccupancyGrid(GridSpec(2, 2, 1.0), InverseSensorModel{0.7, 0.4, 0.5, 1.0, -1.0}),
               std::invalid_argument);
}

TEST(OccupancyGrid, RayMarksFreeSpaceAndHitCell) {
  OccupancyGrid grid(GridSpec(10, 10, 1.0));
  grid.integrate_ray({0.5, 0.5}, {5.5, 0.5}, true);
  for (int x = 0; x < 5; ++x) {
    EXPECT_LT(grid.probability({x, 0}), 0.5);
  }
  EXPECT_GT(grid.probability({5, 0}), 0.5);
  EXPECT_FALSE(grid.observed({6, 0}));
  grid.integrate_ray({0.5, 2.5}, {5.5, 2.5}, false);  // no return: end cell is free
  EXPECT_LT(grid.probability({5, 2}), 0.5);
}

TEST(OccupancyGrid, ScanMappingRecoversRoom) {
  geometry::Environment2D env({{0, 0}, {10, 8}});
  env.add_obstacle(geometry::Polygon::from_aabb({{4, 3}, {6, 5}}));
  sim::World world(env);
  sim::LidarSensor lidar{360, math::kTwoPi, 15.0, 0.0};
  const GridSpec spec(50, 40, 0.2);
  OccupancyGrid grid(spec);
  math::Random rng(1);
  for (const Vec2& p : {Vec2(2, 2), Vec2(8, 2), Vec2(8, 6), Vec2(2, 6)}) {
    const geometry::Pose2 pose(p.x(), p.y(), 0.0);
    const auto scan = lidar.scan(pose, world, rng);
    grid.integrate_scan(pose, scan.angles, scan.ranges, lidar.max_range);
  }
  // The box interior is never observed; its boundary is occupied; open floor is free.
  EXPECT_EQ(grid.state(spec.world_to_cell({1.0, 4.0})), CellState::kFree);
  EXPECT_EQ(grid.state(spec.world_to_cell({4.05, 4.0})), CellState::kOccupied);
  const auto truth = sim::rasterize(env, spec, 0.0);
  const MapComparison cmp = compare_to_ground_truth(grid, truth);
  EXPECT_GT(cmp.coverage, 0.85);
  EXPECT_GT(cmp.accuracy, 0.9);
}

}  // namespace
}  // namespace roboticscore::mapping
