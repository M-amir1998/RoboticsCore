#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/geometry/grid.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/sim/scenarios.hpp"
#include "roboticscore/sim/simulation.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::sim {
namespace {

World make_world() {
  World w(geometry::Environment2D({{0, 0}, {20, 20}}));
  w.environment.add_obstacle(geometry::Circle{{10, 10}, 1.0});
  w.landmarks = {{5, 5}, {15, 5}, {15, 15}, {5, 15}};
  return w;
}

SimulationConfig noisy_config(std::uint64_t seed) {
  SimulationConfig c;
  c.seed = seed;
  c.actuation_noise = {0.01, 0.001, 0.001, 0.01, 1e-4, 1e-4};
  c.odometry_noise = {0.02, 0.002, 0.002, 0.02, 1e-4, 1e-4};
  return c;
}

TEST(Simulation, SameSeedIsBitIdentical) {
  Simulation a(make_world(), {2, 2, 0}, noisy_config(42));
  Simulation b(make_world(), {2, 2, 0}, noisy_config(42));
  for (int k = 0; k < 50; ++k) {
    const StepResult ra = a.step({1.0, 0.1});
    const StepResult rb = b.step({1.0, 0.1});
    ASSERT_EQ(ra.pose.x(), rb.pose.x());
    ASSERT_EQ(ra.odometry.v, rb.odometry.v);
    const auto oa = a.observe_landmarks();
    const auto ob = b.observe_landmarks();
    ASSERT_EQ(oa.size(), ob.size());
    for (std::size_t i = 0; i < oa.size(); ++i) {
      ASSERT_EQ(oa[i].range, ob[i].range);
    }
  }
  Simulation c(make_world(), {2, 2, 0}, noisy_config(43));
  EXPECT_NE(c.step({1.0, 0.1}).pose.x(),
            Simulation(make_world(), {2, 2, 0}, noisy_config(42)).step({1.0, 0.1}).pose.x());
}

TEST(Simulation, SensorStreamsAreIndependent) {
  // Taking LiDAR scans must not change the landmark noise realisation.
  Simulation a(make_world(), {2, 2, 0}, noisy_config(7));
  Simulation b(make_world(), {2, 2, 0}, noisy_config(7));
  (void)b.scan_lidar();
  (void)b.observe_position();
  const auto oa = a.observe_landmarks();
  const auto ob = b.observe_landmarks();
  ASSERT_EQ(oa.size(), ob.size());
  EXPECT_EQ(oa[0].range, ob[0].range);
}

TEST(Simulation, ZeroNoiseMatchesKinematics) {
  Simulation sim(make_world(), {2, 2, 0}, SimulationConfig{});
  const StepResult r = sim.step({1.0, 0.0});
  EXPECT_NEAR(r.pose.x(), 2.1, 1e-15);
  EXPECT_DOUBLE_EQ(r.odometry.v, 1.0);
  EXPECT_NEAR(r.time, 0.1, 1e-15);
  EXPECT_FALSE(r.collision);
}

TEST(Simulation, DetectsCollision) {
  Simulation sim(make_world(), {8.5, 10, 0}, SimulationConfig{});
  for (int k = 0; k < 5; ++k) {
    sim.step({1.0, 0.0});
  }
  EXPECT_TRUE(sim.in_collision());
}

TEST(Sensors, RangeBearingRespectsFieldOfViewAndRange) {
  RangeBearingSensor s;
  s.max_range = 8.0;
  s.fov = math::kPi;  // front half-plane only
  s.range_std = 1e-9;
  s.bearing_std = 1e-9;
  math::Random rng(1);
  const std::vector<Vec2> lms = {{5, 0}, {-5, 0}, {20, 0}, {0, 3}};
  const auto obs = s.observe({0, 0, 0}, lms, rng);
  ASSERT_EQ(obs.size(), 2U);  // behind and too-far landmarks are not seen
  EXPECT_EQ(obs[0].id, 0U);
  EXPECT_NEAR(obs[0].range, 5.0, 1e-6);
  EXPECT_EQ(obs[1].id, 3U);
  EXPECT_NEAR(obs[1].bearing, math::kHalfPi, 1e-6);
}

TEST(Sensors, OutlierProbabilityIsRespected) {
  RangeBearingSensor s;
  s.max_range = 100.0;
  s.outlier_probability = 0.2;
  math::Random rng(3);
  const std::vector<Vec2> lms(50, Vec2(3, 4));
  int outliers = 0;
  int total = 0;
  for (int k = 0; k < 100; ++k) {
    for (const auto& o : s.observe({0, 0, 0}, lms, rng)) {
      outliers += o.outlier ? 1 : 0;
      ++total;
    }
  }
  EXPECT_NEAR(static_cast<double>(outliers) / total, 0.2, 0.02);
}

TEST(Sensors, LidarMeasuresWallsAndObstacles) {
  World w = make_world();
  LidarSensor lidar;
  lidar.beam_count = 4;  // beams at -pi, -pi/2, 0, pi/2
  lidar.range_std = 0.0;
  lidar.max_range = 30.0;
  math::Random rng(1);
  const LidarScan scan = lidar.scan({5, 10, 0}, w, rng);
  ASSERT_EQ(scan.ranges.size(), 4U);
  EXPECT_NEAR(scan.angles[0], -math::kPi, 1e-12);
  EXPECT_NEAR(scan.ranges[0], 5.0, 1e-9);   // west wall
  EXPECT_NEAR(scan.ranges[2], 4.0, 1e-9);   // circle at (10, 10), radius 1
  EXPECT_NEAR(scan.ranges[3], 10.0, 1e-9);  // north wall
  // Dynamic obstacles are visible to the LiDAR.
  w.dynamic_obstacles.push_back({{{5, 7}, 0.5}, {0, 0}});
  const LidarScan scan2 = lidar.scan({5, 10, 0}, w, rng);
  EXPECT_NEAR(scan2.ranges[1], 2.5, 1e-9);
}

TEST(World, DynamicObstaclesReflectAtBounds) {
  DynamicObstacle o{{{9.5, 5}, 0.5}, {2.0, 0.0}};
  o.advance(0.5, {{0, 0}, {10, 10}});
  EXPECT_NEAR(o.shape.center.x(), 8.5, 1e-12);  // 9.5 + 1.0 reflected about 9.5
  EXPECT_LT(o.velocity.x(), 0.0);
}

// Breadth-first reachability on a raster, used to validate generated scenarios.
bool reachable(const geometry::GridSpec& spec, const std::vector<std::uint8_t>& occ,
               const Vec2& from, const Vec2& to) {
  const auto s = spec.world_to_cell(from);
  const auto g = spec.world_to_cell(to);
  std::vector<bool> seen(spec.cell_count(), false);
  std::deque<geometry::GridIndex> q{s};
  seen[spec.linear_index(s)] = true;
  while (!q.empty()) {
    const auto c = q.front();
    q.pop_front();
    if (c == g) return true;
    for (const auto& d : {geometry::GridIndex{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
      const geometry::GridIndex n{c.x + d.x, c.y + d.y};
      if (spec.in_bounds(n) && !seen[spec.linear_index(n)] && occ[spec.linear_index(n)] == 0) {
        seen[spec.linear_index(n)] = true;
        q.push_back(n);
      }
    }
  }
  return false;
}

TEST(Scenarios, AllScenariosHaveFreeAndConnectedStartAndGoal) {
  for (const auto& name : scenario_names()) {
    const Scenario sc = make_scenario(name, 5);
    SCOPED_TRACE(name);
    EXPECT_TRUE(sc.environment.point_free(sc.start.translation(), 0.3));
    EXPECT_TRUE(sc.environment.point_free(sc.goal.translation(), 0.3));
    const auto& b = sc.environment.bounds();
    const double res = 0.25;
    const geometry::GridSpec spec(static_cast<int>(b.size().x() / res),
                                  static_cast<int>(b.size().y() / res), res, b.min);
    const auto occ = rasterize(sc.environment, spec, 0.3);
    EXPECT_TRUE(reachable(spec, occ, sc.start.translation(), sc.goal.translation()));
  }
  EXPECT_THROW((void)make_scenario("nope"), std::invalid_argument);
}

TEST(Scenarios, GeneratorsAreDeterministic) {
  const Scenario a = cluttered_scenario(11);
  const Scenario b = cluttered_scenario(11);
  ASSERT_EQ(a.environment.obstacle_count(), b.environment.obstacle_count());
  EXPECT_EQ(a.environment.clearance({15, 10}), b.environment.clearance({15, 10}));
  // Different seeds produce different mazes (compare full rasters, not a probe point).
  const geometry::GridSpec spec(48, 36, 0.5);
  EXPECT_NE(rasterize(maze_scenario(3).environment, spec, 0.3),
            rasterize(maze_scenario(4).environment, spec, 0.3));
  EXPECT_EQ(rasterize(maze_scenario(3).environment, spec, 0.3),
            rasterize(maze_scenario(3).environment, spec, 0.3));
}

TEST(Scenarios, RasterizeMarksObstacles) {
  geometry::Environment2D env({{0, 0}, {4, 4}});
  env.add_obstacle(geometry::Polygon::from_aabb({{1, 1}, {2, 2}}));
  const geometry::GridSpec spec(4, 4, 1.0);
  const auto occ = rasterize(env, spec);
  EXPECT_EQ(occ[spec.linear_index({1, 1})], 1);
  EXPECT_EQ(occ[spec.linear_index({0, 0})], 0);
  EXPECT_EQ(occ[spec.linear_index({3, 3})], 0);
}

}  // namespace
}  // namespace roboticscore::sim
