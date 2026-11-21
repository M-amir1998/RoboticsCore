#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore_ros/conversions.hpp"

using roboticscore::Vec2;
using namespace roboticscore_ros;

TEST(Conversions, OccupancyLayoutAndThresholds) {
  // 3 x 2 grid, row-major with x fastest: row 0 = {0, 100, -1}, row 1 = {64, 65, 0}.
  const std::vector<std::int8_t> data = {0, 100, -1, 64, 65, 0};
  const auto map = occupancy_to_grid_map(3, 2, 0.5, Vec2(-1.0, 2.0), data);
  EXPECT_FALSE(map.blocked({0, 0}));
  EXPECT_TRUE(map.blocked({1, 0}));
  EXPECT_TRUE(map.blocked({2, 0}));  // unknown counts as obstacle by default
  EXPECT_FALSE(map.blocked({0, 1}));
  EXPECT_TRUE(map.blocked({1, 1}));
  EXPECT_FALSE(map.blocked({2, 1}));
  const Vec2 c = map.spec().cell_center({1, 1});
  EXPECT_DOUBLE_EQ(c.x(), -0.25);
  EXPECT_DOUBLE_EQ(c.y(), 2.75);

  OccupancyThresholds lenient;
  lenient.unknown_is_obstacle = false;
  lenient.occupied_threshold = 101;
  const auto free_map = occupancy_to_grid_map(3, 2, 0.5, Vec2::Zero(), data, lenient);
  for (int x = 0; x < 3; ++x) {
    EXPECT_FALSE(free_map.blocked({x, 0}));
  }
}

TEST(Conversions, OccupancyRejectsWrongSize) {
  const std::vector<std::int8_t> data(5, 0);
  EXPECT_THROW((void)occupancy_to_grid_map(3, 2, 0.5, Vec2::Zero(), data), std::invalid_argument);
}

TEST(Conversions, QuaternionYawRoundTrip) {
  for (double yaw = -3.1; yaw < 3.14; yaw += 0.37) {
    const auto q = quaternion_from_yaw(yaw);
    EXPECT_NEAR(std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3])), 1.0, 1e-12);
    EXPECT_NEAR(yaw_from_quaternion(q[0], q[1], q[2], q[3]), yaw, 1e-12);
    EXPECT_TRUE(is_planar_rotation(q[0], q[1], q[2], q[3]));
  }
  // pi wraps to -pi.
  const auto q = quaternion_from_yaw(std::numbers::pi);
  EXPECT_NEAR(yaw_from_quaternion(q[0], q[1], q[2], q[3]), -std::numbers::pi, 1e-12);
  // 90 degree roll is not planar.
  EXPECT_FALSE(
      is_planar_rotation(std::sin(std::numbers::pi / 4), 0.0, 0.0, std::cos(std::numbers::pi / 4)));
}

TEST(Conversions, BicycleToTwist) {
  const Vec2 t = bicycle_to_twist(2.0, std::atan(0.25), 0.5);
  EXPECT_DOUBLE_EQ(t.x(), 2.0);
  EXPECT_NEAR(t.y(), 1.0, 1e-12);  // v tan(delta) / L = 2 * 0.25 / 0.5
  EXPECT_THROW((void)bicycle_to_twist(1.0, 0.1, 0.0), std::invalid_argument);
}

TEST(Conversions, Deduplicate) {
  const std::vector<Vec2> pts = {{0, 0}, {0, 0}, {1, 0}, {1, 1e-9}, {2, 0}};
  const auto out = deduplicate(pts, 1e-6);
  ASSERT_EQ(out.size(), 3U);
  EXPECT_DOUBLE_EQ(out[2].x(), 2.0);
}
