// Closed-loop test of both nodes: a map with a wall is published, the planner plans around
// it, and the tracker drives a simulated unicycle (integrating cmd_vel, publishing odom) to
// the goal.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include <gtest/gtest.h>

#include "roboticscore_ros/grid_planner_node.hpp"
#include "roboticscore_ros/msg_conversions.hpp"
#include "roboticscore_ros/path_tracker_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using namespace std::chrono_literals;

class RosTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// 10 m x 6 m map at 0.1 m with a wall at x = 5 m from y = 0 to y = 4 m.
nav_msgs::msg::OccupancyGrid make_map() {
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.1F;
  map.info.width = 100;
  map.info.height = 60;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(100 * 60, 0);
  for (int y = 0; y < 40; ++y) {
    for (int x = 49; x < 51; ++x) {
      map.data[static_cast<std::size_t>(y * 100 + x)] = 100;
    }
  }
  return map;
}

TEST_F(RosTest, PlannerAvoidsWallAndTrackerReachesGoal) {
  rclcpp::NodeOptions tracker_options;
  tracker_options.parameter_overrides({{"max_speed", 1.0}, {"wheelbase", 0.3}});
  auto planner = std::make_shared<roboticscore_ros::GridPlannerNode>();
  auto tracker = std::make_shared<roboticscore_ros::PathTrackerNode>(tracker_options);
  auto harness = std::make_shared<rclcpp::Node>("harness");

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  auto map_pub = harness->create_publisher<nav_msgs::msg::OccupancyGrid>("map", latched);
  auto start_pub =
      harness->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
  auto goal_pub = harness->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
  auto odom_pub = harness->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  std::optional<nav_msgs::msg::Path> plan;
  auto plan_sub = harness->create_subscription<nav_msgs::msg::Path>(
      "plan", latched, [&](const nav_msgs::msg::Path::ConstSharedPtr& msg) { plan = *msg; });

  // Simulated unicycle at (1, 1) facing +x.
  double x = 1.0;
  double y = 1.0;
  double yaw = 0.0;
  geometry_msgs::msg::Twist cmd;
  auto cmd_sub = harness->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [&](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) { cmd = *msg; });
  constexpr double kDt = 0.02;
  auto sim_timer = harness->create_wall_timer(20ms, [&] {
    x += cmd.linear.x * std::cos(yaw) * kDt;
    y += cmd.linear.x * std::sin(yaw) * kDt;
    yaw += cmd.angular.z * kDt;
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.pose.pose = roboticscore_ros::to_pose_msg({x, y, yaw});
    odom.twist.twist.linear.x = cmd.linear.x;
    odom_pub->publish(odom);
  });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(tracker);
  executor.add_node(harness);

  map_pub->publish(make_map());
  executor.spin_some(200ms);
  geometry_msgs::msg::PoseWithCovarianceStamped start;
  start.header.frame_id = "map";
  start.pose.pose = roboticscore_ros::to_pose_msg({x, y, yaw});
  start_pub->publish(start);
  executor.spin_some(200ms);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose = roboticscore_ros::to_pose_msg({9.0, 1.0, 0.0});
  goal_pub->publish(goal);

  // Wait for the plan.
  const auto deadline_plan = std::chrono::steady_clock::now() + 10s;
  while (!plan && std::chrono::steady_clock::now() < deadline_plan) {
    executor.spin_some(50ms);
  }
  ASSERT_TRUE(plan.has_value());
  ASSERT_GE(plan->poses.size(), 2U);
  // The path must go around the top of the wall (wall ends at y = 4 m, inflated by 0.3 m).
  double max_y = 0.0;
  for (const auto& p : plan->poses) {
    max_y = std::max(max_y, p.pose.position.y);
  }
  EXPECT_GT(max_y, 4.0);
  EXPECT_NEAR(plan->poses.back().pose.position.x, 9.0, 1e-9);

  // Drive until the tracker reports completion.
  const auto deadline_drive = std::chrono::steady_clock::now() + 90s;
  executor.spin_some(100ms);
  while (tracker->active() && std::chrono::steady_clock::now() < deadline_drive) {
    executor.spin_some(20ms);
  }
  EXPECT_FALSE(tracker->active());
  EXPECT_LT(std::hypot(x - 9.0, y - 1.0), 0.5);
}

TEST_F(RosTest, PlannerPublishesEmptyPathWhenGoalUnreachable) {
  auto planner = std::make_shared<roboticscore_ros::GridPlannerNode>();
  auto map = make_map();
  // Close the gap above the wall.
  for (int yy = 40; yy < 60; ++yy) {
    map.data[static_cast<std::size_t>(yy * 100 + 50)] = 100;
  }
  auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>(map);
  // Use the planning API directly on a node that received the map via a publisher.
  auto harness = std::make_shared<rclcpp::Node>("harness2");
  auto map_pub = harness->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(harness);
  map_pub->publish(*msg);
  for (int i = 0; i < 20; ++i) {
    executor.spin_some(20ms);
  }
  const auto result = planner->plan({1.0, 1.0}, {9.0, 1.0});
  EXPECT_FALSE(result.found());
  const auto ok = planner->plan({1.0, 1.0}, {3.0, 5.0});
  EXPECT_TRUE(ok.found());
}

}  // namespace
