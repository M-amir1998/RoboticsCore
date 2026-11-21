#pragma once

/// @file grid_planner_node.hpp
/// @brief ROS 2 node that plans on a nav_msgs/OccupancyGrid with RoboticsCore grid planners.
///
/// Subscriptions:
///   map          nav_msgs/OccupancyGrid               (transient local, e.g. from map_server)
///   initialpose  geometry_msgs/PoseWithCovarianceStamped  start pose (RViz "2D Pose Estimate")
///   goal_pose    geometry_msgs/PoseStamped              goal pose  (RViz "2D Goal Pose")
/// Publications:
///   plan         nav_msgs/Path (transient local); an empty path if no path exists
/// Parameters:
///   algorithm            "astar" | "dijkstra" | "theta_star"     (default "astar")
///   inflation_radius     obstacle inflation [m]                   (default 0.3)
///   occupied_threshold   occupancy value counted as blocked       (default 65)
///   unknown_is_obstacle  treat unknown (-1) cells as blocked      (default true)
///
/// Start and goal must be expressed in the map frame; there is no TF lookup.

#include <optional>
#include <string>

#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/planning/grid/grid_map.hpp"
#include "roboticscore/planning/grid/grid_search.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace roboticscore_ros {

class GridPlannerNode : public rclcpp::Node {
 public:
  explicit GridPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// Plans from @p start to @p goal on the current (inflated) map. Exposed for testing.
  [[nodiscard]] roboticscore::planning::GridPath plan(const roboticscore::Vec2& start,
                                                      const roboticscore::Vec2& goal) const;

 private:
  void on_start(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);
  void on_goal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
  void replan();

  std::string algorithm_;
  double inflation_radius_ = 0.3;

  std::optional<roboticscore::planning::GridMap> grid_;
  std::string map_frame_;
  std::optional<roboticscore::Vec2> start_;
  std::optional<roboticscore::Vec2> goal_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace roboticscore_ros
