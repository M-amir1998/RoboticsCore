#include "roboticscore_ros/grid_planner_node.hpp"

#include <chrono>
#include <stdexcept>
#include <vector>

#include "roboticscore_ros/msg_conversions.hpp"

namespace roboticscore_ros {

using roboticscore::Vec2;
namespace planning = roboticscore::planning;

GridPlannerNode::GridPlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("grid_planner", options) {
  algorithm_ = declare_parameter<std::string>("algorithm", "astar");
  inflation_radius_ = declare_parameter<double>("inflation_radius", 0.3);
  const int occupied = static_cast<int>(declare_parameter<int>("occupied_threshold", 65));
  const bool unknown_blocked = declare_parameter<bool>("unknown_is_obstacle", true);
  if (algorithm_ != "astar" && algorithm_ != "dijkstra" && algorithm_ != "theta_star") {
    throw std::invalid_argument("grid_planner: algorithm must be astar, dijkstra or theta_star");
  }

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", latched);

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", latched,
      [this, occupied, unknown_blocked](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
        OccupancyThresholds thresholds;
        thresholds.occupied_threshold = occupied;
        thresholds.unknown_is_obstacle = unknown_blocked;
        try {
          grid_ = to_grid_map(*msg, thresholds).inflated(inflation_radius_);
          map_frame_ = msg->header.frame_id;
          RCLCPP_INFO(get_logger(), "map %ux%u @ %.3f m received", msg->info.width,
                      msg->info.height, msg->info.resolution);
          replan();
        } catch (const std::invalid_argument& e) {
          RCLCPP_ERROR(get_logger(), "rejected map: %s", e.what());
        }
      });
  start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
        on_start(msg);
      });
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) { on_goal(msg); });
}

void GridPlannerNode::on_start(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
  if (!map_frame_.empty() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN(get_logger(), "start pose in frame '%s', expected '%s'; ignored",
                msg->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  start_ = Vec2(msg->pose.pose.position.x, msg->pose.pose.position.y);
  replan();
}

void GridPlannerNode::on_goal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
  if (!map_frame_.empty() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN(get_logger(), "goal pose in frame '%s', expected '%s'; ignored",
                msg->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  goal_ = Vec2(msg->pose.position.x, msg->pose.position.y);
  replan();
}

planning::GridPath GridPlannerNode::plan(const Vec2& start, const Vec2& goal) const {
  if (!grid_) {
    return {};
  }
  const auto& spec = grid_->spec();
  const auto s = spec.world_to_cell(start);
  const auto g = spec.world_to_cell(goal);
  if (algorithm_ == "dijkstra") {
    return planning::dijkstra(*grid_, s, g);
  }
  if (algorithm_ == "theta_star") {
    return planning::theta_star(*grid_, s, g);
  }
  return planning::astar(*grid_, s, g);  // 8-connected, octile heuristic
}

void GridPlannerNode::replan() {
  if (!grid_ || !start_ || !goal_) {
    return;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const planning::GridPath result = plan(*start_, *goal_);
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  std::vector<Vec2> points;
  if (result.found()) {
    points = planning::to_world(grid_->spec(), result.cells);
    // Replace the first and last cell centres by the exact start and goal positions.
    points.front() = *start_;
    points.back() = *goal_;
    RCLCPP_INFO(get_logger(), "%s: %.2f m, %zu expanded, %.2f ms", algorithm_.c_str(), result.cost,
                result.expanded, ms);
  } else {
    RCLCPP_WARN(get_logger(), "%s: no path (status %d)", algorithm_.c_str(),
                static_cast<int>(result.status));
  }
  path_pub_->publish(to_path_msg(deduplicate(points), map_frame_, now()));
}

}  // namespace roboticscore_ros
