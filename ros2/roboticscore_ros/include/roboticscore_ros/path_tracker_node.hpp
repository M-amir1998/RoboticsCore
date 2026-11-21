#pragma once

/// @file path_tracker_node.hpp
/// @brief ROS 2 node that follows a nav_msgs/Path with RoboticsCore's Pure Pursuit tracker.
///
/// The tracker works on a kinematic bicycle model. Its steering command delta is converted
/// into a twist (v, v * tan(delta) / wheelbase), so the node also drives differential-drive
/// robots; choose a small virtual wheelbase for them.
///
/// Subscriptions:
///   plan     nav_msgs/Path (transient local); an empty path stops the robot
///   odom     nav_msgs/Odometry, pose in the path frame, twist.linear.x = forward speed
/// Publications:
///   cmd_vel  geometry_msgs/Twist at control_rate; zero once the end of the path is reached
/// Parameters (defaults in parentheses):
///   control_rate [Hz] (20), wheelbase [m] (0.5), max_steer [rad] (0.6),
///   max_speed [m/s] (0.5), max_accel [m/s^2] (1.0), max_lateral_accel [m/s^2] (0.5),
///   lookahead_min [m] (0.4), lookahead_gain [s] (0.5), path_spacing [m] (0.05)

#include <memory>
#include <optional>

#include "roboticscore/control/geometric_trackers.hpp"
#include "roboticscore/models/bicycle.hpp"
#include "roboticscore/trajectory/reference_path.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace roboticscore_ros {

class PathTrackerNode : public rclcpp::Node {
 public:
  explicit PathTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// True while a path is being followed.
  [[nodiscard]] bool active() const noexcept { return path_.has_value(); }

 private:
  void on_path(const nav_msgs::msg::Path::ConstSharedPtr& msg);
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void on_timer();
  void stop();

  roboticscore::models::BicycleParams vehicle_;
  double dt_ = 0.05;
  double max_lateral_accel_ = 0.5;
  double path_spacing_ = 0.05;

  std::unique_ptr<roboticscore::control::PurePursuitTracker> tracker_;
  std::optional<roboticscore::trajectory::ReferencePath> path_;
  std::optional<roboticscore::models::BicycleState> state_;
  double commanded_speed_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace roboticscore_ros
