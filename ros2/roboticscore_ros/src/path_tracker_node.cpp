#include "roboticscore_ros/path_tracker_node.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <vector>

#include "roboticscore/trajectory/time_parameterization.hpp"

#include "roboticscore_ros/msg_conversions.hpp"

namespace roboticscore_ros {

using roboticscore::Vec2;
namespace trajectory = roboticscore::trajectory;

PathTrackerNode::PathTrackerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("path_tracker", options) {
  const double rate = declare_parameter<double>("control_rate", 20.0);
  vehicle_.wheelbase = declare_parameter<double>("wheelbase", 0.5);
  vehicle_.max_steer = declare_parameter<double>("max_steer", 0.6);
  vehicle_.max_speed = declare_parameter<double>("max_speed", 0.5);
  vehicle_.max_accel = declare_parameter<double>("max_accel", 1.0);
  max_lateral_accel_ = declare_parameter<double>("max_lateral_accel", 0.5);
  path_spacing_ = declare_parameter<double>("path_spacing", 0.05);
  roboticscore::control::PurePursuitConfig pp;
  pp.lookahead_min = declare_parameter<double>("lookahead_min", 0.4);
  pp.lookahead_gain = declare_parameter<double>("lookahead_gain", 0.5);
  pp.lookahead_offset = 0.0;
  if (!(rate > 0.0)) {
    throw std::invalid_argument("path_tracker: control_rate must be positive");
  }
  dt_ = 1.0 / rate;
  tracker_ = std::make_unique<roboticscore::control::PurePursuitTracker>(vehicle_, pp);

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::Path::ConstSharedPtr& msg) { on_path(msg); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { on_odom(msg); });
  timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] { on_timer(); });
}

void PathTrackerNode::on_path(const nav_msgs::msg::Path::ConstSharedPtr& msg) {
  const std::vector<Vec2> points = deduplicate(to_points(*msg), 1e-3);
  if (points.size() < 2) {
    RCLCPP_INFO(get_logger(), "empty or single-point path: stopping");
    stop();
    return;
  }
  try {
    // Spline through the waypoints, then a speed profile limited by speed, acceleration and
    // lateral acceleration that starts and ends at rest.
    const auto geometric = trajectory::ReferencePath::from_waypoints(points, path_spacing_, 1.0);
    std::vector<double> s;
    std::vector<double> kappa;
    for (const auto& p : geometric.points()) {
      s.push_back(p.s);
      kappa.push_back(p.curvature);
    }
    const auto profile = trajectory::compute_speed_profile(s, kappa, vehicle_.max_speed,
                                                           vehicle_.max_accel, max_lateral_accel_);
    std::vector<trajectory::PathPoint> timed = geometric.points();
    for (std::size_t i = 0; i < timed.size(); ++i) {
      timed[i].speed = profile.v[i];
    }
    path_.emplace(std::move(timed));
    tracker_->reset();
    commanded_speed_ = state_ ? (*state_)(3) : 0.0;
    RCLCPP_INFO(get_logger(), "following path of %.2f m", path_->length());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "rejected path: %s", e.what());
    stop();
  }
}

void PathTrackerNode::on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  const auto pose = to_se2(msg->pose.pose);
  state_ = roboticscore::models::BicycleState(pose.translation().x(), pose.translation().y(),
                                              pose.theta(), msg->twist.twist.linear.x);
}

void PathTrackerNode::on_timer() {
  if (!path_ || !state_) {
    return;
  }
  const auto command = tracker_->compute(*state_, *path_, dt_);
  if (tracker_->status().finished) {
    RCLCPP_INFO(get_logger(), "goal reached (cross-track error %.3f m)",
                tracker_->status().cross_track_error);
    stop();
    return;
  }
  // Integrate the commanded acceleration into a speed command for velocity-controlled bases.
  commanded_speed_ = std::clamp(commanded_speed_ + command.acceleration * dt_, -vehicle_.max_speed,
                                vehicle_.max_speed);
  const Vec2 twist = bicycle_to_twist(commanded_speed_, command.steering, vehicle_.wheelbase);
  geometry_msgs::msg::Twist msg;
  msg.linear.x = twist.x();
  msg.angular.z = twist.y();
  cmd_pub_->publish(msg);
}

void PathTrackerNode::stop() {
  path_.reset();
  commanded_speed_ = 0.0;
  cmd_pub_->publish(geometry_msgs::msg::Twist());
}

}  // namespace roboticscore_ros
