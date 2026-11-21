#pragma once

/// @file msg_conversions.hpp
/// @brief Conversions between ROS 2 messages and RoboticsCore types. All message-specific
///        code of the adapter lives here; the logic is in conversions.hpp.

#include <cmath>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/planning/grid/grid_map.hpp"

#include "roboticscore_ros/conversions.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

namespace roboticscore_ros {

/// Planar pose of a geometry_msgs/Pose (z and roll/pitch are ignored).
[[nodiscard]] inline roboticscore::geometry::SE2 to_se2(const geometry_msgs::msg::Pose& pose) {
  const auto& q = pose.orientation;
  return {pose.position.x, pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)};
}

/// geometry_msgs/Pose at z = 0 for a planar pose.
[[nodiscard]] inline geometry_msgs::msg::Pose to_pose_msg(const roboticscore::geometry::SE2& pose) {
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  const auto q = quaternion_from_yaw(pose.theta());
  msg.orientation.x = q[0];
  msg.orientation.y = q[1];
  msg.orientation.z = q[2];
  msg.orientation.w = q[3];
  return msg;
}

/// Planning grid from a nav_msgs/OccupancyGrid.
/// @throws std::invalid_argument if the map origin is rotated or the data size is wrong.
[[nodiscard]] inline roboticscore::planning::GridMap to_grid_map(
    const nav_msgs::msg::OccupancyGrid& map, const OccupancyThresholds& thresholds = {}) {
  const auto& q = map.info.origin.orientation;
  if (!is_planar_rotation(q.x, q.y, q.z, q.w)
      || std::abs(yaw_from_quaternion(q.x, q.y, q.z, q.w)) > 1e-6) {
    throw std::invalid_argument("to_grid_map: rotated map origins are not supported");
  }
  return occupancy_to_grid_map(
      static_cast<int>(map.info.width), static_cast<int>(map.info.height), map.info.resolution,
      {map.info.origin.position.x, map.info.origin.position.y},
      std::span<const std::int8_t>(map.data.data(), map.data.size()), thresholds);
}

/// nav_msgs/Path through @p points. Each pose's orientation points to the next point (the
/// last pose keeps the previous heading).
[[nodiscard]] inline nav_msgs::msg::Path to_path_msg(const std::vector<roboticscore::Vec2>& points,
                                                     const std::string& frame_id,
                                                     const builtin_interfaces::msg::Time& stamp) {
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  double heading = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i + 1 < points.size()) {
      const roboticscore::Vec2 d = points[i + 1] - points[i];
      heading = std::atan2(d.y(), d.x());
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose = to_pose_msg({points[i].x(), points[i].y(), heading});
    path.poses.push_back(pose);
  }
  return path;
}

/// Positions of the poses of a nav_msgs/Path.
[[nodiscard]] inline std::vector<roboticscore::Vec2> to_points(const nav_msgs::msg::Path& path) {
  std::vector<roboticscore::Vec2> points;
  points.reserve(path.poses.size());
  for (const auto& p : path.poses) {
    points.emplace_back(p.pose.position.x, p.pose.position.y);
  }
  return points;
}

}  // namespace roboticscore_ros
