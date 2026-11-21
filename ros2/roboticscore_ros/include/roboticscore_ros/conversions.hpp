#pragma once

/// @file conversions.hpp
/// @brief ROS-independent conversions between ROS data layouts and RoboticsCore types.
///
/// Kept free of ROS headers so that the conversion logic can be unit tested without a ROS
/// installation; msg_conversions.hpp adapts these functions to the message types.

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/planning/grid/grid_map.hpp"

namespace roboticscore_ros {

/// How nav_msgs/OccupancyGrid cell values (-1 unknown, 0..100 occupancy probability in
/// percent) are mapped to blocked / free cells.
struct OccupancyThresholds {
  int occupied_threshold = 65;      ///< Values >= this are blocked.
  bool unknown_is_obstacle = true;  ///< Treat -1 (unknown) as blocked.
};

/// Converts a row-major occupancy array (x fastest, row 0 at the origin, i.e. the
/// nav_msgs/OccupancyGrid layout) into a planning grid. @p origin is the world position of
/// the corner of cell (0, 0); rotated map origins are not supported.
/// @throws std::invalid_argument if data.size() != width * height or the geometry is invalid.
[[nodiscard]] roboticscore::planning::GridMap occupancy_to_grid_map(
    int width, int height, double resolution, const roboticscore::Vec2& origin,
    std::span<const std::int8_t> data, const OccupancyThresholds& thresholds = {});

/// Yaw (rotation about z) of a unit quaternion, in [-pi, pi).
[[nodiscard]] double yaw_from_quaternion(double x, double y, double z, double w);

/// Unit quaternion (x, y, z, w) for a rotation of @p yaw about z.
[[nodiscard]] std::array<double, 4> quaternion_from_yaw(double yaw);

/// True if the quaternion is a pure rotation about z (within @p tolerance), i.e. it can be
/// represented by a planar pose without loss.
[[nodiscard]] bool is_planar_rotation(double x, double y, double z, double w,
                                      double tolerance = 1e-6);

/// Converts a planar velocity command for a car-like (bicycle) vehicle into a unicycle twist:
/// returns (linear velocity v, yaw rate v * tan(steering) / wheelbase).
/// @throws std::invalid_argument if wheelbase <= 0.
[[nodiscard]] roboticscore::Vec2 bicycle_to_twist(double speed, double steering, double wheelbase);

/// Removes consecutive duplicate points (closer than @p min_spacing metres) so that the
/// result can be used to build a trajectory::ReferencePath.
[[nodiscard]] std::vector<roboticscore::Vec2> deduplicate(
    const std::vector<roboticscore::Vec2>& points, double min_spacing = 1e-6);

}  // namespace roboticscore_ros
