#include "roboticscore_ros/conversions.hpp"

#include <cmath>
#include <stdexcept>

#include "roboticscore/math/angles.hpp"

namespace roboticscore_ros {

using roboticscore::Vec2;

roboticscore::planning::GridMap occupancy_to_grid_map(int width, int height, double resolution,
                                                      const Vec2& origin,
                                                      std::span<const std::int8_t> data,
                                                      const OccupancyThresholds& thresholds) {
  const roboticscore::geometry::GridSpec spec(width, height, resolution, origin);
  if (data.size() != spec.cell_count()) {
    throw std::invalid_argument("occupancy_to_grid_map: data size does not match width * height");
  }
  std::vector<std::uint8_t> blocked(data.size(), 0);
  for (std::size_t i = 0; i < data.size(); ++i) {
    const int v = data[i];
    const bool unknown = v < 0;
    blocked[i] = static_cast<std::uint8_t>(unknown ? thresholds.unknown_is_obstacle
                                                   : v >= thresholds.occupied_threshold);
  }
  return roboticscore::planning::GridMap(spec, std::move(blocked));
}

double yaw_from_quaternion(double x, double y, double z, double w) {
  // ZYX extraction of the yaw angle; exact for planar rotations.
  return roboticscore::math::wrap_to_pi(
      std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
}

std::array<double, 4> quaternion_from_yaw(double yaw) {
  return {0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw)};
}

bool is_planar_rotation(double x, double y, double z, double w, double tolerance) {
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  return norm > 0.0 && std::abs(x) / norm <= tolerance && std::abs(y) / norm <= tolerance;
}

Vec2 bicycle_to_twist(double speed, double steering, double wheelbase) {
  if (!(wheelbase > 0.0)) {
    throw std::invalid_argument("bicycle_to_twist: wheelbase must be positive");
  }
  return {speed, speed * std::tan(steering) / wheelbase};
}

std::vector<Vec2> deduplicate(const std::vector<Vec2>& points, double min_spacing) {
  std::vector<Vec2> out;
  out.reserve(points.size());
  for (const Vec2& p : points) {
    if (out.empty() || (p - out.back()).norm() > min_spacing) {
      out.push_back(p);
    }
  }
  return out;
}

}  // namespace roboticscore_ros
