#pragma once

/// @file world.hpp
/// @brief Simulated world: static workspace, moving circular obstacles and point landmarks.

#include <optional>
#include <utility>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/collision.hpp"
#include "roboticscore/geometry/environment.hpp"

namespace roboticscore::sim {

/// Circular obstacle moving with constant velocity and reflecting off the workspace bounds.
struct DynamicObstacle {
  geometry::Circle shape;
  Vec2 velocity = Vec2::Zero();

  /// Advances the obstacle by @p dt, reflecting the velocity at the bounds.
  void advance(double dt, const geometry::Aabb& bounds);
};

/// Everything the simulated robot can perceive or collide with.
struct World {
  explicit World(geometry::Environment2D env) : environment(std::move(env)) {}

  geometry::Environment2D environment;
  std::vector<DynamicObstacle> dynamic_obstacles;
  std::vector<Vec2> landmarks;

  /// Distance from @p p to the nearest static or dynamic obstacle (or wall).
  [[nodiscard]] double clearance(const Vec2& p) const;

  /// True if a disc of radius @p radius at @p p touches nothing.
  [[nodiscard]] bool point_free(const Vec2& p, double radius) const {
    return clearance(p) > radius;
  }

  /// First hit along a ray, including dynamic obstacles; nullopt beyond @p max_range.
  [[nodiscard]] std::optional<double> ray_cast(const Vec2& origin, double angle,
                                               double max_range) const;

  /// Advances all dynamic obstacles.
  void advance(double dt);
};

}  // namespace roboticscore::sim
