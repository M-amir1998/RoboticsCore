#pragma once

/// @file environment.hpp
/// @brief Static 2D workspace made of a rectangular boundary plus circular and polygonal
///        obstacles. It is the single collision model shared by continuous-space planners,
///        local planners and simulated range sensors, which keeps benchmark comparisons fair.

#include <optional>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/collision.hpp"

namespace roboticscore::geometry {

/// Rectangular workspace with static obstacles. Queries are exact (no discretisation); a
/// robot with a circular footprint is handled by inflating queries by its radius.
class Environment2D {
 public:
  /// @throws std::invalid_argument if @p bounds is empty.
  explicit Environment2D(const Aabb& bounds);

  void add_obstacle(const Circle& circle);
  void add_obstacle(const Polygon& polygon);

  [[nodiscard]] const Aabb& bounds() const noexcept { return bounds_; }
  [[nodiscard]] const std::vector<Circle>& circles() const noexcept { return circles_; }
  [[nodiscard]] const std::vector<Polygon>& polygons() const noexcept { return polygons_; }
  [[nodiscard]] std::size_t obstacle_count() const noexcept {
    return circles_.size() + polygons_.size();
  }

  /// Distance from @p p to the nearest obstacle or boundary wall; negative inside an
  /// obstacle or outside the bounds.
  [[nodiscard]] double clearance(const Vec2& p) const;

  /// True if a disc of radius @p inflation centred at @p p is inside the bounds and touches
  /// no obstacle.
  [[nodiscard]] bool point_free(const Vec2& p, double inflation = 0.0) const;

  /// True if a disc of radius @p inflation swept along the segment a-b stays collision free.
  [[nodiscard]] bool segment_free(const Vec2& a, const Vec2& b, double inflation = 0.0) const;

  /// True if the polygon (e.g. a vehicle footprint) is inside the bounds and touches no
  /// obstacle.
  [[nodiscard]] bool polygon_free(const Polygon& footprint) const;

  /// Distance to the first obstacle or boundary hit by a ray from @p origin with heading
  /// @p angle, or nullopt when nothing is hit within @p max_range.
  [[nodiscard]] std::optional<double> ray_cast(const Vec2& origin, double angle,
                                               double max_range) const;

 private:
  Aabb bounds_;
  std::vector<Circle> circles_;
  std::vector<Polygon> polygons_;
};

}  // namespace roboticscore::geometry
