#pragma once

/// @file collision.hpp
/// @brief Exact 2D collision and distance primitives for points, segments, circles,
///        axis-aligned boxes and simple (possibly non-convex) polygons.
///
/// Conventions: points on a boundary count as inside/intersecting (closed sets). Distances
/// are Euclidean and zero for intersecting shapes unless stated otherwise.

#include <optional>
#include <vector>

#include "roboticscore/core/types.hpp"

namespace roboticscore::geometry {

/// Closed disc.
struct Circle {
  Vec2 center = Vec2::Zero();
  double radius = 0.0;
};

/// Closed line segment from a to b.
struct Segment {
  Vec2 a = Vec2::Zero();
  Vec2 b = Vec2::Zero();
};

/// Closed axis-aligned bounding box.
struct Aabb {
  Vec2 min = Vec2::Zero();
  Vec2 max = Vec2::Zero();

  [[nodiscard]] bool contains(const Vec2& p) const {
    return p.x() >= min.x() && p.x() <= max.x() && p.y() >= min.y() && p.y() <= max.y();
  }
  [[nodiscard]] bool overlaps(const Aabb& o) const {
    return min.x() <= o.max.x() && o.min.x() <= max.x() && min.y() <= o.max.y()
           && o.min.y() <= max.y();
  }
  /// Box grown by @p margin on every side.
  [[nodiscard]] Aabb expanded(double margin) const {
    return {min - Vec2::Constant(margin), max + Vec2::Constant(margin)};
  }
  [[nodiscard]] Vec2 size() const { return max - min; }
};

/// Simple polygon (non-self-intersecting), stored counter-clockwise.
class Polygon {
 public:
  Polygon() = default;

  /// @throws std::invalid_argument for fewer than three vertices or zero area.
  explicit Polygon(std::vector<Vec2> vertices);

  /// Rectangle of the given length (along yaw) and width centred at @p center.
  [[nodiscard]] static Polygon rectangle(const Vec2& center, double length, double width,
                                         double yaw = 0.0);

  /// Axis-aligned box as a polygon.
  [[nodiscard]] static Polygon from_aabb(const Aabb& box);

  [[nodiscard]] const std::vector<Vec2>& vertices() const noexcept { return vertices_; }
  [[nodiscard]] std::size_t size() const noexcept { return vertices_.size(); }
  [[nodiscard]] Segment edge(std::size_t i) const {
    return {vertices_[i], vertices_[(i + 1) % vertices_.size()]};
  }
  [[nodiscard]] const Aabb& bounding_box() const noexcept { return bbox_; }
  /// Positive signed area (vertices are stored counter-clockwise).
  [[nodiscard]] double area() const;
  [[nodiscard]] Vec2 centroid() const;

 private:
  std::vector<Vec2> vertices_;
  Aabb bbox_;
};

/// Twice the signed area of triangle (a, b, c): > 0 for a counter-clockwise turn.
[[nodiscard]] inline double orientation(const Vec2& a, const Vec2& b, const Vec2& c) {
  return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

/// Closest point on @p s to @p p.
[[nodiscard]] Vec2 closest_point_on_segment(const Vec2& p, const Segment& s);

/// Euclidean distance between a point and a segment.
[[nodiscard]] double point_segment_distance(const Vec2& p, const Segment& s);

/// True if two closed segments share at least one point (handles collinear overlap).
[[nodiscard]] bool segments_intersect(const Segment& s1, const Segment& s2);

/// Minimum distance between two segments (0 if they intersect).
[[nodiscard]] double segment_segment_distance(const Segment& s1, const Segment& s2);

/// Point-in-polygon test (crossing number); boundary points count as inside.
[[nodiscard]] bool point_in_polygon(const Vec2& p, const Polygon& poly);

/// Signed distance from a point to the polygon boundary: negative inside, positive outside.
[[nodiscard]] double signed_distance(const Vec2& p, const Polygon& poly);

/// Signed distance from a point to a circle boundary: negative inside.
[[nodiscard]] inline double signed_distance(const Vec2& p, const Circle& c) {
  return (p - c.center).norm() - c.radius;
}

/// Distance between a segment and a polygon (0 if they touch or the segment is inside).
[[nodiscard]] double segment_polygon_distance(const Segment& s, const Polygon& poly);

/// Distance between a segment and a circle boundary (0 if they touch or overlap).
[[nodiscard]] double segment_circle_distance(const Segment& s, const Circle& c);

/// True if the two closed polygons overlap (edge crossings or containment). O(n m).
[[nodiscard]] bool polygons_intersect(const Polygon& a, const Polygon& b);

/// True if the closed polygon and disc overlap.
[[nodiscard]] bool polygon_circle_intersect(const Polygon& poly, const Circle& c);

/// Distance t >= 0 along the unit direction @p dir from @p origin to the first boundary hit,
/// if any. The origin being inside the shape yields t = 0.
[[nodiscard]] std::optional<double> ray_cast(const Vec2& origin, const Vec2& dir, const Circle& c);
[[nodiscard]] std::optional<double> ray_cast(const Vec2& origin, const Vec2& dir, const Segment& s);
[[nodiscard]] std::optional<double> ray_cast(const Vec2& origin, const Vec2& dir,
                                             const Polygon& poly);

}  // namespace roboticscore::geometry
