#include "roboticscore/geometry/environment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "roboticscore/core/error.hpp"

namespace roboticscore::geometry {
namespace {

// Distance from p to the box (0 inside); used to skip obstacles that cannot matter.
double distance_to_box(const Vec2& p, const Aabb& box) {
  const double dx = std::max({box.min.x() - p.x(), 0.0, p.x() - box.max.x()});
  const double dy = std::max({box.min.y() - p.y(), 0.0, p.y() - box.max.y()});
  return std::hypot(dx, dy);
}

}  // namespace

Environment2D::Environment2D(const Aabb& bounds) : bounds_(bounds) {
  detail::require(bounds.max.x() > bounds.min.x() && bounds.max.y() > bounds.min.y(),
                  "Environment2D: bounds must have positive extent");
}

void Environment2D::add_obstacle(const Circle& circle) {
  detail::require(circle.radius > 0.0, "Environment2D: circle radius must be positive");
  circles_.push_back(circle);
}

void Environment2D::add_obstacle(const Polygon& polygon) {
  polygons_.push_back(polygon);
}

double Environment2D::clearance(const Vec2& p) const {
  double best = std::min({p.x() - bounds_.min.x(), bounds_.max.x() - p.x(), p.y() - bounds_.min.y(),
                          bounds_.max.y() - p.y()});
  for (const Circle& c : circles_) {
    best = std::min(best, signed_distance(p, c));
  }
  for (const Polygon& poly : polygons_) {
    if (best > 0.0 && distance_to_box(p, poly.bounding_box()) >= best) {
      continue;
    }
    best = std::min(best, signed_distance(p, poly));
  }
  return best;
}

bool Environment2D::point_free(const Vec2& p, double inflation) const {
  return clearance(p) > inflation;
}

bool Environment2D::segment_free(const Vec2& a, const Vec2& b, double inflation) const {
  // The bounds shrunk by the inflation are convex, so checking the endpoints suffices.
  const Aabb inner = bounds_.expanded(-inflation);
  if (!inner.contains(a) || !inner.contains(b)) {
    return false;
  }
  const Segment s{a, b};
  for (const Circle& c : circles_) {
    if (point_segment_distance(c.center, s) <= c.radius + inflation) {
      return false;
    }
  }
  const Aabb seg_box = Aabb{a.cwiseMin(b), a.cwiseMax(b)}.expanded(inflation);
  for (const Polygon& poly : polygons_) {
    if (!seg_box.overlaps(poly.bounding_box())) {
      continue;
    }
    if (segment_polygon_distance(s, poly) <= inflation) {
      return false;
    }
  }
  return true;
}

bool Environment2D::polygon_free(const Polygon& footprint) const {
  for (const Vec2& v : footprint.vertices()) {
    if (!bounds_.contains(v)) {
      return false;
    }
  }
  for (const Circle& c : circles_) {
    if (polygon_circle_intersect(footprint, c)) {
      return false;
    }
  }
  for (const Polygon& poly : polygons_) {
    if (polygons_intersect(footprint, poly)) {
      return false;
    }
  }
  return true;
}

std::optional<double> Environment2D::ray_cast(const Vec2& origin, double angle,
                                              double max_range) const {
  detail::require(max_range > 0.0, "Environment2D::ray_cast: max_range must be positive");
  if (!bounds_.contains(origin)) {
    return 0.0;
  }
  const Vec2 dir(std::cos(angle), std::sin(angle));
  // Exit distance through the boundary walls.
  double best = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 2; ++axis) {
    if (dir(axis) > 1e-15) {
      best = std::min(best, (bounds_.max(axis) - origin(axis)) / dir(axis));
    } else if (dir(axis) < -1e-15) {
      best = std::min(best, (bounds_.min(axis) - origin(axis)) / dir(axis));
    }
  }
  for (const Circle& c : circles_) {
    if (const auto t = geometry::ray_cast(origin, dir, c); t && *t < best) {
      best = *t;
    }
  }
  for (const Polygon& poly : polygons_) {
    if (distance_to_box(origin, poly.bounding_box()) >= best) {
      continue;
    }
    if (const auto t = geometry::ray_cast(origin, dir, poly); t && *t < best) {
      best = *t;
    }
  }
  if (best > max_range) {
    return std::nullopt;
  }
  return best;
}

}  // namespace roboticscore::geometry
