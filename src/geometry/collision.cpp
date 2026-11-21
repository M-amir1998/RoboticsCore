#include "roboticscore/geometry/collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "roboticscore/core/error.hpp"

namespace roboticscore::geometry {
namespace {

double cross(const Vec2& a, const Vec2& b) {
  return a.x() * b.y() - a.y() * b.x();
}

// True if q lies within the bounding box of segment s (used for collinear cases).
bool on_segment_box(const Vec2& q, const Segment& s) {
  return q.x() <= std::max(s.a.x(), s.b.x()) && q.x() >= std::min(s.a.x(), s.b.x())
         && q.y() <= std::max(s.a.y(), s.b.y()) && q.y() >= std::min(s.a.y(), s.b.y());
}

int sign(double v) {
  if (v > 0.0) return 1;
  if (v < 0.0) return -1;
  return 0;
}

}  // namespace

// ----------------------------------------------------------------------------- Polygon

Polygon::Polygon(std::vector<Vec2> vertices) : vertices_(std::move(vertices)) {
  detail::require(vertices_.size() >= 3, "Polygon: at least three vertices are required");
  double twice_area = 0.0;
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    twice_area += cross(vertices_[i], vertices_[(i + 1) % vertices_.size()]);
  }
  detail::require(std::abs(twice_area) > 1e-12, "Polygon: polygon must have non-zero area");
  if (twice_area < 0.0) {
    std::reverse(vertices_.begin(), vertices_.end());
  }
  bbox_.min = vertices_.front();
  bbox_.max = vertices_.front();
  for (const Vec2& v : vertices_) {
    bbox_.min = bbox_.min.cwiseMin(v);
    bbox_.max = bbox_.max.cwiseMax(v);
  }
}

Polygon Polygon::rectangle(const Vec2& center, double length, double width, double yaw) {
  detail::require(length > 0.0 && width > 0.0, "Polygon::rectangle: dimensions must be positive");
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const Vec2 ex(c * 0.5 * length, s * 0.5 * length);
  const Vec2 ey(-s * 0.5 * width, c * 0.5 * width);
  return Polygon({center - ex - ey, center + ex - ey, center + ex + ey, center - ex + ey});
}

Polygon Polygon::from_aabb(const Aabb& box) {
  return Polygon(
      {box.min, Vec2(box.max.x(), box.min.y()), box.max, Vec2(box.min.x(), box.max.y())});
}

double Polygon::area() const {
  double twice_area = 0.0;
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    twice_area += cross(vertices_[i], vertices_[(i + 1) % vertices_.size()]);
  }
  return 0.5 * twice_area;
}

Vec2 Polygon::centroid() const {
  Vec2 c = Vec2::Zero();
  double twice_area = 0.0;
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    const Vec2& p = vertices_[i];
    const Vec2& q = vertices_[(i + 1) % vertices_.size()];
    const double w = cross(p, q);
    c += w * (p + q);
    twice_area += w;
  }
  return c / (3.0 * twice_area);
}

// ----------------------------------------------------------------------------- distances

Vec2 closest_point_on_segment(const Vec2& p, const Segment& s) {
  const Vec2 d = s.b - s.a;
  const double len2 = d.squaredNorm();
  if (len2 <= 0.0) {
    return s.a;
  }
  const double t = std::clamp((p - s.a).dot(d) / len2, 0.0, 1.0);
  return s.a + t * d;
}

double point_segment_distance(const Vec2& p, const Segment& s) {
  return (p - closest_point_on_segment(p, s)).norm();
}

bool segments_intersect(const Segment& s1, const Segment& s2) {
  const int o1 = sign(orientation(s1.a, s1.b, s2.a));
  const int o2 = sign(orientation(s1.a, s1.b, s2.b));
  const int o3 = sign(orientation(s2.a, s2.b, s1.a));
  const int o4 = sign(orientation(s2.a, s2.b, s1.b));
  if (o1 != o2 && o3 != o4) {
    return true;
  }
  return (o1 == 0 && on_segment_box(s2.a, s1)) || (o2 == 0 && on_segment_box(s2.b, s1))
         || (o3 == 0 && on_segment_box(s1.a, s2)) || (o4 == 0 && on_segment_box(s1.b, s2));
}

double segment_segment_distance(const Segment& s1, const Segment& s2) {
  if (segments_intersect(s1, s2)) {
    return 0.0;
  }
  return std::min({point_segment_distance(s1.a, s2), point_segment_distance(s1.b, s2),
                   point_segment_distance(s2.a, s1), point_segment_distance(s2.b, s1)});
}

bool point_in_polygon(const Vec2& p, const Polygon& poly) {
  if (!poly.bounding_box().contains(p)) {
    return false;
  }
  const auto& v = poly.vertices();
  bool inside = false;
  for (std::size_t i = 0, j = v.size() - 1; i < v.size(); j = i++) {
    if (point_segment_distance(p, {v[j], v[i]}) <= 1e-12) {
      return true;  // on the boundary
    }
    if ((v[i].y() > p.y()) != (v[j].y() > p.y())) {
      const double x_cross =
          v[j].x() + (p.y() - v[j].y()) * (v[i].x() - v[j].x()) / (v[i].y() - v[j].y());
      if (p.x() < x_cross) {
        inside = !inside;
      }
    }
  }
  return inside;
}

double signed_distance(const Vec2& p, const Polygon& poly) {
  double d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < poly.size(); ++i) {
    d = std::min(d, point_segment_distance(p, poly.edge(i)));
  }
  return point_in_polygon(p, poly) ? -d : d;
}

double segment_polygon_distance(const Segment& s, const Polygon& poly) {
  if (point_in_polygon(s.a, poly) || point_in_polygon(s.b, poly)) {
    return 0.0;
  }
  double d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < poly.size(); ++i) {
    d = std::min(d, segment_segment_distance(s, poly.edge(i)));
    if (d == 0.0) {
      break;
    }
  }
  return d;
}

double segment_circle_distance(const Segment& s, const Circle& c) {
  return std::max(0.0, point_segment_distance(c.center, s) - c.radius);
}

bool polygons_intersect(const Polygon& a, const Polygon& b) {
  if (!a.bounding_box().overlaps(b.bounding_box())) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    for (std::size_t j = 0; j < b.size(); ++j) {
      if (segments_intersect(a.edge(i), b.edge(j))) {
        return true;
      }
    }
  }
  // No edge crossings: either one contains the other or they are disjoint.
  return point_in_polygon(a.vertices().front(), b) || point_in_polygon(b.vertices().front(), a);
}

bool polygon_circle_intersect(const Polygon& poly, const Circle& c) {
  if (!poly.bounding_box().expanded(c.radius).contains(c.center)) {
    return false;
  }
  if (point_in_polygon(c.center, poly)) {
    return true;
  }
  for (std::size_t i = 0; i < poly.size(); ++i) {
    if (point_segment_distance(c.center, poly.edge(i)) <= c.radius) {
      return true;
    }
  }
  return false;
}

// ----------------------------------------------------------------------------- ray casting

std::optional<double> ray_cast(const Vec2& origin, const Vec2& dir, const Circle& c) {
  const Vec2 oc = origin - c.center;
  const double cc = oc.squaredNorm() - c.radius * c.radius;
  if (cc <= 0.0) {
    return 0.0;  // origin inside the disc
  }
  const double b = dir.dot(oc);
  const double disc = b * b - cc;
  if (disc < 0.0) {
    return std::nullopt;
  }
  const double t = -b - std::sqrt(disc);
  if (t < 0.0) {
    return std::nullopt;
  }
  return t;
}

std::optional<double> ray_cast(const Vec2& origin, const Vec2& dir, const Segment& s) {
  const Vec2 e = s.b - s.a;
  const Vec2 ao = s.a - origin;
  const double denom = cross(dir, e);
  if (std::abs(denom) < 1e-15) {
    if (std::abs(cross(ao, dir)) > 1e-12) {
      return std::nullopt;  // parallel, not collinear
    }
    // Collinear: the nearest endpoint that is not behind the origin.
    const double ta = ao.dot(dir);
    const double tb = (s.b - origin).dot(dir);
    if (ta < 0.0 && tb < 0.0) {
      return std::nullopt;
    }
    if (ta < 0.0 || tb < 0.0) {
      return 0.0;  // origin lies on the segment
    }
    return std::min(ta, tb);
  }
  const double t = cross(ao, e) / denom;
  const double u = cross(ao, dir) / denom;
  if (t < 0.0 || u < 0.0 || u > 1.0) {
    return std::nullopt;
  }
  return t;
}

std::optional<double> ray_cast(const Vec2& origin, const Vec2& dir, const Polygon& poly) {
  if (point_in_polygon(origin, poly)) {
    return 0.0;
  }
  std::optional<double> best;
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const auto t = ray_cast(origin, dir, poly.edge(i));
    if (t && (!best || *t < *best)) {
      best = t;
    }
  }
  return best;
}

}  // namespace roboticscore::geometry
