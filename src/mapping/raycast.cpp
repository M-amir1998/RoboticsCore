#include "roboticscore/mapping/raycast.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace roboticscore::mapping {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Liang-Barsky clipping of p0 + t (p1 - p0), t in [0, 1], against [0, w] x [0, h].
bool clip_segment(Vec2& p0, Vec2& p1, double w, double h) {
  const Vec2 d = p1 - p0;
  double t0 = 0.0;
  double t1 = 1.0;
  const double p[4] = {-d.x(), d.x(), -d.y(), d.y()};
  const double q[4] = {p0.x(), w - p0.x(), p0.y(), h - p0.y()};
  for (int i = 0; i < 4; ++i) {
    if (p[i] == 0.0) {
      if (q[i] < 0.0) {
        return false;
      }
      continue;
    }
    const double r = q[i] / p[i];
    if (p[i] < 0.0) {
      t0 = std::max(t0, r);
    } else {
      t1 = std::min(t1, r);
    }
  }
  if (t0 > t1) {
    return false;
  }
  const Vec2 start = p0 + t0 * d;
  p1 = p0 + t1 * d;
  p0 = start;
  return true;
}

}  // namespace

std::vector<GridIndex> bresenham_line(const GridIndex& a, const GridIndex& b) {
  std::vector<GridIndex> cells;
  const int dx = std::abs(b.x - a.x);
  const int dy = -std::abs(b.y - a.y);
  const int sx = a.x < b.x ? 1 : -1;
  const int sy = a.y < b.y ? 1 : -1;
  cells.reserve(static_cast<std::size_t>(std::max(dx, -dy)) + 1);
  int err = dx + dy;
  int x = a.x;
  int y = a.y;
  while (true) {
    cells.push_back({x, y});
    if (x == b.x && y == b.y) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
  return cells;
}

std::vector<GridIndex> traverse_grid(const GridSpec& spec, const Vec2& from, const Vec2& to) {
  std::vector<GridIndex> cells;
  Vec2 g0 = spec.world_to_grid(from);
  Vec2 g1 = spec.world_to_grid(to);
  const auto w = static_cast<double>(spec.width());
  const auto h = static_cast<double>(spec.height());
  if (!clip_segment(g0, g1, w, h)) {
    return cells;
  }
  const auto cell_of = [&](const Vec2& g) {
    return GridIndex{std::clamp(static_cast<int>(std::floor(g.x())), 0, spec.width() - 1),
                     std::clamp(static_cast<int>(std::floor(g.y())), 0, spec.height() - 1)};
  };
  GridIndex c = cell_of(g0);
  const GridIndex end = cell_of(g1);
  const Vec2 d = g1 - g0;
  const int sx = d.x() > 0.0 ? 1 : (d.x() < 0.0 ? -1 : 0);
  const int sy = d.y() > 0.0 ? 1 : (d.y() < 0.0 ? -1 : 0);
  // Parametric distance (t in [0, 1]) to the first vertical/horizontal cell boundary and the
  // distance between successive boundaries.
  double t_max_x = kInf;
  double t_max_y = kInf;
  double t_delta_x = kInf;
  double t_delta_y = kInf;
  if (sx != 0) {
    const double next = sx > 0 ? c.x + 1.0 : static_cast<double>(c.x);
    t_max_x = (next - g0.x()) / d.x();
    t_delta_x = 1.0 / std::abs(d.x());
  }
  if (sy != 0) {
    const double next = sy > 0 ? c.y + 1.0 : static_cast<double>(c.y);
    t_max_y = (next - g0.y()) / d.y();
    t_delta_y = 1.0 / std::abs(d.y());
  }
  const std::size_t max_steps =
      static_cast<std::size_t>(std::abs(end.x - c.x) + std::abs(end.y - c.y)) + 2;
  cells.reserve(max_steps + 2);
  cells.push_back(c);
  for (std::size_t step = 0; step < max_steps && !(c == end); ++step) {
    const double tx = t_max_x;
    const double ty = t_max_y;
    if (std::min(tx, ty) > 1.0) {
      break;
    }
    if (std::abs(tx - ty) <= 1e-12) {
      // Exact corner crossing: report both edge neighbours, then move diagonally.
      const GridIndex side_x{c.x + sx, c.y};
      const GridIndex side_y{c.x, c.y + sy};
      if (spec.in_bounds(side_x)) cells.push_back(side_x);
      if (spec.in_bounds(side_y)) cells.push_back(side_y);
      c = {c.x + sx, c.y + sy};
      t_max_x += t_delta_x;
      t_max_y += t_delta_y;
    } else if (tx < ty) {
      c.x += sx;
      t_max_x += t_delta_x;
    } else {
      c.y += sy;
      t_max_y += t_delta_y;
    }
    if (!spec.in_bounds(c)) {
      break;
    }
    cells.push_back(c);
  }
  return cells;
}

}  // namespace roboticscore::mapping
