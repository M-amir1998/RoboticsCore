#include "roboticscore/planning/grid/grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "roboticscore/core/error.hpp"

namespace roboticscore::planning {
namespace {

constexpr double kSqrt2 = 1.4142135623730951;

constexpr std::array<GridIndex, 4> kFourOffsets = {{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
constexpr std::array<GridIndex, 8> kEightOffsets = {
    {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};

}  // namespace

double heuristic_distance(Heuristic h, const GridIndex& a, const GridIndex& b) {
  const double dx = std::abs(a.x - b.x);
  const double dy = std::abs(a.y - b.y);
  switch (h) {
    case Heuristic::kZero:
      return 0.0;
    case Heuristic::kManhattan:
      return dx + dy;
    case Heuristic::kEuclidean:
      return std::hypot(dx, dy);
    case Heuristic::kOctile:
      return std::max(dx, dy) + (kSqrt2 - 1.0) * std::min(dx, dy);
    case Heuristic::kChebyshev:
      return std::max(dx, dy);
  }
  return 0.0;
}

GridMap::GridMap(const GridSpec& spec) : spec_(spec), cells_(spec.cell_count(), 0) {}

GridMap::GridMap(const GridSpec& spec, std::vector<std::uint8_t> blocked)
    : spec_(spec), cells_(std::move(blocked)) {
  detail::require(cells_.size() == spec_.cell_count(),
                  "GridMap: occupancy vector size must equal the cell count");
}

void GridMap::set_blocked(const GridIndex& c, bool blocked) {
  if (!spec_.in_bounds(c)) {
    throw std::out_of_range("GridMap::set_blocked: cell out of bounds");
  }
  cells_[spec_.linear_index(c)] = blocked ? 1 : 0;
}

GridMap GridMap::inflated(double radius) const {
  detail::require(radius >= 0.0, "GridMap::inflated: radius must be non-negative");
  const double r_cells = radius / spec_.resolution();
  const int k = static_cast<int>(std::floor(r_cells));
  std::vector<GridIndex> kernel;
  for (int dy = -k; dy <= k; ++dy) {
    for (int dx = -k; dx <= k; ++dx) {
      if (dx * dx + dy * dy <= r_cells * r_cells) {
        kernel.push_back({dx, dy});
      }
    }
  }
  GridMap out(spec_, cells_);
  for (int y = 0; y < height(); ++y) {
    for (int x = 0; x < width(); ++x) {
      if (cells_[spec_.linear_index({x, y})] == 0) {
        continue;
      }
      for (const GridIndex& d : kernel) {
        const GridIndex n{x + d.x, y + d.y};
        if (spec_.in_bounds(n)) {
          out.cells_[spec_.linear_index(n)] = 1;
        }
      }
    }
  }
  return out;
}

double GridMap::step_cost(const GridIndex& from, const GridIndex& to) const {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  if (blocked(from) || blocked(to)) {
    return kInf;
  }
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if (dx != 0 && dy != 0) {
    // No corner cutting: both orthogonal cells adjacent to the diagonal must be free.
    if (blocked({from.x + dx, from.y}) || blocked({from.x, from.y + dy})) {
      return kInf;
    }
    return kSqrt2;
  }
  return 1.0;
}

std::span<const GridIndex> GridMap::offsets(Connectivity c) {
  if (c == Connectivity::kFour) {
    return {kFourOffsets.data(), kFourOffsets.size()};
  }
  return {kEightOffsets.data(), kEightOffsets.size()};
}

std::vector<Vec2> to_world(const GridSpec& spec, const std::vector<GridIndex>& cells) {
  std::vector<Vec2> pts;
  pts.reserve(cells.size());
  for (const auto& c : cells) {
    pts.push_back(spec.cell_center(c));
  }
  return pts;
}

double path_length(const GridSpec& spec, const std::vector<GridIndex>& cells) {
  double len = 0.0;
  for (std::size_t i = 1; i < cells.size(); ++i) {
    len += std::hypot(cells[i].x - cells[i - 1].x, cells[i].y - cells[i - 1].y);
  }
  return len * spec.resolution();
}

}  // namespace roboticscore::planning
