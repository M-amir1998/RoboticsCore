#pragma once

/// @file grid.hpp
/// @brief Geometry of a regular 2D grid: cell indexing and world <-> cell conversion.
///
/// Cell (i, j) covers [origin.x + i*res, origin.x + (i+1)*res) x [origin.y + j*res, ...).
/// The origin is the world position of the lower-left corner of cell (0, 0). Storage order
/// is row-major in y: linear index = j * width + i.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>

#include "roboticscore/core/error.hpp"
#include "roboticscore/core/types.hpp"

namespace roboticscore::geometry {

/// Integer cell coordinates.
struct GridIndex {
  int x = 0;
  int y = 0;

  friend bool operator==(const GridIndex&, const GridIndex&) = default;
};

/// Size, resolution and placement of a grid.
class GridSpec {
 public:
  GridSpec() = default;

  /// @throws std::invalid_argument for non-positive sizes or resolution.
  GridSpec(int width, int height, double resolution, const Vec2& origin = Vec2::Zero())
      : width_(width), height_(height), resolution_(resolution), origin_(origin) {
    detail::require(width > 0 && height > 0, "GridSpec: width and height must be positive");
    detail::require(resolution > 0.0, "GridSpec: resolution must be positive");
  }

  [[nodiscard]] int width() const noexcept { return width_; }
  [[nodiscard]] int height() const noexcept { return height_; }
  [[nodiscard]] double resolution() const noexcept { return resolution_; }
  [[nodiscard]] const Vec2& origin() const noexcept { return origin_; }
  [[nodiscard]] std::size_t cell_count() const noexcept {
    return static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  }

  [[nodiscard]] bool in_bounds(const GridIndex& c) const noexcept {
    return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_;
  }

  /// Cell containing @p p (may be out of bounds; check with in_bounds()).
  [[nodiscard]] GridIndex world_to_cell(const Vec2& p) const {
    return {static_cast<int>(std::floor((p.x() - origin_.x()) / resolution_)),
            static_cast<int>(std::floor((p.y() - origin_.y()) / resolution_))};
  }

  /// Cell containing @p p, or nullopt if @p p lies outside the grid.
  [[nodiscard]] std::optional<GridIndex> world_to_cell_checked(const Vec2& p) const {
    const GridIndex c = world_to_cell(p);
    if (!in_bounds(c)) {
      return std::nullopt;
    }
    return c;
  }

  /// World coordinates of the centre of cell @p c.
  [[nodiscard]] Vec2 cell_center(const GridIndex& c) const {
    return origin_ + resolution_ * Vec2(c.x + 0.5, c.y + 0.5);
  }

  /// Continuous grid coordinates (cell units, cell (0,0) spans [0,1)^2) of @p p.
  [[nodiscard]] Vec2 world_to_grid(const Vec2& p) const { return (p - origin_) / resolution_; }

  [[nodiscard]] std::size_t linear_index(const GridIndex& c) const noexcept {
    return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(width_)
           + static_cast<std::size_t>(c.x);
  }

  [[nodiscard]] GridIndex from_linear(std::size_t index) const noexcept {
    const auto w = static_cast<std::size_t>(width_);
    return {static_cast<int>(index % w), static_cast<int>(index / w)};
  }

  /// World extent covered by the grid.
  [[nodiscard]] Vec2 extent() const { return resolution_ * Vec2(width_, height_); }

 private:
  int width_ = 1;
  int height_ = 1;
  double resolution_ = 1.0;
  Vec2 origin_ = Vec2::Zero();
};

}  // namespace roboticscore::geometry

template <>
struct std::hash<roboticscore::geometry::GridIndex> {
  std::size_t operator()(const roboticscore::geometry::GridIndex& c) const noexcept {
    const auto hi = static_cast<std::uint64_t>(static_cast<std::uint32_t>(c.x));
    const auto lo = static_cast<std::uint64_t>(static_cast<std::uint32_t>(c.y));
    return std::hash<std::uint64_t>{}((hi << 32U) | lo);
  }
};
