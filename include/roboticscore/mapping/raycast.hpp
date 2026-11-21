#pragma once

/// @file raycast.hpp
/// @brief Grid ray traversal.
///
/// - bresenham_line(): integer line rasterisation (Bresenham, 1965). Visits exactly one cell
///   per step along the major axis; cheap, but can skip cells that the continuous segment
///   only clips at a corner. Used for inverse-sensor-model updates.
/// - traverse_grid(): exact voxel traversal (Amanatides & Woo, 1987). Visits every cell the
///   continuous segment passes through, in order. Used where missing a cell would be unsafe,
///   e.g. line-of-sight checks in any-angle planning.

#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/grid.hpp"

namespace roboticscore::mapping {

using geometry::GridIndex;
using geometry::GridSpec;

/// Cells of the Bresenham line from @p a to @p b, both endpoints included, ordered from a.
/// Complexity O(max(|dx|, |dy|)).
[[nodiscard]] std::vector<GridIndex> bresenham_line(const GridIndex& a, const GridIndex& b);

/// Cells crossed by the world-space segment from @p from to @p to, ordered from @p from and
/// clipped to the grid. When the segment passes exactly through a cell corner, both
/// edge-adjacent cells are reported (conservative for collision checking).
/// Complexity O(number of cells crossed).
[[nodiscard]] std::vector<GridIndex> traverse_grid(const GridSpec& spec, const Vec2& from,
                                                   const Vec2& to);

}  // namespace roboticscore::mapping
