#pragma once

/// @file grid_map.hpp
/// @brief Binary traversability grid used by all grid planners, plus the shared neighbour
///        and heuristic definitions.

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/grid.hpp"

namespace roboticscore::planning {

using geometry::GridIndex;
using geometry::GridSpec;

/// Neighbourhood used for grid search.
enum class Connectivity { kFour, kEight };

/// Admissible-heuristic choices. Admissibility depends on the connectivity:
///   4-connected: Manhattan (tightest), Euclidean, Octile and Chebyshev are admissible.
///   8-connected: Octile (tightest), Euclidean and Chebyshev are admissible; Manhattan
///                overestimates diagonal moves and makes A* suboptimal.
enum class Heuristic { kZero, kManhattan, kEuclidean, kOctile, kChebyshev };

/// Heuristic distance between two cells in cell units.
[[nodiscard]] double heuristic_distance(Heuristic h, const GridIndex& a, const GridIndex& b);

/// Grid of blocked (1) / free (0) cells.
class GridMap {
 public:
  /// All-free map.
  explicit GridMap(const GridSpec& spec);

  /// @throws std::invalid_argument if @p blocked does not have spec.cell_count() entries.
  GridMap(const GridSpec& spec, std::vector<std::uint8_t> blocked);

  [[nodiscard]] const GridSpec& spec() const noexcept { return spec_; }
  [[nodiscard]] int width() const noexcept { return spec_.width(); }
  [[nodiscard]] int height() const noexcept { return spec_.height(); }

  /// True for blocked or out-of-bounds cells.
  [[nodiscard]] bool blocked(const GridIndex& c) const {
    return !spec_.in_bounds(c) || cells_[spec_.linear_index(c)] != 0;
  }
  [[nodiscard]] bool traversable(const GridIndex& c) const { return !blocked(c); }

  /// @throws std::out_of_range for out-of-bounds cells.
  void set_blocked(const GridIndex& c, bool blocked);

  [[nodiscard]] const std::vector<std::uint8_t>& data() const noexcept { return cells_; }

  /// Copy in which every cell whose centre lies within @p radius metres of a blocked cell
  /// centre is blocked (configuration-space obstacle for a disc robot).
  [[nodiscard]] GridMap inflated(double radius) const;

  /// Cost of moving between two neighbouring cells (1 or sqrt(2) in cell units), or infinity
  /// if either cell is blocked or the move is a diagonal that would cut a blocked corner.
  [[nodiscard]] double step_cost(const GridIndex& from, const GridIndex& to) const;

  /// Neighbour offsets for a connectivity (orthogonal moves first).
  [[nodiscard]] static std::span<const GridIndex> offsets(Connectivity c);

 private:
  GridSpec spec_;
  std::vector<std::uint8_t> cells_;
};

/// World coordinates (cell centres) of a cell path.
[[nodiscard]] std::vector<Vec2> to_world(const GridSpec& spec, const std::vector<GridIndex>& cells);

/// Euclidean length in metres of a cell path (consecutive cells may be far apart, e.g. Theta*).
[[nodiscard]] double path_length(const GridSpec& spec, const std::vector<GridIndex>& cells);

}  // namespace roboticscore::planning
