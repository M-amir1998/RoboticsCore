#pragma once

/// @file occupancy_grid.hpp
/// @brief Probabilistic 2D occupancy grid in log-odds form (Moravec & Elfes, 1985; Thrun,
///        Burgard & Fox, "Probabilistic Robotics", ch. 9).
///
/// Each cell stores l = log(p / (1 - p)). A measurement updates l <- l + l_meas - l_prior,
/// which is the exact binary Bayes filter under the usual cell-independence assumption.
/// Log-odds are clamped to [clamp_min, clamp_max] so the map stays responsive to change and
/// probabilities never saturate to exactly 0 or 1.

#include <cstdint>
#include <span>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/grid.hpp"
#include "roboticscore/geometry/se2.hpp"

namespace roboticscore::mapping {

/// Inverse sensor model of a range beam.
struct InverseSensorModel {
  double p_hit = 0.7;       ///< P(occupied | beam ended in cell)
  double p_miss = 0.4;      ///< P(occupied | beam passed through cell)
  double p_prior = 0.5;     ///< Prior occupancy probability
  double clamp_min = -4.0;  ///< Log-odds lower bound (~ p = 0.018)
  double clamp_max = 4.0;   ///< Log-odds upper bound (~ p = 0.982)
};

/// Ternary classification of a cell.
enum class CellState : std::uint8_t { kUnknown, kFree, kOccupied };

/// Log-odds conversion helpers.
[[nodiscard]] double probability_to_log_odds(double p);
[[nodiscard]] double log_odds_to_probability(double l);

/// 2D occupancy grid.
class OccupancyGrid {
 public:
  /// @throws std::invalid_argument if the sensor model probabilities are not in (0, 1) or the
  ///         clamp interval is empty.
  explicit OccupancyGrid(const geometry::GridSpec& spec, const InverseSensorModel& model = {});

  [[nodiscard]] const geometry::GridSpec& spec() const noexcept { return spec_; }
  [[nodiscard]] const InverseSensorModel& model() const noexcept { return model_; }

  /// @throws std::out_of_range for cells outside the grid.
  [[nodiscard]] double log_odds(const geometry::GridIndex& c) const;
  [[nodiscard]] double probability(const geometry::GridIndex& c) const;
  [[nodiscard]] CellState state(const geometry::GridIndex& c, double occupied_threshold = 0.65,
                                double free_threshold = 0.35) const;

  /// Adds @p delta to the cell's log-odds (with clamping). Out-of-bounds cells are ignored.
  void add_log_odds(const geometry::GridIndex& c, double delta);

  /// Updates the cells along one beam: free space up to the end point, and the end cell as
  /// occupied when @p hit is true.
  void integrate_ray(const Vec2& origin, const Vec2& end, bool hit);

  /// Integrates a planar range scan taken from @p sensor_pose. Beams with range >= max_range
  /// (no return) only carve free space up to max_range.
  /// @throws std::invalid_argument if angles and ranges differ in size.
  void integrate_scan(const geometry::Pose2& sensor_pose, std::span<const double> angles,
                      std::span<const double> ranges, double max_range);

  /// Occupancy probabilities in GridSpec::linear_index order.
  [[nodiscard]] std::vector<double> probabilities() const;

  /// 1 for cells with probability >= threshold, else 0. Unknown cells (prior) are free when
  /// threshold > p_prior; pass a threshold <= p_prior to treat unknown space as occupied.
  [[nodiscard]] std::vector<std::uint8_t> occupied_mask(double threshold = 0.65) const;

  /// Number of cells that have received at least one update.
  [[nodiscard]] std::size_t observed_cells() const;

  /// True if cell @p c has received at least one update (false outside the grid).
  [[nodiscard]] bool observed(const geometry::GridIndex& c) const {
    return spec_.in_bounds(c) && observed_[spec_.linear_index(c)] != 0;
  }

 private:
  geometry::GridSpec spec_;
  InverseSensorModel model_;
  double l_hit_;
  double l_miss_;
  double l_prior_;
  std::vector<double> log_odds_;
  std::vector<std::uint8_t> observed_;
};

/// Agreement between an estimated map and a ground-truth occupancy raster.
struct MapComparison {
  double accuracy = 0.0;  ///< Fraction of observed cells classified correctly.
  double coverage = 0.0;  ///< Fraction of all cells that were observed.
  std::size_t false_occupied = 0;
  std::size_t false_free = 0;
};

/// Compares the classified map (threshold 0.5) with @p truth (1 = occupied).
/// @throws std::invalid_argument if sizes differ.
[[nodiscard]] MapComparison compare_to_ground_truth(const OccupancyGrid& map,
                                                    const std::vector<std::uint8_t>& truth);

}  // namespace roboticscore::mapping
