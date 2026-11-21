#include "roboticscore/mapping/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "roboticscore/core/error.hpp"
#include "roboticscore/mapping/raycast.hpp"

namespace roboticscore::mapping {

double probability_to_log_odds(double p) {
  detail::require(p > 0.0 && p < 1.0, "probability_to_log_odds: p must lie in (0, 1)");
  return std::log(p / (1.0 - p));
}

double log_odds_to_probability(double l) {
  // Two branches keep exp() from overflowing for large |l|.
  if (l >= 0.0) {
    return 1.0 / (1.0 + std::exp(-l));
  }
  const double e = std::exp(l);
  return e / (1.0 + e);
}

OccupancyGrid::OccupancyGrid(const geometry::GridSpec& spec, const InverseSensorModel& model)
    : spec_(spec),
      model_(model),
      l_hit_(probability_to_log_odds(model.p_hit)),
      l_miss_(probability_to_log_odds(model.p_miss)),
      l_prior_(probability_to_log_odds(model.p_prior)),
      log_odds_(spec.cell_count(), l_prior_),
      observed_(spec.cell_count(), 0) {
  detail::require(model.clamp_min < model.clamp_max,
                  "OccupancyGrid: clamp_min must be below clamp_max");
  detail::require(l_prior_ >= model.clamp_min && l_prior_ <= model.clamp_max,
                  "OccupancyGrid: prior must lie inside the clamp interval");
}

double OccupancyGrid::log_odds(const geometry::GridIndex& c) const {
  if (!spec_.in_bounds(c)) {
    throw std::out_of_range("OccupancyGrid: cell out of bounds");
  }
  return log_odds_[spec_.linear_index(c)];
}

double OccupancyGrid::probability(const geometry::GridIndex& c) const {
  return log_odds_to_probability(log_odds(c));
}

CellState OccupancyGrid::state(const geometry::GridIndex& c, double occupied_threshold,
                               double free_threshold) const {
  const double p = probability(c);
  if (p >= occupied_threshold) {
    return CellState::kOccupied;
  }
  if (p <= free_threshold) {
    return CellState::kFree;
  }
  return CellState::kUnknown;
}

void OccupancyGrid::add_log_odds(const geometry::GridIndex& c, double delta) {
  if (!spec_.in_bounds(c)) {
    return;
  }
  const std::size_t i = spec_.linear_index(c);
  log_odds_[i] = std::clamp(log_odds_[i] + delta, model_.clamp_min, model_.clamp_max);
  observed_[i] = 1;
}

void OccupancyGrid::integrate_ray(const Vec2& origin, const Vec2& end, bool hit) {
  const auto cells = bresenham_line(spec_.world_to_cell(origin), spec_.world_to_cell(end));
  const double miss = l_miss_ - l_prior_;
  for (std::size_t i = 0; i + 1 < cells.size(); ++i) {
    add_log_odds(cells[i], miss);
  }
  add_log_odds(cells.back(), hit ? l_hit_ - l_prior_ : miss);
}

void OccupancyGrid::integrate_scan(const geometry::Pose2& sensor_pose,
                                   std::span<const double> angles, std::span<const double> ranges,
                                   double max_range) {
  detail::require(angles.size() == ranges.size(),
                  "OccupancyGrid::integrate_scan: angles and ranges must match");
  detail::require(max_range > 0.0, "OccupancyGrid::integrate_scan: max_range must be positive");
  const Vec2& origin = sensor_pose.translation();
  for (std::size_t i = 0; i < angles.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < 0.0) {
      continue;  // invalid return
    }
    const bool hit = r < max_range;
    const double length = std::min(r, max_range);
    const double a = sensor_pose.theta() + angles[i];
    integrate_ray(origin, origin + length * Vec2(std::cos(a), std::sin(a)), hit);
  }
}

std::vector<double> OccupancyGrid::probabilities() const {
  std::vector<double> p(log_odds_.size());
  std::transform(log_odds_.begin(), log_odds_.end(), p.begin(), log_odds_to_probability);
  return p;
}

std::vector<std::uint8_t> OccupancyGrid::occupied_mask(double threshold) const {
  const double l_threshold = probability_to_log_odds(std::clamp(threshold, 1e-12, 1.0 - 1e-12));
  std::vector<std::uint8_t> mask(log_odds_.size());
  for (std::size_t i = 0; i < log_odds_.size(); ++i) {
    mask[i] = log_odds_[i] >= l_threshold ? 1 : 0;
  }
  return mask;
}

std::size_t OccupancyGrid::observed_cells() const {
  return static_cast<std::size_t>(std::count(observed_.begin(), observed_.end(), 1));
}

MapComparison compare_to_ground_truth(const OccupancyGrid& map,
                                      const std::vector<std::uint8_t>& truth) {
  const auto& spec = map.spec();
  detail::require(truth.size() == spec.cell_count(),
                  "compare_to_ground_truth: truth raster size mismatch");
  MapComparison cmp;
  std::size_t observed = 0;
  std::size_t correct = 0;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    const geometry::GridIndex c = spec.from_linear(i);
    if (!map.observed(c)) {
      continue;
    }
    ++observed;
    const bool occupied = map.probability(c) > 0.5;
    if (occupied == (truth[i] != 0)) {
      ++correct;
    } else if (occupied) {
      ++cmp.false_occupied;
    } else {
      ++cmp.false_free;
    }
  }
  cmp.coverage = static_cast<double>(observed) / static_cast<double>(truth.size());
  cmp.accuracy = observed > 0 ? static_cast<double>(correct) / static_cast<double>(observed) : 0.0;
  return cmp;
}

}  // namespace roboticscore::mapping
