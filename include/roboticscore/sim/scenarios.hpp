#pragma once

/// @file scenarios.hpp
/// @brief Deterministic scenario generators for planning, control and estimation
///        benchmarks, and rasterisation of continuous workspaces into occupancy grids.
///
/// Every generator is a pure function of its arguments (including the seed), so benchmark
/// runs on different machines plan on identical maps.

#include <cstdint>
#include <string>
#include <vector>

#include "roboticscore/geometry/environment.hpp"
#include "roboticscore/geometry/grid.hpp"
#include "roboticscore/geometry/se2.hpp"

namespace roboticscore::sim {

/// A workspace with a start and goal pose.
struct Scenario {
  std::string name;
  geometry::Environment2D environment;
  geometry::Pose2 start;
  geometry::Pose2 goal;
};

/// Empty 20 m x 20 m workspace.
[[nodiscard]] Scenario open_space_scenario();

/// Randomly placed circles and boxes in a 30 m x 20 m workspace; start and goal regions are
/// kept free.
[[nodiscard]] Scenario cluttered_scenario(std::uint64_t seed, int obstacle_count = 45);

/// Two rooms joined by a single gap of width @p gap in a dividing wall.
[[nodiscard]] Scenario narrow_passage_scenario(double gap = 1.2);

/// Perfect maze (randomised depth-first search) of @p cells_x x @p cells_y corridors.
[[nodiscard]] Scenario maze_scenario(std::uint64_t seed, int cells_x = 8, int cells_y = 6,
                                     double cell_size = 3.0);

/// Warehouse with parallel shelving rows and cross aisles.
[[nodiscard]] Scenario warehouse_scenario();

/// Parking lot with two rows of parked cars and one free slot; the goal is a reverse-parked
/// rear-axle pose inside the free slot for a 4.5 m x 1.9 m car.
[[nodiscard]] Scenario parking_scenario();

/// Manhattan-style grid of city blocks separated by 4 m streets.
[[nodiscard]] Scenario urban_grid_scenario();

/// Names accepted by make_scenario().
[[nodiscard]] std::vector<std::string> scenario_names();

/// Scenario by name (seed is used by randomised generators only).
/// @throws std::invalid_argument for an unknown name.
[[nodiscard]] Scenario make_scenario(const std::string& name, std::uint64_t seed = 1);

/// Occupancy raster: cell = 1 if its centre is within @p inflation of an obstacle or wall
/// (clearance <= inflation), else 0. Storage order follows GridSpec::linear_index.
[[nodiscard]] std::vector<std::uint8_t> rasterize(const geometry::Environment2D& env,
                                                  const geometry::GridSpec& spec,
                                                  double inflation = 0.0);

}  // namespace roboticscore::sim
