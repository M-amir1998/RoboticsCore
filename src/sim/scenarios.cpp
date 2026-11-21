#include "roboticscore/sim/scenarios.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"

namespace roboticscore::sim {
namespace {

using geometry::Aabb;
using geometry::Circle;
using geometry::Environment2D;
using geometry::Polygon;
using geometry::Pose2;

// Axis-aligned box from its corners.
Polygon box(double x0, double y0, double x1, double y1) {
  return Polygon::from_aabb(Aabb{Vec2(x0, y0), Vec2(x1, y1)});
}

}  // namespace

Scenario open_space_scenario() {
  return {"open_space", Environment2D(Aabb{Vec2(0, 0), Vec2(20, 20)}), Pose2(2, 2, 0),
          Pose2(18, 18, 0)};
}

Scenario cluttered_scenario(std::uint64_t seed, int obstacle_count) {
  detail::require(obstacle_count >= 0, "cluttered_scenario: obstacle_count must be >= 0");
  Environment2D env(Aabb{Vec2(0, 0), Vec2(30, 20)});
  const Pose2 start(2, 2, 0);
  const Pose2 goal(28, 18, 0);
  math::Random rng(seed);
  int placed = 0;
  int attempts = 0;
  while (placed < obstacle_count && attempts < 100 * (obstacle_count + 1)) {
    ++attempts;
    const Vec2 c(rng.uniform(1.0, 29.0), rng.uniform(1.0, 19.0));
    const bool is_circle = rng.uniform01() < 0.5;
    const double extent = is_circle ? rng.uniform(0.4, 1.2) : 0.0;
    const double w = rng.uniform(0.8, 2.6);
    const double h = rng.uniform(0.8, 2.6);
    const double yaw = rng.uniform(-math::kPi, math::kPi);
    const double reach = is_circle ? extent : 0.5 * std::hypot(w, h);
    if ((c - start.translation()).norm() - reach < 2.5
        || (c - goal.translation()).norm() - reach < 2.5) {
      continue;
    }
    if (is_circle) {
      env.add_obstacle(Circle{c, extent});
    } else {
      env.add_obstacle(Polygon::rectangle(c, w, h, yaw));
    }
    ++placed;
  }
  return {"cluttered", std::move(env), start, goal};
}

Scenario narrow_passage_scenario(double gap) {
  detail::require(gap > 0.0 && gap < 10.0, "narrow_passage_scenario: gap must lie in (0, 10)");
  Environment2D env(Aabb{Vec2(0, 0), Vec2(20, 12)});
  env.add_obstacle(box(9.6, 0.0, 10.4, 6.0 - 0.5 * gap));
  env.add_obstacle(box(9.6, 6.0 + 0.5 * gap, 10.4, 12.0));
  return {"narrow_passage", std::move(env), Pose2(3, 2, 0), Pose2(17, 10, 0)};
}

Scenario maze_scenario(std::uint64_t seed, int cells_x, int cells_y, double cell_size) {
  detail::require(cells_x >= 2 && cells_y >= 2, "maze_scenario: at least 2x2 cells required");
  detail::require(cell_size > 0.5, "maze_scenario: cell_size must exceed the wall thickness");
  const auto idx = [&](int x, int y) { return static_cast<std::size_t>(y * cells_x + x); };
  const auto total = static_cast<std::size_t>(cells_x * cells_y);
  // open_right[c] / open_up[c]: passage from cell c to its +x / +y neighbour.
  std::vector<bool> open_right(total, false);
  std::vector<bool> open_up(total, false);
  std::vector<bool> visited(total, false);
  math::Random rng(seed);
  std::vector<std::pair<int, int>> stack{{0, 0}};
  visited[0] = true;
  while (!stack.empty()) {
    const auto [x, y] = stack.back();
    std::vector<std::pair<int, int>> options;
    const std::pair<int, int> deltas[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto& [dx, dy] : deltas) {
      const int nx = x + dx;
      const int ny = y + dy;
      if (nx >= 0 && ny >= 0 && nx < cells_x && ny < cells_y && !visited[idx(nx, ny)]) {
        options.emplace_back(dx, dy);
      }
    }
    if (options.empty()) {
      stack.pop_back();
      continue;
    }
    const auto [dx, dy] = options[rng.uniform_index(options.size())];
    const int nx = x + dx;
    const int ny = y + dy;
    if (dx == 1) open_right[idx(x, y)] = true;
    if (dx == -1) open_right[idx(nx, ny)] = true;
    if (dy == 1) open_up[idx(x, y)] = true;
    if (dy == -1) open_up[idx(nx, ny)] = true;
    visited[idx(nx, ny)] = true;
    stack.emplace_back(nx, ny);
  }

  const double t = 0.3;  // wall thickness
  Environment2D env(Aabb{Vec2(0, 0), Vec2(cells_x * cell_size, cells_y * cell_size)});
  for (int y = 0; y < cells_y; ++y) {
    for (int x = 0; x < cells_x; ++x) {
      const double x1 = (x + 1) * cell_size;
      const double y1 = (y + 1) * cell_size;
      if (x + 1 < cells_x && !open_right[idx(x, y)]) {
        env.add_obstacle(box(x1 - 0.5 * t, y * cell_size - 0.5 * t, x1 + 0.5 * t, y1 + 0.5 * t));
      }
      if (y + 1 < cells_y && !open_up[idx(x, y)]) {
        env.add_obstacle(box(x * cell_size - 0.5 * t, y1 - 0.5 * t, x1 + 0.5 * t, y1 + 0.5 * t));
      }
    }
  }
  const double h = 0.5 * cell_size;
  return {"maze", std::move(env), Pose2(h, h, 0),
          Pose2((cells_x - 1) * cell_size + h, (cells_y - 1) * cell_size + h, 0)};
}

Scenario warehouse_scenario() {
  Environment2D env(Aabb{Vec2(0, 0), Vec2(30, 20)});
  for (const double x : {5.0, 9.0, 13.0, 17.0, 21.0, 25.0}) {
    env.add_obstacle(box(x - 0.6, 2.5, x + 0.6, 8.5));
    env.add_obstacle(box(x - 0.6, 11.5, x + 0.6, 17.5));
  }
  // Pallets left in the aisles.
  env.add_obstacle(Circle{Vec2(11.0, 5.0), 0.5});
  env.add_obstacle(Circle{Vec2(19.0, 14.5), 0.5});
  env.add_obstacle(box(14.2, 9.4, 15.8, 10.6));
  return {"warehouse", std::move(env), Pose2(1.5, 1.5, 0), Pose2(23.0, 14.0, math::kHalfPi)};
}

Scenario parking_scenario() {
  Environment2D env(Aabb{Vec2(0, 0), Vec2(30, 20)});
  const double slot = 2.8;
  const int free_slot = 5;
  for (int i = 0; i < 9; ++i) {
    const double cx = 1.5 + slot * i + 0.5 * slot;
    // Bottom row is full; top row has one free slot.
    env.add_obstacle(Polygon::rectangle(Vec2(cx, 3.5), 4.4, 1.8, math::kHalfPi));
    if (i != free_slot) {
      env.add_obstacle(Polygon::rectangle(Vec2(cx, 16.5), 4.4, 1.8, math::kHalfPi));
    }
  }
  // Kerb behind the top row.
  env.add_obstacle(box(0.0, 19.2, 30.0, 20.0));
  const double goal_x = 1.5 + slot * free_slot + 0.5 * slot;
  return {"parking", std::move(env), Pose2(4.0, 9.5, 0.0), Pose2(goal_x, 17.6, -math::kHalfPi)};
}

Scenario urban_grid_scenario() {
  Environment2D env(Aabb{Vec2(0, 0), Vec2(40, 40)});
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double x0 = 4.0 + 12.0 * i;
      const double y0 = 4.0 + 12.0 * j;
      env.add_obstacle(box(x0, y0, x0 + 8.0, y0 + 8.0));
    }
  }
  return {"urban_grid", std::move(env), Pose2(2, 2, 0), Pose2(38, 38, math::kHalfPi)};
}

std::vector<std::string> scenario_names() {
  return {"open_space", "cluttered", "narrow_passage", "maze",
          "warehouse",  "parking",   "urban_grid"};
}

Scenario make_scenario(const std::string& name, std::uint64_t seed) {
  if (name == "open_space") return open_space_scenario();
  if (name == "cluttered") return cluttered_scenario(seed);
  if (name == "narrow_passage") return narrow_passage_scenario();
  if (name == "maze") return maze_scenario(seed);
  if (name == "warehouse") return warehouse_scenario();
  if (name == "parking") return parking_scenario();
  if (name == "urban_grid") return urban_grid_scenario();
  throw std::invalid_argument("make_scenario: unknown scenario '" + name + "'");
}

std::vector<std::uint8_t> rasterize(const geometry::Environment2D& env,
                                    const geometry::GridSpec& spec, double inflation) {
  detail::require(inflation >= 0.0, "rasterize: inflation must be non-negative");
  std::vector<std::uint8_t> cells(spec.cell_count(), 0);
  for (int y = 0; y < spec.height(); ++y) {
    for (int x = 0; x < spec.width(); ++x) {
      const geometry::GridIndex c{x, y};
      if (env.clearance(spec.cell_center(c)) <= inflation) {
        cells[spec.linear_index(c)] = 1;
      }
    }
  }
  return cells;
}

}  // namespace roboticscore::sim
