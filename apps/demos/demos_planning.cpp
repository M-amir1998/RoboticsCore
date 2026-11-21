#include <cmath>
#include <vector>

#include "roboticscore/math/angles.hpp"
#include "roboticscore/planning/grid/dstar_lite.hpp"
#include "roboticscore/planning/grid/grid_search.hpp"
#include "roboticscore/planning/local/potential_field.hpp"
#include "roboticscore/planning/local/velocity_obstacles.hpp"
#include "roboticscore/planning/sampling/sampling_planners.hpp"
#include "roboticscore/sim/scenarios.hpp"

#include "demo_common.hpp"

namespace roboticscore::demos {
namespace {

using planning::GridIndex;

json cells_to_json(const std::vector<GridIndex>& cells) {
  json a = json::array();
  for (const auto& c : cells)
    a.push_back(json::array({c.x, c.y}));
  return a;
}

json grid_search_trace(const planning::GridPath& p, const std::vector<GridIndex>& expanded) {
  json j;
  j["found"] = p.found();
  j["cost"] = r4(p.cost);
  j["expanded"] = p.expanded;
  j["path"] = cells_to_json(p.cells);
  j["expansion_order"] = cells_to_json(expanded);
  return j;
}

json demo_astar(const DemoContext& ctx) {
  const auto sc = sim::cluttered_scenario(ctx.seed, ctx.get("obstacles", 45));
  const double res = ctx.get("resolution", 0.25);
  const geometry::GridSpec spec(static_cast<int>(30.0 / res), static_cast<int>(20.0 / res), res);
  const planning::GridMap map(spec,
                              sim::rasterize(sc.environment, spec, ctx.get("inflation", 0.3)));
  const GridIndex s = spec.world_to_cell(sc.start.translation());
  const GridIndex g = spec.world_to_cell(sc.goal.translation());
  json out;
  out["environment"] = to_json(sc.environment);
  out["grid"] = to_json(spec);
  std::vector<std::uint8_t> occ = map.data();
  out["occupancy"] = occ;
  out["start"] = json::array({s.x, s.y});
  out["goal"] = json::array({g.x, g.y});
  std::vector<GridIndex> order;
  const auto record = [&](const GridIndex& c) { order.push_back(c); };
  order.clear();
  out["astar"] = grid_search_trace(planning::astar(map, s, g, {}, record), order);
  order.clear();
  out["dijkstra"] = grid_search_trace(
      planning::dijkstra(map, s, g, planning::Connectivity::kEight, record), order);
  order.clear();
  out["theta_star"] = grid_search_trace(planning::theta_star(map, s, g, {}, record), order);
  return out;
}

json demo_dstar_lite(const DemoContext& ctx) {
  const geometry::GridSpec spec(60, 40, 0.5);
  auto sc = sim::cluttered_scenario(ctx.seed, ctx.get("obstacles", 25));
  planning::GridMap known(spec, sim::rasterize(sc.environment, spec, 0.25));
  const GridIndex start = spec.world_to_cell(sc.start.translation());
  const GridIndex goal = spec.world_to_cell(sc.goal.translation());
  known.set_blocked(start, false);
  known.set_blocked(goal, false);
  planning::DStarLite planner(known, start, goal);
  std::vector<GridIndex> expanded;
  planner.compute_shortest_path([&](const GridIndex& c) { expanded.push_back(c); });
  json out;
  out["grid"] = to_json(spec);
  out["occupancy"] = known.data();
  out["start"] = json::array({start.x, start.y});
  out["goal"] = json::array({goal.x, goal.y});
  out["initial_expansions"] = cells_to_json(expanded);
  json steps = json::array();
  GridIndex robot = start;
  const int insert_step = ctx.get("insert_step", 14);
  for (int k = 0; k < 400 && !(robot == goal); ++k) {
    json step;
    std::vector<GridIndex> changed;
    if (k == insert_step) {
      // A wall appears across the current plan a few cells ahead of the robot.
      const auto path = planner.path();
      const GridIndex mid = path[std::min<std::size_t>(path.size() - 1, 8)];
      for (int d = -6; d <= 6; ++d) {
        const GridIndex c{mid.x + d, mid.y - d};
        if (spec.in_bounds(c) && !(c == goal)) changed.push_back(c);
        const GridIndex c2{mid.x + d + 1, mid.y - d};
        if (spec.in_bounds(c2) && !(c2 == goal)) changed.push_back(c2);
      }
      std::vector<planning::CellChange> changes;
      for (const auto& c : changed)
        changes.push_back({c, true});
      planner.update_cells(changes);
      expanded.clear();
      planner.compute_shortest_path([&](const GridIndex& c) { expanded.push_back(c); });
      step["replanned"] = cells_to_json(expanded);
      step["replan_expansions"] = expanded.size();
    }
    step["robot"] = json::array({robot.x, robot.y});
    step["path"] = cells_to_json(planner.path());
    step["new_obstacles"] = cells_to_json(changed);
    steps.push_back(step);
    robot = planner.next_step();
    planner.move_start(robot);
  }
  out["steps"] = steps;
  // Reference: a fresh A* on the final map expands this many nodes.
  out["fresh_astar_expansions"] = planning::astar(planner.map(), robot, goal).expanded;
  return out;
}

json tree_json(const std::vector<planning::TreeNode>& tree) {
  json a = json::array();
  for (const auto& n : tree)
    a.push_back(json::array({r4(n.point.x()), r4(n.point.y()), n.parent}));
  return a;
}

json demo_rrt_star(const DemoContext& ctx) {
  const auto sc = sim::cluttered_scenario(ctx.seed, ctx.get("obstacles", 35));
  planning::SamplingConfig cfg;
  cfg.step_size = ctx.get("step_size", 1.0);
  cfg.robot_radius = 0.3;
  cfg.max_iterations = ctx.get("iterations", 2500);
  cfg.seed = ctx.seed;
  json events = json::array();
  const auto observer = [&](const planning::SamplingEvent& e) {
    if (e.type == planning::SamplingEvent::Type::kSolutionImproved) {
      events.push_back(json::array({2, e.iteration, e.node, r4(e.cost)}));
    } else {
      events.push_back(json::array({e.type == planning::SamplingEvent::Type::kNodeAdded ? 0 : 1,
                                    e.iteration, e.node, e.parent}));
    }
  };
  const auto star = planning::rrt_star(sc.environment, sc.start.translation(),
                                       sc.goal.translation(), cfg, observer);
  json out;
  out["environment"] = to_json(sc.environment);
  out["start"] = to_json(sc.start.translation());
  out["goal"] = to_json(sc.goal.translation());
  out["nodes"] = tree_json(star.tree);
  out["events"] = events;  // [type, iteration, node, parent|cost]
  out["path"] = to_json(star.path);
  out["cost_history"] = star.cost_history;
  const auto informed = planning::informed_rrt_star(sc.environment, sc.start.translation(),
                                                    sc.goal.translation(), cfg);
  out["informed"] = {{"nodes", tree_json(informed.tree)},
                     {"path", to_json(informed.path)},
                     {"cost_history", informed.cost_history}};
  const auto plain =
      planning::rrt(sc.environment, sc.start.translation(), sc.goal.translation(), cfg);
  out["rrt"] = {
      {"nodes", tree_json(plain.tree)}, {"path", to_json(plain.path)}, {"cost", plain.cost}};
  return out;
}

json demo_potential_field(const DemoContext&) {
  json out = json::array();
  const planning::PotentialFieldConfig cfg;
  // Success: scattered obstacles. Failure: the goal lies behind a U-shaped wall.
  geometry::Environment2D open({{0, 0}, {20, 20}});
  open.add_obstacle(geometry::Circle{{10, 6}, 1.0});
  open.add_obstacle(geometry::Circle{{7, 11}, 0.8});
  geometry::Environment2D trap({{0, 0}, {20, 20}});
  trap.add_obstacle(geometry::Polygon::from_aabb({{9.5, 6}, {10.5, 14}}));
  trap.add_obstacle(geometry::Polygon::from_aabb({{6, 6}, {10.5, 7}}));
  trap.add_obstacle(geometry::Polygon::from_aabb({{6, 13}, {10.5, 14}}));
  const std::pair<const geometry::Environment2D*, std::pair<Vec2, Vec2>> cases[] = {
      {&open, {Vec2(3, 3), Vec2(17, 15)}}, {&trap, {Vec2(3, 10), Vec2(16, 10)}}};
  for (const auto& [env, sg] : cases) {
    const auto r = planning::potential_field_plan(*env, sg.first, sg.second, cfg);
    json field = json::array();  // potential on a coarse grid (clipped) for contour plots
    for (int y = 0; y <= 40; ++y) {
      json row = json::array();
      for (int x = 0; x <= 40; ++x) {
        const double u = planning::potential({x * 0.5, y * 0.5}, sg.second, *env, cfg);
        row.push_back(r4(std::min(u, 60.0)));
      }
      field.push_back(row);
    }
    const char* status = r.status == planning::PotentialFieldStatus::kReachedGoal ? "reached_goal"
                         : r.status == planning::PotentialFieldStatus::kLocalMinimum
                             ? "local_minimum"
                             : "other";
    out.push_back({{"environment", to_json(*env)},
                   {"start", to_json(sg.first)},
                   {"goal", to_json(sg.second)},
                   {"path", to_json(r.path)},
                   {"status", status},
                   {"field", field}});
  }
  return {{"cases", out}};
}

json demo_rvo(const DemoContext& ctx) {
  const int n = ctx.get("agents", 8);
  const double radius = 0.35;
  std::vector<Vec2> pos(static_cast<std::size_t>(n));
  std::vector<Vec2> vel(static_cast<std::size_t>(n), Vec2::Zero());
  std::vector<Vec2> goal(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * math::kPi * i / n;
    pos[static_cast<std::size_t>(i)] = 5.0 * Vec2(std::cos(a), std::sin(a));
    goal[static_cast<std::size_t>(i)] = -pos[static_cast<std::size_t>(i)];
  }
  const planning::VelocityObstacleConfig cfg;
  json frames = json::array();
  for (int step = 0; step < 220; ++step) {
    std::vector<Vec2> next(pos.size());
    for (std::size_t i = 0; i < pos.size(); ++i) {
      std::vector<planning::VoNeighbor> others;
      for (std::size_t j = 0; j < pos.size(); ++j) {
        if (j != i) others.push_back({pos[j], vel[j], radius, true});
      }
      Vec2 pref = goal[i] - pos[i];
      if (pref.norm() > cfg.max_speed) pref *= cfg.max_speed / pref.norm();
      next[i] = planning::select_velocity(pos[i], vel[i], radius, pref, others, cfg).velocity;
    }
    for (std::size_t i = 0; i < pos.size(); ++i) {
      vel[i] = next[i];
      pos[i] += 0.1 * vel[i];
    }
    frames.push_back(to_json(pos));
  }
  return {{"radius", radius}, {"goals", to_json(goal)}, {"frames", frames}};
}

}  // namespace

void register_planning_demos() {
  registry()["astar"] = {"A*, Dijkstra and Theta* expansions on a cluttered grid", demo_astar};
  registry()["dstar_lite"] = {"D* Lite replanning after a wall appears", demo_dstar_lite};
  registry()["rrt_star"] = {"RRT* tree growth and rewiring, Informed RRT*, RRT", demo_rrt_star};
  registry()["potential_field"] = {"Potential field success and U-trap local minimum",
                                   demo_potential_field};
  registry()["rvo"] = {"Reciprocal velocity obstacles: agents swapping on a circle", demo_rvo};
}

}  // namespace roboticscore::demos
