#include <vector>

#include "../pipelines/pipelines.hpp"
#include "demo_common.hpp"

namespace roboticscore::demos {
namespace {

json demo_hero(const DemoContext& ctx) {
  const auto r = pipelines::run_hero(ctx.seed);
  json frames = json::array();
  for (const auto& f : r.frames) {
    json dyn = json::array();
    for (const auto& c : f.dynamic_obstacles) {
      dyn.push_back(json::array({r4(c.center.x()), r4(c.center.y()), r4(c.radius)}));
    }
    frames.push_back(
        {{"t", r4(f.time)},
         {"truth", to_json(f.truth)},
         {"estimate", to_json(geometry::Pose2(f.estimate(0), f.estimate(1), f.estimate(2)))},
         {"covariance", cov2(f.covariance.topLeftCorner<2, 2>())},
         {"target", to_json(f.target)},
         {"rollout", to_json(f.chosen_rollout)},
         {"lidar", to_json(f.lidar_hits)},
         {"dynamic", dyn},
         {"observed", f.observed_landmarks},
         {"clearance", r4(f.clearance)}});
  }
  json maps = json::array();
  for (const auto& m : r.maps)
    maps.push_back({{"frame", m.frame}, {"p", m.probability}});
  return {{"environment", to_json(r.scenario.environment)},
          {"start", to_json(r.scenario.start)},
          {"goal", to_json(r.scenario.goal)},
          {"landmarks", to_json(r.landmarks)},
          {"grid", to_json(r.map_spec)},
          {"global_path", to_json(r.global_path)},
          {"frames", frames},
          {"maps", maps},
          {"reached_goal", r.reached_goal},
          {"collision", r.collision},
          {"position_rmse", r.position_rmse},
          {"heading_rmse", r.heading_rmse},
          {"map_accuracy", r.map_accuracy}};
}

json demo_parking(const DemoContext&) {
  const auto r = pipelines::run_parking();
  json planned = json::array();
  for (const auto& p : r.planned) {
    planned.push_back(
        json::array({r4(p.pose.x()), r4(p.pose.y()), r4(p.pose.theta()), p.forward ? 1 : -1}));
  }
  json driven = json::array();
  for (const auto& x : r.driven)
    driven.push_back(json::array({r4(x(0)), r4(x(1)), r4(x(2)), r4(x(3))}));
  json predictions = json::array();
  for (std::size_t i = 0; i < r.predictions.size(); i += 4) {
    predictions.push_back(json::array({i, to_json(r.predictions[i])}));
  }
  json expanded = json::array();
  for (const auto& p : r.expanded)
    expanded.push_back(json::array({r4(p.x()), r4(p.y())}));
  return {{"environment", to_json(r.scenario.environment)},
          {"start", to_json(r.scenario.start)},
          {"goal", to_json(r.scenario.goal)},
          {"footprint",
           {{"length", r.footprint.length},
            {"width", r.footprint.width},
            {"rear_overhang", r.footprint.rear_overhang}}},
          {"planned", planned},
          {"expanded", expanded},
          {"driven", driven},
          {"steering", r.steering},
          {"predictions", predictions},
          {"parked", r.parked},
          {"final_position_error", r.final_position_error},
          {"final_heading_error", r.final_heading_error},
          {"max_cross_track", r.max_cross_track}};
}

}  // namespace

void register_integrated_demos() {
  registry()["hero"] = {"Integrated mobile robot: EKF + mapping + Theta* + DWA", demo_hero};
  registry()["parking"] = {"Car parking: Hybrid A* + MPC tracking", demo_parking};
}

}  // namespace roboticscore::demos
