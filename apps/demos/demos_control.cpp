#include <memory>
#include <string>
#include <vector>

#include "roboticscore/control/geometric_trackers.hpp"
#include "roboticscore/control/lqr.hpp"
#include "roboticscore/control/mpc.hpp"
#include "roboticscore/trajectory/tracks.hpp"

#include "../pipelines/pipelines.hpp"
#include "demo_common.hpp"

namespace roboticscore::demos {
namespace {

json run_json(const pipelines::TrackingRun& run, std::size_t stride) {
  json states = json::array();
  json steer = json::array();
  json cte = json::array();
  for (std::size_t i = 0; i < run.states.size(); i += stride) {
    const auto& x = run.states[i];
    states.push_back(json::array({r4(x(0)), r4(x(1)), r4(x(2)), r4(x(3))}));
    steer.push_back(r4(run.steering[i]));
    cte.push_back(r4(run.cross_track[i]));
  }
  return {
      {"states", states}, {"steering", steer}, {"cross_track", cte}, {"finished", run.finished}};
}

json path_json(const trajectory::ReferencePath& path) {
  json pts = json::array();
  for (std::size_t i = 0; i < path.size(); i += 2)
    pts.push_back(to_json(path[i].position));
  return pts;
}

json demo_controllers(const DemoContext& ctx) {
  const std::string track = ctx.get<std::string>("track", "sharp_turns");
  auto path = trajectory::make_track(trajectory::track_from_name(track), 5.0);
  path.apply_speed_profile(ctx.get("max_speed", 6.0), 1.5, 2.0, 0.0, 0.0);
  const models::BicycleParams car{2.7, 0.6, 10.0, 3.0};
  const models::BicycleState start(0.0, 1.0, 0.1, 0.0);
  json out;
  out["track"] = track;
  out["path"] = path_json(path);
  out["wheelbase"] = car.wheelbase;
  std::vector<std::unique_ptr<control::PathTracker>> trackers;
  trackers.push_back(std::make_unique<control::PurePursuitTracker>(car));
  trackers.push_back(std::make_unique<control::StanleyTracker>(car));
  trackers.push_back(std::make_unique<control::LqrTracker>(car));
  trackers.push_back(std::make_unique<control::MpcTracker>(car));
  for (auto& t : trackers) {
    const auto run = pipelines::run_tracking(*t, path, car, start, 0.05, 3000);
    out["runs"][std::string(t->name())] = run_json(run, 2);
  }
  // Failure illustration: Pure Pursuit with a too-short and a too-long fixed lookahead.
  for (const double ld : {1.0, 9.0}) {
    control::PurePursuitConfig cfg;
    cfg.lookahead_gain = 0.0;
    cfg.lookahead_offset = ld;
    cfg.lookahead_min = ld;
    cfg.lookahead_max = ld;
    control::PurePursuitTracker pp(car, cfg);
    const auto run = pipelines::run_tracking(pp, path, car, start, 0.05, 3000);
    out["pure_pursuit_lookahead"][std::to_string(static_cast<int>(ld))] = run_json(run, 2);
  }
  return out;
}

}  // namespace

void register_control_demos() {
  registry()["controllers"] = {"Pure Pursuit, Stanley, LQR and MPC on one track", demo_controllers};
}

}  // namespace roboticscore::demos
