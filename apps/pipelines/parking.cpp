#include <algorithm>
#include <cmath>

#include "roboticscore/control/mpc.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/planning/kinodynamic/hybrid_astar.hpp"
#include "roboticscore/trajectory/reference_path.hpp"

#include "pipelines.hpp"

namespace roboticscore::pipelines {
namespace {

// Splits a directed path at gear changes into reference paths with signed speeds.
std::vector<trajectory::ReferencePath> split_by_gear(
    const std::vector<planning::DirectedPose>& path, double speed) {
  std::vector<trajectory::ReferencePath> segments;
  std::vector<trajectory::PathPoint> current;
  bool forward = path.empty() || path.front().forward;
  const auto flush = [&]() {
    if (current.size() >= 2) {
      trajectory::ReferencePath seg(current);
      seg.apply_speed_profile(speed, 0.8, 1.0, 0.0, 0.0);
      segments.push_back(std::move(seg));
    }
    current.clear();
  };
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto& p = path[i];
    if (p.forward != forward) {
      // The cusp pose ends one segment and starts the next.
      const trajectory::PathPoint cusp = current.back();
      flush();
      current.push_back({0.0, cusp.position, cusp.heading, cusp.curvature, 0.0});
      forward = p.forward;
    }
    const double s =
        current.empty()
            ? 0.0
            : current.back().s + (p.pose.translation() - current.back().position).norm();
    if (!current.empty() && s <= current.back().s + 1e-9) continue;
    current.push_back(
        {s, p.pose.translation(), p.pose.theta(), p.curvature, forward ? speed : -speed});
  }
  flush();
  return segments;
}

}  // namespace

ParkingResult run_parking() {
  ParkingResult out;
  out.scenario = sim::parking_scenario();
  const models::BicycleParams vehicle{2.7, 0.6, 3.0, 2.0};
  out.footprint = models::VehicleFootprint{4.5, 1.9, 0.9};
  const planning::HybridAStar planner(out.scenario.environment, vehicle, out.footprint);
  const auto plan = planner.plan(out.scenario.start, out.scenario.goal);
  out.planned = plan.path;
  out.expanded = plan.expanded_poses;
  out.planned_ok = plan.found;
  if (!plan.found) return out;

  const auto segments = split_by_gear(plan.path, 1.5);
  const models::KinematicBicycle car(vehicle);
  control::MpcConfig mpc_cfg;
  mpc_cfg.horizon = 15;
  mpc_cfg.state_weights = Vec4(3.0, 3.0, 10.0, 0.5);  // precise heading matters when parking
  mpc_cfg.min_speed = -vehicle.max_speed;
  control::MpcTracker mpc(vehicle, mpc_cfg);
  models::BicycleState x(out.scenario.start.x(), out.scenario.start.y(), out.scenario.start.theta(),
                         0.0);
  const double dt = 0.05;
  for (const auto& seg : segments) {
    mpc.reset();
    for (int k = 0; k < 800; ++k) {
      const auto u = mpc.compute(x, seg, dt);
      x = car.step(x, models::BicycleInput(u.acceleration, u.steering), dt);
      out.driven.push_back(x);
      out.steering.push_back(u.steering);
      out.predictions.push_back(mpc.status().prediction);
      out.max_cross_track = std::max(out.max_cross_track, std::abs(mpc.status().cross_track_error));
      if (mpc.status().path_s >= seg.length() - 0.05 && std::abs(x(3)) < 0.05) break;
    }
  }
  const Vec2 end(x(0), x(1));
  out.final_position_error = (end - out.scenario.goal.translation()).norm();
  out.final_heading_error = std::abs(math::angle_difference(x(2), out.scenario.goal.theta()));
  out.parked = out.final_position_error < 0.3 && out.final_heading_error < 0.1
               && planner.pose_free(geometry::Pose2(x(0), x(1), x(2)));
  return out;
}

}  // namespace roboticscore::pipelines
