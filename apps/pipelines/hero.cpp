#include <algorithm>
#include <cmath>

#include "roboticscore/estimation/extended_kalman_filter.hpp"
#include "roboticscore/localization/robot_models.hpp"
#include "roboticscore/mapping/occupancy_grid.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/planning/grid/grid_search.hpp"
#include "roboticscore/planning/local/dwa.hpp"
#include "roboticscore/sim/simulation.hpp"
#include "roboticscore/trajectory/reference_path.hpp"

#include "pipelines.hpp"

namespace roboticscore::pipelines {

HeroResult run_hero(std::uint64_t seed) {
  HeroResult out;
  out.scenario = sim::warehouse_scenario();
  const auto& env = out.scenario.environment;
  const Vec2 goal = out.scenario.goal.translation();
  out.landmarks = {{3, 10},   {7, 0.6},   {11, 10},  {15, 0.6},  {19, 19.4}, {23, 10},
                   {27, 0.6}, {29.4, 10}, {7, 19.4}, {15, 19.4}, {27, 19.4}, {1, 6}};

  // Ground-truth world with moving obstacles.
  sim::World world(env);
  world.landmarks = out.landmarks;
  world.dynamic_obstacles = {{{{14.0, 0.8}, 0.35}, {-0.35, 0.0}},
                             {{{23.0, 13.0}, 0.35}, {0.0, -0.25}},
                             {{{7.0, 6.0}, 0.35}, {0.0, 0.3}}};

  sim::SimulationConfig cfg;
  cfg.dt = 0.1;
  cfg.seed = seed;
  cfg.robot_radius = 0.3;
  cfg.actuation_noise = {0.005, 0.0005, 0.0005, 0.005, 1e-5, 1e-5};
  cfg.odometry_noise = {0.02, 0.002, 0.002, 0.02, 1e-5, 1e-5};
  cfg.landmark_sensor.max_range = 8.0;
  cfg.landmark_sensor.range_std = 0.1;
  cfg.landmark_sensor.bearing_std = 0.02;
  cfg.lidar = {120, math::kTwoPi, 6.0, 0.02};
  sim::Simulation sim(world, out.scenario.start, cfg);

  // Global plan on the static map (configuration space for the robot disc + margin).
  out.map_spec = geometry::GridSpec(150, 100, 0.2, env.bounds().min);
  const planning::GridMap cspace(out.map_spec, sim::rasterize(env, out.map_spec, 0.7));
  const auto plan =
      planning::theta_star(cspace, out.map_spec.world_to_cell(out.scenario.start.translation()),
                           out.map_spec.world_to_cell(goal));
  if (!plan.found()) {
    return out;
  }
  out.global_path = planning::to_world(out.map_spec, plan.cells);
  out.global_path.front() = out.scenario.start.translation();
  out.global_path.back() = goal;
  std::vector<trajectory::PathPoint> pts;
  double s = 0.0;
  for (std::size_t i = 0; i < out.global_path.size(); ++i) {
    if (i > 0) s += (out.global_path[i] - out.global_path[i - 1]).norm();
    pts.push_back({s, out.global_path[i], 0.0, 0.0, 1.0});
  }
  const trajectory::ReferencePath route(pts);

  // Estimator and mapper.
  const localization::VelocityNoise odom_model{0.02, 0.002, 0.002, 0.02, 1e-5, 1e-5};
  const auto process = localization::unicycle_process_model(odom_model);
  Vec3 x0 = out.scenario.start.vector();
  x0 += Vec3(0.1, -0.1, 0.02);  // imperfect initialisation
  estimation::ExtendedKalmanFilter ekf(x0, Vec3(0.04, 0.04, 0.005).asDiagonal(),
                                       localization::pose_state_space());
  mapping::OccupancyGrid map(out.map_spec);

  planning::DwaConfig dwa;
  dwa.max_speed = 1.0;
  dwa.max_yaw_rate = 1.4;
  dwa.robot_radius = 0.4;  // robot radius + margin for localisation error
  dwa.predict_time = 1.8;
  dwa.heading_weight = 1.0;
  dwa.clearance_weight = 0.35;
  dwa.velocity_weight = 0.25;
  dwa.escape_when_blocked = true;

  models::UnicycleInput command{0.0, 0.0};
  std::ptrdiff_t hint = -1;
  double pos_ss = 0.0;
  double head_ss = 0.0;
  for (int k = 0; k < 1500; ++k) {
    const geometry::Pose2 est(ekf.state()(0), ekf.state()(1), ekf.state()(2));
    // Carrot on the global path ahead of the estimated position.
    const auto proj = route.project(est.translation(), hint, 80);
    hint = static_cast<std::ptrdiff_t>(proj.index);
    Vec2 target = route.at(proj.s + 1.6).position;
    if ((goal - est.translation()).norm() < 1.6) target = goal;

    // Moving obstacles as sensed now (position and velocity), predicted inside DWA.
    std::vector<planning::MovingObstacle> moving;
    std::vector<geometry::Circle> dynamic;
    for (const auto& o : sim.world().dynamic_obstacles) {
      moving.push_back({o.shape, o.velocity});
      dynamic.push_back(o.shape);
    }
    const auto plan_local = planning::dwa_plan(est, command, target, env, moving, dwa);
    command = plan_local.command;  // escape command when nothing is admissible

    const sim::StepResult step = sim.step(command);
    ekf.predict(process, Vec2(step.odometry.v, step.odometry.omega), cfg.dt);
    HeroFrame frame;
    for (const auto& z : sim.observe_landmarks()) {
      (void)ekf.update(localization::range_bearing_model(out.landmarks[z.id], 0.1, 0.02),
                       Vec2(z.range, z.bearing));
      frame.observed_landmarks.push_back(z.id);
    }
    const geometry::Pose2 est_after(ekf.state()(0), ekf.state()(1), ekf.state()(2));
    const auto scan = sim.scan_lidar();
    map.integrate_scan(est_after, scan.angles, scan.ranges, cfg.lidar.max_range);

    frame.time = step.time;
    frame.truth = step.pose;
    frame.estimate = ekf.state();
    frame.covariance = ekf.covariance();
    frame.target = target;
    if (plan_local.valid) {
      for (const auto& p : plan_local.candidates[plan_local.best].rollout) {
        frame.chosen_rollout.push_back(p.translation());
      }
    }
    for (std::size_t i = 0; i < scan.ranges.size(); i += 2) {
      if (!scan.hit[i]) continue;
      const double a = step.pose.theta() + scan.angles[i];
      frame.lidar_hits.push_back(step.pose.translation()
                                 + scan.ranges[i] * Vec2(std::cos(a), std::sin(a)));
    }
    frame.dynamic_obstacles = dynamic;
    frame.clearance = sim.world().clearance(step.pose.translation());
    out.frames.push_back(std::move(frame));
    out.collision = out.collision || step.collision;
    const Vec2 e = step.pose.translation() - est_after.translation();
    pos_ss += e.squaredNorm();
    const double he = math::angle_difference(step.pose.theta(), est_after.theta());
    head_ss += he * he;

    if (k % 10 == 0) {
      MapSnapshot snap;
      snap.frame = out.frames.size() - 1;
      for (const double p : map.probabilities()) {
        snap.probability.push_back(static_cast<std::uint8_t>(std::lround(255.0 * p)));
      }
      out.maps.push_back(std::move(snap));
    }
    if ((step.pose.translation() - goal).norm() < 0.3) {
      out.reached_goal = true;
      break;
    }
  }
  MapSnapshot last;
  last.frame = out.frames.size() - 1;
  for (const double p : map.probabilities()) {
    last.probability.push_back(static_cast<std::uint8_t>(std::lround(255.0 * p)));
  }
  out.maps.push_back(std::move(last));
  const auto n = static_cast<double>(std::max<std::size_t>(out.frames.size(), 1));
  out.position_rmse = std::sqrt(pos_ss / n);
  out.heading_rmse = std::sqrt(head_ss / n);
  out.map_accuracy =
      mapping::compare_to_ground_truth(map, sim::rasterize(env, out.map_spec, 0.0)).accuracy;
  return out;
}

}  // namespace roboticscore::pipelines
