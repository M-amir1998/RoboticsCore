#include <cmath>
#include <vector>

#include "roboticscore/estimation/extended_kalman_filter.hpp"
#include "roboticscore/estimation/particle_filter.hpp"
#include "roboticscore/localization/robot_models.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/registration/icp.hpp"
#include "roboticscore/sim/simulation.hpp"

#include "../pipelines/pipelines.hpp"
#include "demo_common.hpp"

namespace roboticscore::demos {
namespace {

const std::vector<Vec2> kLandmarks = {{2, 2}, {18, 3}, {17, 16}, {3, 17}, {10, 10}, {10, 1}};

sim::SimulationConfig localization_sim(std::uint64_t seed) {
  sim::SimulationConfig cfg;
  cfg.seed = seed;
  cfg.odometry_noise = {0.04, 0.004, 0.004, 0.04, 1e-5, 1e-5};
  cfg.landmark_sensor.max_range = 9.0;
  cfg.landmark_sensor.range_std = 0.2;
  cfg.landmark_sensor.bearing_std = 0.04;
  return cfg;
}

// Smooth closed drive through the arena.
models::UnicycleInput drive_command(int k) {
  return {1.0, 0.35 * std::sin(0.02 * k) + 0.12};
}

json demo_particle_filter(const DemoContext& ctx) {
  sim::World world(geometry::Environment2D({{0, 0}, {20, 20}}));
  world.landmarks = kLandmarks;
  sim::Simulation sim(world, geometry::Pose2(5, 5, 0.3), localization_sim(ctx.seed));
  math::Random rng(ctx.seed + 100);
  // Poor prior: particles spread over a large part of the arena.
  auto pf = estimation::ParticleFilter::from_gaussian(
      Vec3(8, 8, 0.0), Vec3(9.0, 9.0, 1.5).asDiagonal(), ctx.get("particles", 1500), rng,
      localization::pose_state_space());
  const auto mnoise = localization::VelocityNoise{0.04, 0.004, 0.004, 0.04, 1e-4, 1e-4};
  json frames = json::array();
  for (int k = 0; k < ctx.get("steps", 160); ++k) {
    const auto step = sim.step(drive_command(k));
    const Vec2 u(step.odometry.v, step.odometry.omega);
    pf.predict(
        [&](Eigen::Ref<VecX> x, math::Random& g) {
          const Mat2 m = mnoise.control_covariance(u(0), u(1));
          x = models::Unicycle::step(
              Vec3(x),
              {u(0) + g.normal(0, std::sqrt(m(0, 0))), u(1) + g.normal(0, std::sqrt(m(1, 1)))},
              0.1);
        },
        rng);
    const auto obs = sim.observe_landmarks();
    pf.update([&](const Eigen::Ref<const VecX>& x) {
      double ll = 0.0;
      for (const auto& z : obs) {
        ll += localization::range_bearing_log_likelihood(Vec3(x), kLandmarks[z.id],
                                                         Vec2(z.range, z.bearing), 0.2, 0.04);
      }
      return ll;
    });
    const bool resampled =
        pf.resample_if_needed(0.5, estimation::ResamplingMethod::kSystematic, rng);
    json particles = json::array();
    const auto& p = pf.particles();
    for (Index i = 0; i < p.cols(); i += 3) {
      particles.push_back(json::array({r4(p(0, i)), r4(p(1, i)), r4(p(2, i))}));
    }
    json observed = json::array();
    for (const auto& z : obs)
      observed.push_back(z.id);
    const VecX est = pf.mean();
    frames.push_back({{"truth", to_json(step.pose)},
                      {"estimate", to_json(geometry::Pose2(est(0), est(1), est(2)))},
                      {"particles", particles},
                      {"observed", observed},
                      {"ess", r4(pf.effective_sample_size())},
                      {"resampled", resampled}});
  }
  return {{"landmarks", to_json(kLandmarks)},
          {"bounds", json::array({0, 0, 20, 20})},
          {"frames", frames}};
}

json demo_ekf_localization(const DemoContext& ctx) {
  sim::World world(geometry::Environment2D({{0, 0}, {20, 20}}));
  world.landmarks = kLandmarks;
  const geometry::Pose2 start(5, 5, 0.3);
  sim::Simulation sim(world, start, localization_sim(ctx.seed));
  const auto mnoise = localization::VelocityNoise{0.04, 0.004, 0.004, 0.04, 1e-5, 1e-5};
  const auto process = localization::unicycle_process_model(mnoise);
  estimation::ExtendedKalmanFilter ekf(start.vector(), Vec3(0.01, 0.01, 0.001).asDiagonal(),
                                       localization::pose_state_space());
  geometry::Pose2 dead_reckoning = start;
  json frames = json::array();
  for (int k = 0; k < ctx.get("steps", 300); ++k) {
    const auto step = sim.step(drive_command(k));
    const Vec2 u(step.odometry.v, step.odometry.omega);
    ekf.predict(process, u, 0.1);
    dead_reckoning = models::Unicycle::step(dead_reckoning, {u(0), u(1)}, 0.1);
    json observed = json::array();
    for (const auto& z : sim.observe_landmarks()) {
      (void)ekf.update(localization::range_bearing_model(kLandmarks[z.id], 0.2, 0.04),
                       Vec2(z.range, z.bearing));
      observed.push_back(z.id);
    }
    const VecX& x = ekf.state();
    frames.push_back({{"truth", to_json(step.pose)},
                      {"estimate", to_json(geometry::Pose2(x(0), x(1), x(2)))},
                      {"covariance", cov2(ekf.covariance().topLeftCorner<2, 2>())},
                      {"dead_reckoning", to_json(dead_reckoning)},
                      {"observed", observed}});
  }
  return {{"landmarks", to_json(kLandmarks)},
          {"bounds", json::array({0, 0, 20, 20})},
          {"frames", frames}};
}

json demo_icp(const DemoContext& ctx) {
  math::Random rng(ctx.seed);
  std::vector<Vec2> target;
  const auto segment = [&](const Vec2& a, const Vec2& b, int n) {
    for (int i = 0; i < n; ++i) {
      target.push_back(a + (b - a) * (i / static_cast<double>(n))
                       + Vec2(rng.normal(0, 0.01), rng.normal(0, 0.01)));
    }
  };
  segment({0, 0}, {6, 0}, 60);
  segment({6, 0}, {6, 3}, 30);
  segment({6, 3}, {3, 4.5}, 30);
  segment({3, 4.5}, {0, 4}, 30);
  segment({0, 4}, {0, 0}, 40);
  segment({2, 1}, {3, 2}, 15);
  const geometry::SE2 truth(0.6, -0.4, math::deg2rad(ctx.get("rotation_deg", 25.0)));
  std::vector<Vec2> source;
  for (const auto& p : target) {
    source.push_back(truth.inverse() * p + Vec2(rng.normal(0, 0.01), rng.normal(0, 0.01)));
  }
  json out;
  out["target"] = to_json(target);
  out["source"] = to_json(source);
  out["truth"] = to_json(truth);
  for (const auto method :
       {registration::IcpMethod::kPointToPoint, registration::IcpMethod::kPointToLine}) {
    registration::IcpParams params;
    params.method = method;
    params.record_history = true;
    params.max_iterations = 40;
    const auto r = registration::icp_2d(source, target, params);
    out[method == registration::IcpMethod::kPointToLine ? "point_to_line" : "point_to_point"] = {
        {"history", to_json(r.history)}, {"rmse", r.rmse}, {"converged", r.converged()}};
  }
  return out;
}

json demo_pose_graph(const DemoContext& ctx) {
  const auto r = pipelines::run_loop_slam(ctx.seed);
  json iterations = json::array();
  for (const auto& poses : r.iterations)
    iterations.push_back(to_json(poses));
  json closures = json::array();
  for (const auto& [a, b] : r.loop_closures)
    closures.push_back(json::array({a, b}));
  return {{"truth", to_json(r.truth)},
          {"odometry", to_json(r.odometry)},
          {"iterations", iterations},
          {"loop_closures", closures},
          {"chi2", r.chi2},
          {"ate_before", r.ate_before},
          {"ate_after", r.ate_after}};
}

}  // namespace

void register_estimation_demos() {
  registry()["particle_filter"] = {"Particle filter localisation from a poor prior",
                                   demo_particle_filter};
  registry()["ekf_localization"] = {"EKF landmark localisation vs dead reckoning",
                                    demo_ekf_localization};
  registry()["icp"] = {"Point-to-point vs point-to-line ICP alignment", demo_icp};
  registry()["pose_graph"] = {"Pose-graph SLAM loop closure (Gauss-Newton)", demo_pose_graph};
}

}  // namespace roboticscore::demos
