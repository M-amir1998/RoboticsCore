// roboticscore_bench: scenario benchmarks with identical maps, starts, goals, seeds and
// collision checkers for every algorithm in a family. Writes one JSON document that the
// Python tooling turns into the tables in docs/benchmarks/.
//
//   roboticscore_bench --out results/benchmarks.json [--runs N]
//
// Always run a Release build; timings are wall-clock on the executing machine.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "roboticscore/control/geometric_trackers.hpp"
#include "roboticscore/control/lqr.hpp"
#include "roboticscore/control/mpc.hpp"
#include "roboticscore/core/version.hpp"
#include "roboticscore/estimation/consistency.hpp"
#include "roboticscore/estimation/extended_kalman_filter.hpp"
#include "roboticscore/estimation/particle_filter.hpp"
#include "roboticscore/estimation/unscented_kalman_filter.hpp"
#include "roboticscore/localization/robot_models.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/math/statistics.hpp"
#include "roboticscore/planning/grid/grid_search.hpp"
#include "roboticscore/planning/sampling/sampling_planners.hpp"
#include "roboticscore/sim/scenarios.hpp"
#include "roboticscore/sim/simulation.hpp"
#include "roboticscore/slam/ekf_slam.hpp"
#include "roboticscore/slam/fastslam.hpp"
#include "roboticscore/trajectory/tracks.hpp"

#include "pipelines.hpp"

namespace {

using json = nlohmann::json;
using namespace roboticscore;
using Clock = std::chrono::steady_clock;

double ms_since(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

double median(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double mean(const std::vector<double>& v) {
  return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double min_clearance(const geometry::Environment2D& env, const std::vector<Vec2>& path) {
  double c = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i) {
    for (int k = 0; k <= 10; ++k) {
      c = std::min(c, env.clearance(path[i - 1] + (path[i] - path[i - 1]) * (k / 10.0)));
    }
  }
  return c;
}

// ----------------------------------------------------------------------------- planning

json bench_grid_planners(int repeats) {
  json rows = json::array();
  const double res = 0.25;
  for (const auto& name : sim::scenario_names()) {
    const auto sc = sim::make_scenario(name, 1);
    const auto& b = sc.environment.bounds();
    const geometry::GridSpec spec(static_cast<int>(b.size().x() / res),
                                  static_cast<int>(b.size().y() / res), res, b.min);
    const planning::GridMap map(spec, sim::rasterize(sc.environment, spec, 0.3));
    const auto s = spec.world_to_cell(sc.start.translation());
    const auto g = spec.world_to_cell(sc.goal.translation());
    const std::vector<std::pair<std::string, std::function<planning::GridPath()>>> planners = {
        {"BFS", [&] { return planning::breadth_first_search(map, s, g); }},
        {"Dijkstra", [&] { return planning::dijkstra(map, s, g); }},
        {"A*", [&] { return planning::astar(map, s, g); }},
        {"Weighted A* (w=2)",
         [&] {
           return planning::astar(
               map, s, g, {planning::Connectivity::kEight, planning::Heuristic::kOctile, 2.0});
         }},
        {"Theta*", [&] { return planning::theta_star(map, s, g); }},
    };
    for (const auto& [planner, run] : planners) {
      std::vector<double> times;
      planning::GridPath result;
      for (int r = 0; r < repeats; ++r) {
        const auto t0 = Clock::now();
        result = run();
        times.push_back(ms_since(t0));
      }
      const auto pts = planning::to_world(spec, result.cells);
      rows.push_back(
          {{"scenario", name},
           {"planner", planner},
           {"success", result.found()},
           {"time_ms", median(times)},
           {"path_length", result.cost},
           {"expanded", result.expanded},
           {"min_clearance", result.found() ? min_clearance(sc.environment, pts) : 0.0}});
    }
  }
  return rows;
}

json bench_sampling_planners(int runs) {
  json rows = json::array();
  for (const std::string name : {"open_space", "cluttered", "narrow_passage", "warehouse"}) {
    const auto sc = sim::make_scenario(name, 1);
    const std::vector<std::pair<
        std::string, std::function<planning::SamplingResult(const planning::SamplingConfig&)>>>
        planners = {
            {"RRT",
             [&](auto& c) {
               return planning::rrt(sc.environment, sc.start.translation(), sc.goal.translation(),
                                    c);
             }},
            {"RRT-Connect",
             [&](auto& c) {
               return planning::rrt_connect(sc.environment, sc.start.translation(),
                                            sc.goal.translation(), c);
             }},
            {"RRT*",
             [&](auto& c) {
               return planning::rrt_star(sc.environment, sc.start.translation(),
                                         sc.goal.translation(), c);
             }},
            {"Informed RRT*",
             [&](auto& c) {
               return planning::informed_rrt_star(sc.environment, sc.start.translation(),
                                                  sc.goal.translation(), c);
             }},
            {"PRM",
             [&](auto& c) {
               return planning::prm(sc.environment, sc.start.translation(), sc.goal.translation(),
                                    c);
             }},
        };
    for (const auto& [planner, run] : planners) {
      std::vector<double> times;
      std::vector<double> lengths;
      std::vector<double> nodes;
      std::vector<double> checks;
      std::vector<double> clearances;
      int success = 0;
      for (int r = 0; r < runs; ++r) {
        planning::SamplingConfig cfg;
        cfg.seed = 1000 + static_cast<std::uint64_t>(r);
        cfg.robot_radius = 0.3;
        cfg.max_iterations = 3000;
        cfg.step_size = 1.0;
        cfg.prm_samples = 600;
        const auto t0 = Clock::now();
        const auto res = run(cfg);
        times.push_back(ms_since(t0));
        if (res.found) {
          ++success;
          lengths.push_back(res.cost);
          clearances.push_back(min_clearance(sc.environment, res.path));
        }
        nodes.push_back(static_cast<double>(res.tree.size()));
        checks.push_back(static_cast<double>(res.collision_checks));
      }
      rows.push_back({{"scenario", name},
                      {"planner", planner},
                      {"runs", runs},
                      {"success_rate", static_cast<double>(success) / runs},
                      {"time_ms", median(times)},
                      {"path_length", median(lengths)},
                      {"nodes", median(nodes)},
                      {"collision_checks", median(checks)},
                      {"min_clearance", median(clearances)}});
    }
  }
  return rows;
}

// ------------------------------------------------------------------------------- control

json bench_controllers() {
  json rows = json::array();
  const models::BicycleParams car{2.7, 0.6, 10.0, 3.0};
  for (const auto& track : trajectory::track_names()) {
    auto path = trajectory::make_track(trajectory::track_from_name(track), 5.0);
    path.apply_speed_profile(6.0, 1.5, 2.0, 0.0, 0.0);
    std::vector<std::unique_ptr<control::PathTracker>> trackers;
    trackers.push_back(std::make_unique<control::PurePursuitTracker>(car));
    trackers.push_back(std::make_unique<control::StanleyTracker>(car));
    trackers.push_back(std::make_unique<control::LqrTracker>(car));
    trackers.push_back(std::make_unique<control::MpcTracker>(car));
    for (auto& t : trackers) {
      const double dt = 0.05;
      const auto run = pipelines::run_tracking(*t, path, car, {0.0, 0.0, 0.0, 0.0}, dt, 4000);
      // Metrics after a 2 s settling window.
      const std::size_t skip = std::min<std::size_t>(40, run.cross_track.size());
      double cte_ss = 0.0, head_ss = 0.0, effort = 0.0, smooth = 0.0, max_cte = 0.0;
      std::size_t n = 0;
      for (std::size_t i = skip; i < run.cross_track.size(); ++i) {
        cte_ss += run.cross_track[i] * run.cross_track[i];
        head_ss += run.heading_error[i] * run.heading_error[i];
        max_cte = std::max(max_cte, std::abs(run.cross_track[i]));
        effort += run.steering[i] * run.steering[i] * dt;
        if (i > 0) {
          const double rate = (run.steering[i] - run.steering[i - 1]) / dt;
          smooth += rate * rate * dt;
        }
        ++n;
      }
      const double nn = static_cast<double>(std::max<std::size_t>(n, 1));
      rows.push_back({{"track", track},
                      {"controller", std::string(t->name())},
                      {"finished", run.finished},
                      {"cross_track_rmse", std::sqrt(cte_ss / nn)},
                      {"max_cross_track", max_cte},
                      {"heading_rmse", std::sqrt(head_ss / nn)},
                      {"steering_effort", effort},
                      {"steering_rate_energy", smooth},
                      {"completion_time_s", static_cast<double>(run.states.size()) * dt},
                      {"compute_us_mean", mean(run.compute_time_us)}});
    }
  }
  return rows;
}

// ---------------------------------------------------------------------------- estimation

struct NoiseSuite {
  std::string name;
  sim::OdometryNoise odom;
  double range_std;
  double bearing_std;
  std::vector<Vec2> landmarks;
  double outlier_probability;
};

json bench_estimators(int runs) {
  const std::vector<Vec2> dense = {{2, 2},   {18, 3}, {17, 16}, {3, 17},
                                   {10, 10}, {10, 1}, {1, 10},  {19, 11}};
  const std::vector<Vec2> sparse = {{2, 2}, {17, 16}};
  const sim::OdometryNoise low{0.01, 0.001, 0.001, 0.01, 1e-5, 1e-5};
  const sim::OdometryNoise high{0.1, 0.01, 0.01, 0.1, 1e-4, 1e-4};
  const std::vector<NoiseSuite> suites = {
      {"low_noise", low, 0.05, 0.01, dense, 0.0},
      {"high_odometry_noise", high, 0.05, 0.01, dense, 0.0},
      {"high_measurement_noise", low, 0.5, 0.1, dense, 0.0},
      {"sparse_landmarks", low, 0.05, 0.01, sparse, 0.0},
      {"outlier_measurement", low, 0.05, 0.01, dense, 0.05},
  };
  json rows = json::array();
  for (const auto& suite : suites) {
    const localization::VelocityNoise model{suite.odom.alpha1,      suite.odom.alpha2,
                                            suite.odom.alpha3,      suite.odom.alpha4,
                                            suite.odom.floor_var_v, suite.odom.floor_var_omega};
    const auto process = localization::unicycle_process_model(model);
    for (const std::string filter : {"EKF", "EKF + 99% gate", "UKF", "PF (500)"}) {
      std::vector<double> pos_rmse, head_rmse, nees_avg, nis_avg, time_ms;
      int diverged = 0;
      for (int r = 0; r < runs; ++r) {
        sim::World world(geometry::Environment2D({{-5, -5}, {25, 25}}));
        world.landmarks = suite.landmarks;
        sim::SimulationConfig cfg;
        cfg.seed = 500 + static_cast<std::uint64_t>(r);
        cfg.odometry_noise = suite.odom;
        cfg.landmark_sensor.max_range = 12.0;
        cfg.landmark_sensor.range_std = suite.range_std;
        cfg.landmark_sensor.bearing_std = suite.bearing_std;
        cfg.landmark_sensor.outlier_probability = suite.outlier_probability;
        const geometry::Pose2 start(5, 5, 0.3);
        sim::Simulation sim(world, start, cfg);
        const Mat3 p0 = Vec3(0.01, 0.01, 0.001).asDiagonal();
        estimation::ExtendedKalmanFilter ekf(start.vector(), p0, localization::pose_state_space());
        estimation::UnscentedKalmanFilter ukf(start.vector(), p0, {},
                                              localization::pose_state_space());
        math::Random rng(900 + static_cast<std::uint64_t>(r));
        auto pf = estimation::ParticleFilter::from_gaussian(start.vector(), p0, 500, rng,
                                                            localization::pose_state_space());
        const double gate =
            filter == "EKF + 99% gate" ? math::chi_squared_quantile(0.99, 2) : estimation::kNoGate;
        double pos_ss = 0.0, head_ss = 0.0, nees_sum = 0.0, nis_sum = 0.0;
        int nis_n = 0;
        const int steps = 300;
        const auto t0 = Clock::now();
        for (int k = 0; k < steps; ++k) {
          const models::UnicycleInput cmd{1.0, 0.35 * std::sin(0.02 * k) + 0.12};
          const auto step = sim.step(cmd);
          const Vec2 u(step.odometry.v, step.odometry.omega);
          const auto obs = sim.observe_landmarks();
          VecX est;
          MatX cov;
          if (filter == "UKF") {
            ukf.predict(process, u, cfg.dt);
            for (const auto& z : obs) {
              const auto res =
                  ukf.update(localization::range_bearing_model(suite.landmarks[z.id],
                                                               suite.range_std, suite.bearing_std),
                             Vec2(z.range, z.bearing));
              nis_sum += res.stats.nis;
              ++nis_n;
            }
            est = ukf.state();
            cov = ukf.covariance();
          } else if (filter == "PF (500)") {
            const Mat2 m = model.control_covariance(u(0), u(1));
            pf.predict(
                [&](Eigen::Ref<VecX> x, math::Random& g) {
                  x = models::Unicycle::step(Vec3(x),
                                             {u(0) + g.normal(0, std::sqrt(m(0, 0))),
                                              u(1) + g.normal(0, std::sqrt(m(1, 1)))},
                                             cfg.dt);
                },
                rng);
            pf.update([&](const Eigen::Ref<const VecX>& x) {
              double ll = 0.0;
              for (const auto& z : obs) {
                ll += localization::range_bearing_log_likelihood(
                    Vec3(x), suite.landmarks[z.id], Vec2(z.range, z.bearing), suite.range_std,
                    suite.bearing_std);
              }
              return ll;
            });
            pf.resample_if_needed(0.5, estimation::ResamplingMethod::kSystematic, rng);
            est = pf.mean();
            cov = pf.covariance();
          } else {
            ekf.predict(process, u, cfg.dt);
            for (const auto& z : obs) {
              const auto res =
                  ekf.update(localization::range_bearing_model(suite.landmarks[z.id],
                                                               suite.range_std, suite.bearing_std),
                             Vec2(z.range, z.bearing), gate);
              if (res.accepted) {
                nis_sum += res.stats.nis;
                ++nis_n;
              }
            }
            est = ekf.state();
            cov = ekf.covariance();
          }
          const VecX err = localization::pose_residual(step.pose.vector(), est);
          pos_ss += err.head<2>().squaredNorm();
          head_ss += err(2) * err(2);
          if (filter != "PF (500)") {
            try {
              nees_sum += estimation::nees(err, cov);
            } catch (const NumericalError&) {
              nees_sum += 1e6;
            }
          }
        }
        time_ms.push_back(ms_since(t0) / steps);
        const double prmse = std::sqrt(pos_ss / steps);
        if (prmse > 5.0) ++diverged;
        pos_rmse.push_back(prmse);
        head_rmse.push_back(std::sqrt(head_ss / steps));
        nees_avg.push_back(nees_sum / steps);
        if (nis_n > 0) nis_avg.push_back(nis_sum / nis_n);
      }
      json row = {{"suite", suite.name},
                  {"filter", filter},
                  {"runs", runs},
                  {"position_rmse", median(pos_rmse)},
                  {"heading_rmse", median(head_rmse)},
                  {"diverged_runs", diverged},
                  {"step_time_ms", median(time_ms)}};
      if (filter != "PF (500)") {
        row["nees_mean"] = mean(nees_avg);  // consistent filter: ~3 (state dimension)
        row["nis_mean"] = mean(nis_avg);    // consistent filter: ~2 (measurement dimension)
      }
      rows.push_back(row);
    }
  }
  return rows;
}

// ---------------------------------------------------------------------------------- SLAM

json bench_slam(int runs) {
  json rows = json::array();
  const std::vector<Vec2> world_lm = {{8, 0},   {8, 8},  {0, 10}, {-6, 6},
                                      {-8, -2}, {0, -6}, {5, -5}, {-3, 14}};
  for (const std::string method : {"EKF-SLAM", "FastSLAM 1.0 (50)"}) {
    std::vector<double> ate, lm_rmse, time_ms;
    for (int r = 0; r < runs; ++r) {
      math::Random rng(700 + static_cast<std::uint64_t>(r));
      sim::OdometryNoise odom{0.02, 0.002, 0.002, 0.02, 1e-6, 1e-6};
      sim::RangeBearingSensor sensor;
      sensor.max_range = 12.0;
      sensor.range_std = 0.1;
      sensor.bearing_std = 0.02;
      const localization::VelocityNoise model{0.02, 0.002, 0.002, 0.02, 1e-6, 1e-6};
      slam::EkfSlam ekf(geometry::Pose2(), Mat3::Identity() * 1e-6, model, 0.1, 0.02);
      slam::FastSlam fast(50, geometry::Pose2(), model, 0.1, 0.02,
                          800 + static_cast<std::uint64_t>(r));
      geometry::Pose2 truth;
      std::vector<geometry::Pose2> truths, ests;
      const auto t0 = Clock::now();
      for (int k = 0; k < 500; ++k) {
        const models::UnicycleInput u{1.0, 0.15};
        truth = models::Unicycle::step(truth, u, 0.1);
        const auto noisy = odom.corrupt(u, rng);
        std::vector<slam::RangeBearingObservation> zs;
        for (const auto& o : sensor.observe(truth, world_lm, rng))
          zs.push_back({o.id, o.range, o.bearing});
        if (method == "EKF-SLAM") {
          ekf.predict(noisy, 0.1);
          ekf.update(zs);
          ests.push_back(ekf.pose());
        } else {
          fast.predict(noisy, 0.1);
          fast.update(zs);
          ests.push_back(fast.mean_pose());
        }
        truths.push_back(truth);
      }
      time_ms.push_back(ms_since(t0) / 500.0);
      ate.push_back(slam::absolute_trajectory_error(ests, truths, false));
      double ss = 0.0;
      for (std::size_t i = 0; i < world_lm.size(); ++i) {
        Vec2 est;
        if (method == "EKF-SLAM") {
          est = ekf.landmark(i).value_or(Vec2(1e3, 1e3));
        } else {
          const auto& lms = fast.best_particle().landmarks;
          est = i < lms.size() && lms[i].initialized ? lms[i].mean : Vec2(1e3, 1e3);
        }
        ss += (est - world_lm[i]).squaredNorm();
      }
      lm_rmse.push_back(std::sqrt(ss / static_cast<double>(world_lm.size())));
    }
    rows.push_back({{"method", method},
                    {"runs", runs},
                    {"ate", median(ate)},
                    {"landmark_rmse", median(lm_rmse)},
                    {"step_time_ms", median(time_ms)}});
  }
  // Pose graph: drift before and after loop closure.
  std::vector<double> before, after, time_ms;
  for (int r = 0; r < runs; ++r) {
    const auto t0 = Clock::now();
    const auto res = pipelines::run_loop_slam(static_cast<std::uint64_t>(r + 1));
    time_ms.push_back(ms_since(t0));
    before.push_back(res.ate_before);
    after.push_back(res.ate_after);
  }
  rows.push_back({{"method", "Pose graph (Gauss-Newton)"},
                  {"runs", runs},
                  {"ate_odometry", median(before)},
                  {"ate", median(after)},
                  {"total_time_ms", median(time_ms)}});
  return rows;
}

std::string compiler() {
#if defined(__clang__)
  return "Clang " __clang_version__;
#elif defined(_MSC_VER)
  return "MSVC " + std::to_string(_MSC_FULL_VER);
#elif defined(__GNUC__)
  return "GCC " __VERSION__;
#else
  return "unknown";
#endif
}

}  // namespace

int main(int argc, char** argv) {
  std::string out_path = "benchmarks.json";
  int runs = 20;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--out" && i + 1 < argc)
      out_path = argv[++i];
    else if (a == "--runs" && i + 1 < argc)
      runs = std::atoi(argv[++i]);
    else {
      std::cerr << "usage: roboticscore_bench [--out file] [--runs N]\n";
      return 2;
    }
  }
#ifdef NDEBUG
  const char* build = "Release";
#else
  const char* build = "Debug (timings not representative)";
#endif
  json out;
  out["library_version"] = std::string(version());
  out["compiler"] = compiler();
  out["build_type"] = build;
  std::cout << "grid planners...\n";
  out["grid_planners"] = bench_grid_planners(5);
  std::cout << "sampling planners...\n";
  out["sampling_planners"] = bench_sampling_planners(runs);
  std::cout << "controllers...\n";
  out["controllers"] = bench_controllers();
  std::cout << "estimators...\n";
  out["estimators"] = bench_estimators(runs);
  std::cout << "slam...\n";
  out["slam"] = bench_slam(runs);
  std::ofstream(out_path) << out.dump(2);
  std::cout << "wrote " << out_path << "\n";
  return 0;
}
