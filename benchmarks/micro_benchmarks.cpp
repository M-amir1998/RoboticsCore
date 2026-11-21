// Microbenchmarks of hot kernels. Run a Release build with repetitions, e.g.
//   roboticscore_microbenchmarks --benchmark_repetitions=5 --benchmark_report_aggregates_only=true

#include <vector>

#include <benchmark/benchmark.h>

#include "roboticscore/control/lqr.hpp"
#include "roboticscore/estimation/extended_kalman_filter.hpp"
#include "roboticscore/estimation/unscented_kalman_filter.hpp"
#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/localization/robot_models.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/models/double_integrator.hpp"
#include "roboticscore/planning/grid/grid_search.hpp"
#include "roboticscore/planning/kinodynamic/reeds_shepp.hpp"
#include "roboticscore/registration/icp.hpp"
#include "roboticscore/sim/scenarios.hpp"
#include "roboticscore/spatial/kd_tree.hpp"

namespace {

using namespace roboticscore;

std::vector<Vec2> random_points(std::size_t n, std::uint64_t seed) {
  math::Random rng(seed);
  std::vector<Vec2> pts(n);
  for (auto& p : pts)
    p = Vec2(rng.uniform(0, 100), rng.uniform(0, 100));
  return pts;
}

void BM_Se2ComposeExpLog(benchmark::State& state) {
  const geometry::SE2 a(1.0, 2.0, 0.3);
  const Vec3 xi(0.1, -0.2, 0.05);
  for (auto _ : state) {
    const geometry::SE2 b = a * geometry::SE2::exp(xi);
    benchmark::DoNotOptimize(b.log());
  }
}
BENCHMARK(BM_Se2ComposeExpLog);

void BM_KdTreeNearest(benchmark::State& state) {
  const auto pts = random_points(static_cast<std::size_t>(state.range(0)), 1);
  const spatial::KdTree2D tree(pts);
  const auto queries = random_points(1024, 2);
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(tree.nearest(queries[i++ & 1023U]));
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_KdTreeNearest)->RangeMultiplier(10)->Range(100, 100000)->Complexity(benchmark::oLogN);

void BM_AStarCluttered(benchmark::State& state) {
  const auto sc = sim::cluttered_scenario(1);
  const geometry::GridSpec spec(120, 80, 0.25);
  const planning::GridMap map(spec, sim::rasterize(sc.environment, spec, 0.3));
  const auto s = spec.world_to_cell(sc.start.translation());
  const auto g = spec.world_to_cell(sc.goal.translation());
  for (auto _ : state) {
    benchmark::DoNotOptimize(planning::astar(map, s, g));
  }
}
BENCHMARK(BM_AStarCluttered)->Unit(benchmark::kMicrosecond);

void BM_EkfPredictUpdate(benchmark::State& state) {
  const auto process = localization::unicycle_process_model({});
  const auto meas = localization::range_bearing_model({5.0, 5.0}, 0.1, 0.02);
  estimation::ExtendedKalmanFilter ekf(Vec3(0, 0, 0), Mat3::Identity() * 0.1,
                                       localization::pose_state_space());
  for (auto _ : state) {
    ekf.predict(process, Vec2(1.0, 0.1), 0.1);
    benchmark::DoNotOptimize(ekf.update(meas, Vec2(7.0, 0.7)));
  }
}
BENCHMARK(BM_EkfPredictUpdate);

void BM_UkfPredictUpdate(benchmark::State& state) {
  const auto process = localization::unicycle_process_model({});
  const auto meas = localization::range_bearing_model({5.0, 5.0}, 0.1, 0.02);
  estimation::UnscentedKalmanFilter ukf(Vec3(0, 0, 0), Mat3::Identity() * 0.1, {},
                                        localization::pose_state_space());
  for (auto _ : state) {
    ukf.predict(process, Vec2(1.0, 0.1), 0.1);
    benchmark::DoNotOptimize(ukf.update(meas, Vec2(7.0, 0.7)));
  }
}
BENCHMARK(BM_UkfPredictUpdate);

void BM_ReedsSheppShortest(benchmark::State& state) {
  math::Random rng(3);
  std::vector<geometry::Pose2> goals;
  for (int i = 0; i < 256; ++i)
    goals.emplace_back(rng.uniform(-10, 10), rng.uniform(-10, 10), rng.uniform(-3, 3));
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(planning::reeds_shepp_shortest({0, 0, 0}, goals[i++ & 255U], 3.0));
  }
}
BENCHMARK(BM_ReedsSheppShortest);

void BM_DareDoubleIntegrator(benchmark::State& state) {
  const models::DoubleIntegrator di(2, 0.05);
  const MatX q = MatX::Identity(4, 4);
  const MatX r = MatX::Identity(2, 2) * 0.1;
  for (auto _ : state) {
    benchmark::DoNotOptimize(control::solve_dare(di.a(), di.b(), q, r));
  }
}
BENCHMARK(BM_DareDoubleIntegrator);

void BM_IcpPointToLine(benchmark::State& state) {
  std::vector<Vec2> target;
  for (int i = 0; i < 400; ++i) {
    const double t = i / 400.0;
    target.emplace_back(10.0 * t, i % 2 == 0 ? 0.0 : 5.0 * t);
  }
  const geometry::SE2 offset(0.2, -0.1, 0.1);
  std::vector<Vec2> source;
  for (const auto& p : target)
    source.push_back(offset.inverse() * p);
  registration::IcpParams params;
  params.method = registration::IcpMethod::kPointToLine;
  for (auto _ : state) {
    benchmark::DoNotOptimize(registration::icp_2d(source, target, params));
  }
}
BENCHMARK(BM_IcpPointToLine)->Unit(benchmark::kMicrosecond);

}  // namespace
