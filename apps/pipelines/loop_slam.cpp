#include <cmath>

#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/slam/fastslam.hpp"
#include "roboticscore/slam/pose_graph.hpp"

#include "pipelines.hpp"

namespace roboticscore::pipelines {

LoopSlamResult run_loop_slam(std::uint64_t seed) {
  LoopSlamResult out;
  math::Random rng(seed);
  slam::PoseGraph2D graph;
  geometry::Pose2 truth(0, 0, 0);
  geometry::Pose2 odom(0, 0, 0);
  out.truth.push_back(truth);
  out.odometry.push_back(odom);
  graph.add_node(odom);
  const Mat3 odom_info =
      Vec3(1.0 / (0.04 * 0.04), 1.0 / (0.04 * 0.04), 1.0 / (0.012 * 0.012)).asDiagonal();
  // Two laps of a 16 m x 8 m block: straight runs and quarter-circle corners of radius 2 m.
  const double ds = 0.5;
  for (int lap = 0; lap < 2; ++lap) {
    for (int side = 0; side < 4; ++side) {
      const int straight = side % 2 == 0 ? 24 : 8;
      const int corner = 6;
      for (int k = 0; k < straight + corner; ++k) {
        const double dtheta = k < straight ? 0.0 : math::kHalfPi / corner;
        const double chord = k < straight ? ds : 2.0 * 2.0 * std::sin(0.5 * dtheta);
        const geometry::Pose2 step =
            geometry::Pose2(chord * std::cos(0.5 * dtheta), chord * std::sin(0.5 * dtheta), dtheta);
        truth = truth * step;
        const geometry::Pose2 meas =
            step * geometry::Pose2(rng.normal(0, 0.04), rng.normal(0, 0.04), rng.normal(0, 0.012));
        odom = odom * meas;
        out.truth.push_back(truth);
        out.odometry.push_back(odom);
        const std::size_t id = graph.add_node(odom);
        graph.add_edge({id - 1, id, meas, odom_info});
      }
    }
  }
  // Simulated place recognition: revisits within 1 m of an earlier pose (not recent ones).
  const Mat3 loop_info =
      Vec3(1.0 / (0.05 * 0.05), 1.0 / (0.05 * 0.05), 1.0 / (0.01 * 0.01)).asDiagonal();
  for (std::size_t i = 0; i < out.truth.size(); i += 3) {
    for (std::size_t j = 0; j + 40 < i; ++j) {
      if ((out.truth[i].translation() - out.truth[j].translation()).norm() < 1.0) {
        const geometry::Pose2 rel = out.truth[j].between(out.truth[i]);
        const geometry::Pose2 meas =
            rel * geometry::Pose2(rng.normal(0, 0.05), rng.normal(0, 0.05), rng.normal(0, 0.01));
        graph.add_edge({j, i, meas, loop_info});
        out.loop_closures.emplace_back(j, i);
        break;
      }
    }
  }
  out.ate_before = slam::absolute_trajectory_error(out.odometry, out.truth, false);
  slam::PoseGraphOptions opt;
  opt.method = slam::PoseGraphOptions::Method::kGaussNewton;
  opt.record_history = true;
  const auto result = graph.optimize(opt);
  out.iterations = result.pose_history;
  out.chi2 = result.chi2_history;
  out.ate_after = slam::absolute_trajectory_error(graph.poses(), out.truth, false);
  return out;
}

}  // namespace roboticscore::pipelines
