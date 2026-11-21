#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/planning/local/dwa.hpp"
#include "roboticscore/planning/local/potential_field.hpp"
#include "roboticscore/planning/local/velocity_obstacles.hpp"
#include "roboticscore/planning/sampling/sampling_planners.hpp"
#include "roboticscore/planning/smoothing.hpp"
#include "roboticscore/sim/scenarios.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::planning {
namespace {

using geometry::Circle;
using geometry::Environment2D;
using geometry::Polygon;
using geometry::Pose2;

// ------------------------------------------------------------------------------ smoothing

TEST(Smoothing, ResampleKeepsVerticesAndSpacing) {
  const std::vector<Vec2> path = {{0, 0}, {1, 0}, {1, 2.5}};
  const auto r = resample_polyline(path, 0.5);
  EXPECT_MATRIX_NEAR(r.front(), path.front(), 0.0);
  EXPECT_MATRIX_NEAR(r.back(), path.back(), 0.0);
  EXPECT_NEAR(polyline_length(r), polyline_length(path), 1e-12);
  for (std::size_t i = 1; i < r.size(); ++i) {
    EXPECT_LE((r[i] - r[i - 1]).norm(), 0.5 + 1e-12);
  }
  EXPECT_THROW((void)resample_polyline(path, 0.0), std::invalid_argument);
}

TEST(Smoothing, ShortcutAndGradientKeepPathCollisionFree) {
  const sim::Scenario sc = sim::cluttered_scenario(3, 35);
  SamplingConfig cfg;
  cfg.robot_radius = 0.3;
  cfg.seed = 4;
  const SamplingResult raw =
      rrt(sc.environment, sc.start.translation(), sc.goal.translation(), cfg);
  ASSERT_TRUE(raw.found);
  math::Random rng(1);
  const auto shortcut = shortcut_path(raw.path, sc.environment, 0.3, 300, rng);
  EXPECT_LE(polyline_length(shortcut), polyline_length(raw.path) + 1e-9);
  for (std::size_t i = 1; i < shortcut.size(); ++i) {
    ASSERT_TRUE(sc.environment.segment_free(shortcut[i - 1], shortcut[i], 0.3)) << i;
  }
  // Gradient smoothing of the jagged raw RRT path.
  GradientSmoothingParams p;
  p.robot_radius = 0.3;
  p.iterations = 60;
  const auto dense = resample_polyline(raw.path, 0.25);
  const auto smooth = gradient_smooth(dense, sc.environment, p);
  ASSERT_EQ(smooth.size(), dense.size());
  EXPECT_MATRIX_NEAR(smooth.front(), dense.front(), 0.0);
  EXPECT_MATRIX_NEAR(smooth.back(), dense.back(), 0.0);
  EXPECT_LT(polyline_roughness(smooth), polyline_roughness(dense));
  for (std::size_t i = 1; i < smooth.size(); ++i) {
    ASSERT_TRUE(sc.environment.segment_free(smooth[i - 1], smooth[i], 0.3)) << i;
  }
}

// ------------------------------------------------------------------------------------ DWA

TEST(Dwa, DrivesTowardsGoalWithinDynamicWindow) {
  const Environment2D env({{0, 0}, {20, 20}});
  DwaConfig cfg;
  const models::UnicycleInput v0{0.5, 0.0};
  const DwaResult r = dwa_plan({2, 10, 0}, v0, {18, 10}, env, {}, cfg);
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.command.v, 0.6, 1e-12);  // max reachable within one period
  EXPECT_NEAR(r.command.omega, 0.0, 1e-12);
  for (const auto& c : r.candidates) {
    EXPECT_LE(std::abs(c.command.v - v0.v), cfg.max_accel * cfg.control_period + 1e-12);
    EXPECT_LE(std::abs(c.command.omega - v0.omega), cfg.max_yaw_accel * cfg.control_period + 1e-12);
  }
}

TEST(Dwa, AvoidsObstacleAheadAndNeverSelectsCollidingRollout) {
  Environment2D env({{0, 0}, {20, 20}});
  env.add_obstacle(Circle{{4.0, 10.0}, 0.8});
  DwaConfig cfg;
  Pose2 pose(2, 10, 0);
  models::UnicycleInput vel{0.0, 0.0};
  double min_clearance = std::numeric_limits<double>::infinity();
  for (int k = 0; k < 200; ++k) {
    const DwaResult r = dwa_plan(pose, vel, {18, 10}, env, {}, cfg);
    ASSERT_TRUE(r.valid) << "step " << k;
    EXPECT_GT(r.candidates[r.best].clearance, 0.0);
    vel = r.command;
    pose = models::Unicycle::step(pose, vel, cfg.control_period);
    min_clearance = std::min(min_clearance, env.clearance(pose.translation()) - cfg.robot_radius);
  }
  EXPECT_GT(min_clearance, 0.0);
  EXPECT_GT(pose.x(), 6.0);  // made it past the obstacle
}

TEST(Dwa, CannotEscapeWhenBoxedIn) {
  Environment2D env({{0, 0}, {10, 10}});
  for (const double a : {0.0, 1.0, 2.0, 3.0, 4.0, 5.0}) {
    env.add_obstacle(
        Circle{{5 + 0.7 * std::cos(a * 1.0472), 5 + 0.7 * std::sin(a * 1.0472)}, 0.32});
  }
  // The gaps between the discs are narrower than the robot: small motions inside the cage
  // are admissible, but no admissible rollout ever leaves it.
  const DwaResult r = dwa_plan({5, 5, 0}, {0, 0}, {9, 9}, env, {}, DwaConfig{});
  ASSERT_TRUE(r.valid);
  for (const auto& c : r.candidates) {
    if (!c.admissible) continue;
    for (const auto& p : c.rollout) {
      ASSERT_LT((p.translation() - Vec2(5, 5)).norm(), 0.7);
    }
  }
}

// ------------------------------------------------------------------------- potential field

TEST(PotentialField, ForceIsNegativeGradientOfPotential) {
  Environment2D env({{0, 0}, {10, 10}});
  env.add_obstacle(Circle{{5, 5}, 1.0});
  const PotentialFieldConfig cfg;
  const Vec2 goal(9, 9);
  for (const Vec2 p : {Vec2(3.5, 5.2), Vec2(2, 2), Vec2(5, 7.2)}) {
    constexpr double h = 1e-5;
    const Vec2 grad(
        (potential(p + Vec2(h, 0), goal, env, cfg) - potential(p - Vec2(h, 0), goal, env, cfg))
            / (2 * h),
        (potential(p + Vec2(0, h), goal, env, cfg) - potential(p - Vec2(0, h), goal, env, cfg))
            / (2 * h));
    EXPECT_MATRIX_NEAR(potential_force(p, goal, env, cfg), Vec2(-grad), 1e-4);
  }
}

TEST(PotentialField, ReachesGoalInOpenSpace) {
  Environment2D env({{0, 0}, {20, 20}});
  env.add_obstacle(Circle{{10, 6}, 1.0});
  const auto r = potential_field_plan(env, {3, 3}, {17, 9}, PotentialFieldConfig{});
  EXPECT_EQ(r.status, PotentialFieldStatus::kReachedGoal);
}

TEST(PotentialField, GetsTrappedInUShapedObstacle) {
  // Documented failure mode: the goal lies behind a U-shaped wall opening away from the robot.
  Environment2D env({{0, 0}, {20, 20}});
  env.add_obstacle(Polygon::from_aabb({{9.5, 6}, {10.5, 14}}));
  env.add_obstacle(Polygon::from_aabb({{6, 6}, {10.5, 7}}));
  env.add_obstacle(Polygon::from_aabb({{6, 13}, {10.5, 14}}));
  const auto r = potential_field_plan(env, {7.5, 10}, {16, 10}, PotentialFieldConfig{});
  EXPECT_EQ(r.status, PotentialFieldStatus::kLocalMinimum);
  EXPECT_GT((r.path.back() - Vec2(16, 10)).norm(), 3.0);
}

// ----------------------------------------------------------------- velocity obstacles

TEST(VelocityObstacles, TimeToCollisionAnalytic) {
  // Head-on at 2 m/s closing speed, 10 m apart, combined radius 1 m: t = 9 / 2.
  EXPECT_NEAR(time_to_collision({10, 0}, {2, 0}, 1.0), 4.5, 1e-12);
  EXPECT_TRUE(std::isinf(time_to_collision({10, 0}, {-2, 0}, 1.0)));  // separating
  EXPECT_TRUE(std::isinf(time_to_collision({10, 5}, {1, 0}, 1.0)));   // miss
  EXPECT_DOUBLE_EQ(time_to_collision({0.5, 0}, {0, 0}, 1.0), 0.0);    // overlapping
}

TEST(VelocityObstacles, KeepsPreferredVelocityWhenFree) {
  const VelocityObstacleConfig cfg;
  const VoNeighbor far{{50, 50}, {0, 0}, 0.3, false};
  const auto s = select_velocity({0, 0}, {0, 0}, 0.3, {0.8, 0.0}, std::span(&far, 1), cfg);
  EXPECT_TRUE(s.collision_free);
  EXPECT_MATRIX_NEAR(s.velocity, Vec2(0.8, 0.0), 1e-12);
}

TEST(VelocityObstacles, ReciprocalAgentsSwapWithoutCollision) {
  // Classic benchmark: agents on a circle swap to antipodal positions.
  const int n = 6;
  const double radius = 0.3;
  std::vector<Vec2> pos(n);
  std::vector<Vec2> vel(n, Vec2::Zero());
  std::vector<Vec2> goal(n);
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * math::kPi * i / n;
    pos[static_cast<std::size_t>(i)] = 4.0 * Vec2(std::cos(a), std::sin(a));
    goal[static_cast<std::size_t>(i)] = -pos[static_cast<std::size_t>(i)];
  }
  const VelocityObstacleConfig cfg;  // default side bias breaks the symmetric deadlock
  double min_sep = std::numeric_limits<double>::infinity();
  const double dt = 0.1;
  for (int step = 0; step < 300; ++step) {
    std::vector<Vec2> next(n);
    for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
      std::vector<VoNeighbor> others;
      for (std::size_t j = 0; j < static_cast<std::size_t>(n); ++j) {
        if (j != i) others.push_back({pos[j], vel[j], radius, true});
      }
      Vec2 pref = goal[i] - pos[i];
      if (pref.norm() > cfg.max_speed) pref *= cfg.max_speed / pref.norm();
      next[i] = select_velocity(pos[i], vel[i], radius, pref, others, cfg).velocity;
    }
    for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
      vel[i] = next[i];
      pos[i] += vel[i] * dt;
    }
    for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
      for (std::size_t j = i + 1; j < static_cast<std::size_t>(n); ++j) {
        min_sep = std::min(min_sep, (pos[i] - pos[j]).norm());
      }
    }
  }
  EXPECT_GT(min_sep, 2.0 * radius);  // never in contact
  for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
    EXPECT_LT((pos[i] - goal[i]).norm(), 0.5) << "agent " << i;
  }
}

}  // namespace
}  // namespace roboticscore::planning
