// End-to-end checks of the integrated pipelines that also drive the README animations.

#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "roboticscore/control/geometric_trackers.hpp"
#include "roboticscore/control/lqr.hpp"
#include "roboticscore/control/mpc.hpp"
#include "roboticscore/trajectory/tracks.hpp"

#include "pipelines.hpp"

namespace roboticscore::pipelines {
namespace {

class HeroPipeline : public ::testing::TestWithParam<std::uint64_t> {};

TEST_P(HeroPipeline, ReachesGoalWithoutCollision) {
  const HeroResult r = run_hero(GetParam());
  ASSERT_FALSE(r.global_path.empty());
  EXPECT_TRUE(r.reached_goal);
  EXPECT_FALSE(r.collision);
  EXPECT_LT(r.position_rmse, 0.1);
  EXPECT_LT(r.heading_rmse, 0.05);
  EXPECT_GT(r.map_accuracy, 0.9);
  for (const auto& f : r.frames) {
    ASSERT_GT(f.clearance, 0.3) << "t = " << f.time;
  }
}

INSTANTIATE_TEST_SUITE_P(Seeds, HeroPipeline, ::testing::Values(1U, 2U, 3U, 4U));

TEST(ParkingPipeline, HybridAStarAndMpcParkTheCar) {
  const ParkingResult r = run_parking();
  ASSERT_TRUE(r.planned_ok);
  EXPECT_TRUE(r.parked);
  EXPECT_LT(r.final_position_error, 0.3);
  EXPECT_LT(r.final_heading_error, 0.1);
  EXPECT_LT(r.max_cross_track, 0.4);
}

TEST(LoopSlamPipeline, LoopClosuresRemoveDrift) {
  for (const std::uint64_t seed : {1U, 2U, 3U}) {
    const LoopSlamResult r = run_loop_slam(seed);
    EXPECT_FALSE(r.loop_closures.empty());
    EXPECT_LT(r.ate_after, 0.5 * r.ate_before) << "seed " << seed;
    EXPECT_LT(r.chi2.back(), r.chi2.front());
  }
}

TEST(ControllerPipeline, AllControllersFinishAllTracks) {
  const models::BicycleParams car{2.7, 0.6, 10.0, 3.0};
  for (const auto& name : trajectory::track_names()) {
    auto path = trajectory::make_track(trajectory::track_from_name(name), 5.0);
    path.apply_speed_profile(6.0, 1.5, 2.0, 0.0, 0.0);
    std::unique_ptr<control::PathTracker> trackers[] = {
        std::make_unique<control::PurePursuitTracker>(car),
        std::make_unique<control::StanleyTracker>(car), std::make_unique<control::LqrTracker>(car),
        std::make_unique<control::MpcTracker>(car)};
    for (auto& t : trackers) {
      const TrackingRun run = run_tracking(*t, path, car, {0.0, 0.5, 0.0, 0.0}, 0.05, 4000);
      SCOPED_TRACE(name + " / " + std::string(t->name()));
      EXPECT_TRUE(run.finished);
      double worst = 0.0;
      for (std::size_t i = 100; i < run.cross_track.size(); ++i) {
        worst = std::max(worst, std::abs(run.cross_track[i]));
      }
      EXPECT_LT(worst, 1.0);  // Pure Pursuit cuts the 4 m-radius corners by up to ~0.5 m
    }
  }
}

}  // namespace
}  // namespace roboticscore::pipelines
