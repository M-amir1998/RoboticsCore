#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/control/geometric_trackers.hpp"
#include "roboticscore/control/pid.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/trajectory/tracks.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::control {
namespace {

// ------------------------------------------------------------------------------------ PID

TEST(Pid, ProportionalAndSaturation) {
  PidController pid({2.0, 0.0, 0.0, -1.0, 1.0});
  EXPECT_DOUBLE_EQ(pid.update(0.3, 0.0, 0.1), 0.6);
  EXPECT_DOUBLE_EQ(pid.update(5.0, 0.0, 0.1), 1.0);
  EXPECT_DOUBLE_EQ(pid.update(-5.0, 0.0, 0.1), -1.0);
  EXPECT_THROW(PidController({-1.0}), std::invalid_argument);
  EXPECT_THROW((void)pid.update(0.0, 0.0, 0.0), std::invalid_argument);
}

TEST(Pid, AntiWindupLimitsIntegralDuringSaturation) {
  const auto run = [](AntiWindup mode) {
    PidConfig c{1.0, 2.0, 0.0, -1.0, 1.0};
    c.anti_windup = mode;
    PidController pid(c);
    for (int i = 0; i < 100; ++i) {
      (void)pid.update(10.0, 0.0, 0.1);  // saturated for 10 s
    }
    return pid.last_terms().integral;
  };
  const double none = run(AntiWindup::kNone);
  const double clamp = run(AntiWindup::kClamping);
  const double back = run(AntiWindup::kBackCalculation);
  EXPECT_NEAR(none, 200.0, 1e-9);  // integral of e = 10 over 10 s with Ki = 2
  EXPECT_LT(clamp, 1.0);           // frozen as soon as the output saturates
  EXPECT_LT(back, none);
}

TEST(Pid, AntiWindupRecoversFasterAfterSaturation) {
  // First-order plant y' = -y + u; the setpoint is unreachable for a while, then drops.
  const auto overshoot = [](AntiWindup mode) {
    PidConfig c{2.0, 3.0, 0.0, -1.0, 1.0};
    c.anti_windup = mode;
    PidController pid(c);
    double y = 0.0;
    double peak = 0.0;
    for (int i = 0; i < 1000; ++i) {
      const double sp = i < 300 ? 5.0 : 0.5;
      const double u = pid.update(sp, y, 0.01);
      y += 0.01 * (-y + u);
      if (i >= 300) peak = std::max(peak, y);
    }
    return peak;
  };
  EXPECT_LT(overshoot(AntiWindup::kClamping), overshoot(AntiWindup::kNone));
}

TEST(Pid, DerivativeOnMeasurementHasNoSetpointKick) {
  PidController pid({0.0, 0.0, 1.0});
  (void)pid.update(0.0, 1.0, 0.1);
  EXPECT_DOUBLE_EQ(pid.update(100.0, 1.0, 0.1), 0.0);  // setpoint jump only
  EXPECT_NEAR(pid.update(100.0, 1.5, 0.1), -5.0, 1e-12);
  PidController filtered({0.0, 0.0, 1.0, -1e9, 1e9, 0.5});
  (void)filtered.update(0.0, 1.0, 0.1);
  EXPECT_GT(filtered.update(0.0, 1.5, 0.1), -5.0);  // low-pass attenuates the spike
  filtered.reset();
  EXPECT_DOUBLE_EQ(filtered.last_terms().integral, 0.0);
}

TEST(Pid, PiEliminatesSteadyStateError) {
  PidController pid({1.0, 1.5, 0.0, -10.0, 10.0});
  double y = 0.0;
  for (int i = 0; i < 2000; ++i) {
    const double u = pid.update(2.0, y, 0.01);
    y += 0.01 * (-y + u + 0.5);  // constant disturbance
  }
  EXPECT_NEAR(y, 2.0, 1e-3);
}

// ------------------------------------------------------------------- geometric trackers

const models::BicycleParams kCar{2.5, 0.6, 10.0, 3.0};

TEST(PurePursuit, SteersTowardsPathWithExpectedGeometry) {
  const auto path = trajectory::ReferencePath::from_waypoints({{0, 0}, {50, 0}}, 0.25, 2.0);
  PurePursuitConfig cfg;
  cfg.lookahead_gain = 0.0;
  cfg.lookahead_offset = 4.0;
  PurePursuitTracker pp(kCar, cfg);
  // On the path: no steering.
  EXPECT_NEAR(pp.compute(models::BicycleState(5, 0, 0, 2), path, 0.1).steering, 0.0, 1e-9);
  // 1 m to the left of a straight path: target 4 m ahead along the path.
  pp.reset();
  const VehicleCommand c = pp.compute(models::BicycleState(5, 1, 0, 2), path, 0.1);
  const double ld = std::hypot(4.0, 1.0);
  const double alpha = std::atan2(-1.0, 4.0);
  EXPECT_NEAR(c.steering, std::atan(2.0 * kCar.wheelbase * std::sin(alpha) / ld), 1e-6);
  EXPECT_LT(c.steering, 0.0);
  EXPECT_NEAR(c.acceleration, 0.0, 1e-9);  // already at the reference speed
}

TEST(Stanley, SignConventionsAndSpeedDependence) {
  const auto path = trajectory::ReferencePath::from_waypoints({{0, 0}, {50, 0}}, 0.25, 5.0);
  StanleyTracker st(kCar);
  EXPECT_NEAR(st.compute(models::BicycleState(5, 0, 0, 5), path, 0.1).steering, 0.0, 1e-9);
  st.reset();
  const double left_slow = st.compute(models::BicycleState(5, 0.5, 0, 1), path, 0.1).steering;
  st.reset();
  const double left_fast = st.compute(models::BicycleState(5, 0.5, 0, 8), path, 0.1).steering;
  EXPECT_LT(left_slow, 0.0);
  EXPECT_LT(left_fast, 0.0);
  EXPECT_LT(std::abs(left_fast), std::abs(left_slow));  // gain decreases with speed
  st.reset();
  // Rear axle on the path, yaw 0.2 rad: the front axle sits L sin(0.2) left of the path, so
  // both the heading and the cross-track terms steer right.
  const double e_front = kCar.wheelbase * std::sin(0.2);
  EXPECT_NEAR(st.compute(models::BicycleState(5, 0, 0.2, 5), path, 0.1).steering,
              -0.2 + std::atan2(-1.5 * e_front, 1.0 + 5.0), 1e-3);
}

double closed_loop_rmse(PathTracker& tracker, const trajectory::ReferencePath& path) {
  const models::KinematicBicycle car(kCar);
  models::BicycleState x(0.0, 0.5, 0.0, 0.0);  // start offset from the path
  double ss = 0.0;
  int n = 0;
  for (int k = 0; k < 4000 && !tracker.status().finished; ++k) {
    const VehicleCommand u = tracker.compute(x, path, 0.05);
    x = car.step(x, models::BicycleInput(u.acceleration, u.steering), 0.05);
    if (k > 100) {
      ss += tracker.status().cross_track_error * tracker.status().cross_track_error;
      ++n;
    }
  }
  EXPECT_TRUE(tracker.status().finished) << tracker.name();
  return std::sqrt(ss / std::max(n, 1));
}

TEST(GeometricTrackers, ClosedLoopTrackingOfSCurve) {
  const auto path = trajectory::make_track(trajectory::TrackType::kSCurve, 4.0);
  PurePursuitTracker pp(kCar);
  StanleyTracker st(kCar);
  EXPECT_LT(closed_loop_rmse(pp, path), 0.35);
  EXPECT_LT(closed_loop_rmse(st, path), 0.2);
}

}  // namespace
}  // namespace roboticscore::control
