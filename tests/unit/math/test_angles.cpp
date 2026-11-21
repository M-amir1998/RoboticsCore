#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::math {
namespace {

constexpr double kTol = 1e-12;

TEST(WrapToPi, IdentityInsideRange) {
  for (const double a : {-3.0, -1.0, -1e-9, 0.0, 1e-9, 1.0, 3.0}) {
    EXPECT_DOUBLE_EQ(wrap_to_pi(a), a);
  }
}

TEST(WrapToPi, HalfOpenIntervalAtPlusMinusPi) {
  // The interval is [-pi, pi): +pi maps to -pi so wrapping is a function.
  EXPECT_DOUBLE_EQ(wrap_to_pi(kPi), -kPi);
  EXPECT_DOUBLE_EQ(wrap_to_pi(-kPi), -kPi);
  EXPECT_LT(wrap_to_pi(std::nextafter(kPi, 0.0)), kPi);
  EXPECT_NEAR(wrap_to_pi(std::nextafter(kPi, 10.0)), -kPi, kTol);
}

TEST(WrapToPi, MultipleRotations) {
  for (int k = -50; k <= 50; ++k) {
    EXPECT_NEAR(wrap_to_pi(0.3 + k * kTwoPi), 0.3, 1e-12 * (1 + std::abs(k)));
    EXPECT_NEAR(wrap_to_pi(-2.9 + k * kTwoPi), -2.9, 1e-12 * (1 + std::abs(k)));
  }
}

TEST(WrapToPi, ZeroAndFullTurns) {
  EXPECT_DOUBLE_EQ(wrap_to_pi(0.0), 0.0);
  EXPECT_NEAR(wrap_to_pi(kTwoPi), 0.0, kTol);
  EXPECT_NEAR(wrap_to_pi(-kTwoPi), 0.0, kTol);
  EXPECT_NEAR(wrap_to_pi(3 * kPi), -kPi, kTol);
}

TEST(WrapToPi, NonFinitePropagatesNaN) {
  EXPECT_TRUE(std::isnan(wrap_to_pi(std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(std::isnan(wrap_to_pi(std::numeric_limits<double>::infinity())));
}

TEST(WrapTo2Pi, RangeAndBoundaries) {
  EXPECT_DOUBLE_EQ(wrap_to_2pi(0.0), 0.0);
  EXPECT_NEAR(wrap_to_2pi(kTwoPi), 0.0, kTol);
  EXPECT_NEAR(wrap_to_2pi(-kHalfPi), 1.5 * kPi, kTol);
  EXPECT_NEAR(wrap_to_2pi(kPi), kPi, kTol);
  EXPECT_NEAR(wrap_to_2pi(-kPi), kPi, kTol);
  // Tiny negative numbers must not round up to exactly 2*pi.
  const double w = wrap_to_2pi(-1e-20);
  EXPECT_GE(w, 0.0);
  EXPECT_LT(w, kTwoPi);
}

TEST(AngleDifference, ShortestSignedRotation) {
  EXPECT_NEAR(angle_difference(0.1, -0.1), 0.2, kTol);
  EXPECT_NEAR(angle_difference(-0.1, 0.1), -0.2, kTol);
  // Across the discontinuity: from 179 deg to -179 deg is +2 deg, not -358 deg.
  EXPECT_NEAR(angle_difference(deg2rad(-179.0), deg2rad(179.0)), deg2rad(2.0), kTol);
  EXPECT_NEAR(angle_difference(deg2rad(179.0), deg2rad(-179.0)), deg2rad(-2.0), kTol);
  EXPECT_NEAR(angle_difference(kPi, 0.0), -kPi, kTol);
}

TEST(InterpolateAngle, EndpointsAndShortArc) {
  const double a = deg2rad(170.0);
  const double b = deg2rad(-170.0);
  EXPECT_NEAR(interpolate_angle(a, b, 0.0), a, kTol);
  EXPECT_NEAR(interpolate_angle(a, b, 1.0), b, kTol);
  // Midpoint lies on the short arc through 180 deg.
  EXPECT_NEAR(std::abs(interpolate_angle(a, b, 0.5)), kPi, kTol);
}

TEST(AngleConversions, RoundTrip) {
  EXPECT_NEAR(rad2deg(deg2rad(123.456)), 123.456, 1e-12);
  EXPECT_NEAR(deg2rad(180.0), kPi, kTol);
}

TEST(CircularMean, HandlesDiscontinuity) {
  const std::vector<double> angles = {deg2rad(175.0), deg2rad(-175.0)};
  EXPECT_NEAR(std::abs(circular_mean(angles)), kPi, 1e-12);
  EXPECT_NEAR(mean_resultant_length(angles), std::cos(deg2rad(5.0)), 1e-12);
}

TEST(CircularMean, WeightsShiftTheMean) {
  const std::vector<double> angles = {0.0, kHalfPi};
  const std::vector<double> weights = {3.0, 1.0};
  EXPECT_NEAR(circular_mean(angles, weights), std::atan2(1.0, 3.0), 1e-12);
}

TEST(CircularMean, OppositeAnglesHaveZeroResultant) {
  const std::vector<double> angles = {0.0, kPi};
  EXPECT_NEAR(mean_resultant_length(angles), 0.0, 1e-15);
}

TEST(CircularMean, RejectsInvalidInput) {
  const std::vector<double> empty;
  const std::vector<double> angles = {0.0, 1.0};
  const std::vector<double> bad_weights = {1.0};
  EXPECT_THROW((void)circular_mean(empty), std::invalid_argument);
  EXPECT_THROW((void)circular_mean(angles, bad_weights), std::invalid_argument);
}

TEST(WrapToPiProperty, InvariantUnderFullTurnsAndInRange) {
  Random rng(testing::kPropertySeed);
  for (int i = 0; i < 2000; ++i) {
    const double a = rng.uniform(-1e3, 1e3);
    const int k = static_cast<int>(rng.uniform_index(21)) - 10;
    const double w = wrap_to_pi(a);
    ASSERT_GE(w, -kPi);
    ASSERT_LT(w, kPi);
    ASSERT_NEAR(std::cos(w), std::cos(a), 1e-9);
    ASSERT_NEAR(std::sin(w), std::sin(a), 1e-9);
    ASSERT_NEAR(std::abs(angle_difference(wrap_to_pi(a + k * kTwoPi), w)), 0.0, 1e-9);
  }
}

}  // namespace
}  // namespace roboticscore::math
