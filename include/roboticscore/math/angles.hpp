#pragma once

/// @file angles.hpp
/// @brief Planar angle utilities with a single, explicit wrapping convention.
///
/// Convention: wrapped angles lie in the half-open interval [-pi, pi). Using a half-open
/// interval makes wrapping a true function (pi and -pi map to the same representative),
/// which matters for regression tests and for comparing headings.
/// Non-finite inputs propagate as NaN.

#include <cmath>
#include <numbers>
#include <span>

namespace roboticscore::math {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline constexpr double kHalfPi = 0.5 * std::numbers::pi;

/// Converts degrees to radians.
[[nodiscard]] constexpr double deg2rad(double degrees) noexcept {
  return degrees * (kPi / 180.0);
}

/// Converts radians to degrees.
[[nodiscard]] constexpr double rad2deg(double radians) noexcept {
  return radians * (180.0 / kPi);
}

/// Wraps an angle to [-pi, pi).
///
/// Uses std::remainder, which is exact with respect to the floating-point value of 2*pi. That
/// value differs from the true 2*pi by about 2.4e-16, so for very large inputs the result is off
/// by roughly |angle| * 4e-17 rad (e.g. 4e-11 rad at |angle| = 1e6). Headings in this library
/// are wrapped after every update, so inputs stay within a few turns in practice.
[[nodiscard]] inline double wrap_to_pi(double angle) noexcept {
  double r = std::remainder(angle, kTwoPi);  // r in [-pi, pi]
  if (r >= kPi) {
    r -= kTwoPi;
  }
  return r;
}

/// Wraps an angle to [0, 2*pi).
[[nodiscard]] inline double wrap_to_2pi(double angle) noexcept {
  double r = std::fmod(angle, kTwoPi);  // r in (-2pi, 2pi)
  if (r < 0.0) {
    r += kTwoPi;
  }
  if (r >= kTwoPi) {  // r was a tiny negative number that rounded up to 2*pi
    r = 0.0;
  }
  return r;
}

/// Signed shortest rotation that takes @p from to @p to, i.e. wrap_to_pi(to - from).
/// The result is in [-pi, pi); an exact half-turn is reported as -pi.
[[nodiscard]] inline double angle_difference(double to, double from) noexcept {
  return wrap_to_pi(to - from);
}

/// Interpolates along the shortest arc from @p a (t = 0) to @p b (t = 1).
/// For an exact half-turn the rotation direction follows angle_difference (negative).
[[nodiscard]] inline double interpolate_angle(double a, double b, double t) noexcept {
  return wrap_to_pi(a + t * angle_difference(b, a));
}

/// Mean resultant length R in [0, 1] of a set of weighted angles. R close to zero means
/// the circular mean is ill-defined (e.g. angles spread uniformly or two opposite angles).
/// Weights may be empty (uniform weighting).
[[nodiscard]] double mean_resultant_length(std::span<const double> angles,
                                           std::span<const double> weights = {});

/// Weighted circular mean atan2(sum w sin, sum w cos), wrapped to [-pi, pi).
/// Weights may be empty (uniform weighting). Callers that need to detect an undefined mean
/// should check mean_resultant_length(); this function returns 0 in that degenerate case.
/// @throws std::invalid_argument if angles is empty or weights has a different size.
[[nodiscard]] double circular_mean(std::span<const double> angles,
                                   std::span<const double> weights = {});

}  // namespace roboticscore::math
