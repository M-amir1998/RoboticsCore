#pragma once

/// @file so2.hpp
/// @brief Planar rotations SO(2).
///
/// Stored as a unit complex number (cos, sin) so composition and point rotation need no
/// trigonometric calls. The Lie algebra so(2) is identified with R (the rotation angle).

#include <cmath>

#include "roboticscore/core/types.hpp"
#include "roboticscore/math/angles.hpp"

namespace roboticscore::geometry {

/// Element of SO(2).
class SO2 {
 public:
  /// Identity rotation.
  SO2() = default;

  /// Rotation by @p angle radians (any value; it is not wrapped until angle() is called).
  explicit SO2(double angle) : c_(std::cos(angle)), s_(std::sin(angle)) {}

  /// Exponential map so(2) -> SO(2).
  [[nodiscard]] static SO2 exp(double theta) { return SO2(theta); }

  /// Closest rotation to an arbitrary 2x2 matrix, using atan2 of its first column.
  /// @throws std::invalid_argument if the first column is (numerically) zero.
  [[nodiscard]] static SO2 from_matrix(const Mat2& m);

  /// Logarithm map SO(2) -> so(2), the rotation angle in [-pi, pi).
  [[nodiscard]] double log() const { return math::wrap_to_pi(std::atan2(s_, c_)); }

  /// Rotation angle in [-pi, pi).
  [[nodiscard]] double angle() const { return log(); }

  [[nodiscard]] double cos() const noexcept { return c_; }
  [[nodiscard]] double sin() const noexcept { return s_; }

  /// 2x2 rotation matrix.
  [[nodiscard]] Mat2 matrix() const {
    Mat2 r;
    r << c_, -s_, s_, c_;
    return r;
  }

  [[nodiscard]] SO2 inverse() const noexcept { return {c_, -s_}; }

  /// Composition this * other.
  [[nodiscard]] SO2 operator*(const SO2& other) const noexcept {
    return {c_ * other.c_ - s_ * other.s_, s_ * other.c_ + c_ * other.s_};
  }

  /// Rotates a point.
  [[nodiscard]] Vec2 operator*(const Vec2& p) const noexcept {
    return {c_ * p.x() - s_ * p.y(), s_ * p.x() + c_ * p.y()};
  }

  /// Re-projects onto the unit circle to remove accumulated round-off.
  [[nodiscard]] SO2 normalized() const;

  /// so(2) hat operator: w -> [[0, -w], [w, 0]].
  [[nodiscard]] static Mat2 hat(double w) {
    Mat2 m;
    m << 0.0, -w, w, 0.0;
    return m;
  }

  /// so(2) vee operator (inverse of hat); uses the antisymmetric part of @p m.
  [[nodiscard]] static double vee(const Mat2& m) { return 0.5 * (m(1, 0) - m(0, 1)); }

 private:
  SO2(double c, double s) noexcept : c_(c), s_(s) {}

  double c_ = 1.0;
  double s_ = 0.0;
};

}  // namespace roboticscore::geometry
