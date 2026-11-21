#pragma once

/// @file se2.hpp
/// @brief Planar rigid transforms SE(2), the pose type used by all 2D algorithms.
///
/// Tangent-space convention: xi = (v_x, v_y, omega), translation part first. exp and log are
/// the exact group maps (not the "decoupled" x + theta parameterisation); see
/// docs/mathematics/lie_groups.md for formulas and the small-angle series used near zero.

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/so2.hpp"

namespace roboticscore::geometry {

/// Element of SE(2): a rotation followed by a translation, p' = R p + t.
class SE2 {
 public:
  /// Identity transform.
  SE2() = default;

  SE2(double x, double y, double theta) : rotation_(theta), translation_(x, y) {}
  SE2(const SO2& rotation, const Vec2& translation)
      : rotation_(rotation), translation_(translation) {}

  /// Exponential map se(2) -> SE(2) for xi = (v_x, v_y, omega).
  [[nodiscard]] static SE2 exp(const Vec3& xi);

  /// Logarithm map SE(2) -> se(2); omega is in [-pi, pi).
  [[nodiscard]] Vec3 log() const;

  /// Builds a transform from a 3x3 homogeneous matrix; the rotation block is re-projected
  /// onto SO(2).
  /// @throws std::invalid_argument if the last row is not (0, 0, 1).
  [[nodiscard]] static SE2 from_matrix(const Mat3& m);

  [[nodiscard]] double x() const noexcept { return translation_.x(); }
  [[nodiscard]] double y() const noexcept { return translation_.y(); }
  /// Heading in [-pi, pi).
  [[nodiscard]] double theta() const { return rotation_.angle(); }
  [[nodiscard]] const Vec2& translation() const noexcept { return translation_; }
  [[nodiscard]] const SO2& rotation() const noexcept { return rotation_; }

  /// (x, y, theta) with theta in [-pi, pi).
  [[nodiscard]] Vec3 vector() const { return {x(), y(), theta()}; }

  /// 3x3 homogeneous matrix.
  [[nodiscard]] Mat3 matrix() const;

  [[nodiscard]] SE2 inverse() const {
    const SO2 r_inv = rotation_.inverse();
    return {r_inv, -(r_inv * translation_)};
  }

  /// Composition this * other (apply other first, then this).
  [[nodiscard]] SE2 operator*(const SE2& other) const {
    return {rotation_ * other.rotation_, translation_ + rotation_ * other.translation_};
  }

  /// Transforms a point from the local frame to the parent frame.
  [[nodiscard]] Vec2 operator*(const Vec2& p) const { return rotation_ * p + translation_; }

  /// Relative transform this^-1 * other, i.e. other expressed in this frame.
  [[nodiscard]] SE2 between(const SE2& other) const { return inverse() * other; }

  /// Adjoint matrix such that T * exp(xi) * T^-1 = exp(Ad(T) xi).
  [[nodiscard]] Mat3 adjoint() const;

  /// Geodesic interpolation a * exp(t * log(a^-1 b)), t in [0, 1].
  [[nodiscard]] static SE2 interpolate(const SE2& a, const SE2& b, double t);

  /// se(2) hat operator: (v_x, v_y, omega) -> 3x3 twist matrix.
  [[nodiscard]] static Mat3 hat(const Vec3& xi);

  /// se(2) vee operator (inverse of hat).
  [[nodiscard]] static Vec3 vee(const Mat3& m);

  /// Left Jacobian V(omega) of the translation part of exp: t = V(omega) v.
  [[nodiscard]] static Mat2 translation_jacobian(double omega);

 private:
  SO2 rotation_;
  Vec2 translation_ = Vec2::Zero();
};

/// Pose of a planar robot.
using Pose2 = SE2;

}  // namespace roboticscore::geometry
