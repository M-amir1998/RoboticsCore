#pragma once

/// @file se3.hpp
/// @brief Spatial rigid transforms SE(3).
///
/// Tangent-space convention: xi = (v, omega) with the translational part first, matching
/// SE2. The adjoint therefore has the block form [[R, hat(t) R], [0, R]].

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/so3.hpp"

namespace roboticscore::geometry {

/// Element of SE(3): p' = R p + t.
class SE3 {
 public:
  /// Identity transform.
  SE3() = default;
  SE3(const SO3& rotation, const Vec3& translation)
      : rotation_(rotation), translation_(translation) {}

  /// Exponential map se(3) -> SE(3) for xi = (v, omega).
  [[nodiscard]] static SE3 exp(const Vec6& xi);

  /// Logarithm map SE(3) -> se(3).
  [[nodiscard]] Vec6 log() const;

  /// Builds a transform from a 4x4 homogeneous matrix; the rotation block is projected onto
  /// SO(3).
  /// @throws std::invalid_argument if the last row is not (0, 0, 0, 1) or the rotation block
  ///         cannot be projected (see SO3::from_matrix).
  [[nodiscard]] static SE3 from_matrix(const Mat4& m);

  [[nodiscard]] const SO3& rotation() const noexcept { return rotation_; }
  [[nodiscard]] const Vec3& translation() const noexcept { return translation_; }

  /// 4x4 homogeneous matrix.
  [[nodiscard]] Mat4 matrix() const;

  [[nodiscard]] SE3 inverse() const;
  [[nodiscard]] SE3 operator*(const SE3& other) const;
  [[nodiscard]] Vec3 operator*(const Vec3& p) const { return rotation_ * p + translation_; }

  /// Relative transform this^-1 * other.
  [[nodiscard]] SE3 between(const SE3& other) const { return inverse() * other; }

  /// 6x6 adjoint such that T * exp(xi) * T^-1 = exp(Ad(T) xi).
  [[nodiscard]] Mat6 adjoint() const;

  /// Geodesic interpolation a * exp(t * log(a^-1 b)).
  [[nodiscard]] static SE3 interpolate(const SE3& a, const SE3& b, double t);

  /// se(3) hat operator: (v, omega) -> 4x4 twist matrix.
  [[nodiscard]] static Mat4 hat(const Vec6& xi);

  /// se(3) vee operator (inverse of hat).
  [[nodiscard]] static Vec6 vee(const Mat4& m);

 private:
  SO3 rotation_;
  Vec3 translation_ = Vec3::Zero();
};

/// Pose of a body in 3D.
using Pose3 = SE3;

}  // namespace roboticscore::geometry
