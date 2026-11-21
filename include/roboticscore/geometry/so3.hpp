#pragma once

/// @file so3.hpp
/// @brief Spatial rotations SO(3), stored as a unit quaternion.
///
/// The quaternion is kept normalised after every operation. Its sign is not canonicalised
/// (q and -q represent the same rotation); log() picks the representative with w >= 0, so
/// rotation vectors returned by log() have norm in [0, pi].

#include <Eigen/Geometry>

#include "roboticscore/core/types.hpp"

namespace roboticscore::geometry {

/// Element of SO(3).
class SO3 {
 public:
  /// Identity rotation.
  SO3() = default;

  /// Rotation represented by @p q, which is normalised.
  /// @throws std::invalid_argument if |q| is (numerically) zero or not finite.
  explicit SO3(const Eigen::Quaterniond& q);

  /// Exponential map so(3) -> SO(3) for a rotation vector (axis * angle).
  [[nodiscard]] static SO3 exp(const Vec3& omega);

  /// Logarithm map SO(3) -> so(3); the result has norm in [0, pi].
  [[nodiscard]] Vec3 log() const;

  /// Nearest rotation (in Frobenius norm) to @p m, computed with an SVD.
  /// @throws std::invalid_argument if @p m is singular or has negative determinant
  ///         (a reflection cannot be projected onto SO(3)).
  [[nodiscard]] static SO3 from_matrix(const Mat3& m);

  /// Rotation about a unit axis.
  /// @throws std::invalid_argument if @p axis is (numerically) zero.
  [[nodiscard]] static SO3 from_axis_angle(const Vec3& axis, double angle);

  /// Intrinsic Z-Y-X (yaw, pitch, roll) Euler angles: R = Rz(yaw) Ry(pitch) Rx(roll).
  [[nodiscard]] static SO3 from_euler_zyx(double yaw, double pitch, double roll);

  /// Inverse of from_euler_zyx returning (yaw, pitch, roll) with pitch in [-pi/2, pi/2].
  /// At gimbal lock (|pitch| = pi/2) roll is set to 0 and yaw absorbs the combined rotation,
  /// so from_euler_zyx(to_euler_zyx()) still reproduces the rotation.
  [[nodiscard]] Vec3 to_euler_zyx() const;

  [[nodiscard]] static SO3 rot_x(double angle);
  [[nodiscard]] static SO3 rot_y(double angle);
  [[nodiscard]] static SO3 rot_z(double angle);

  [[nodiscard]] const Eigen::Quaterniond& quaternion() const noexcept { return q_; }
  [[nodiscard]] Mat3 matrix() const { return q_.toRotationMatrix(); }

  [[nodiscard]] SO3 inverse() const;
  [[nodiscard]] SO3 operator*(const SO3& other) const;
  [[nodiscard]] Vec3 operator*(const Vec3& p) const { return q_ * p; }

  /// Adjoint of SO(3) is the rotation matrix itself.
  [[nodiscard]] Mat3 adjoint() const { return matrix(); }

  /// Geodesic interpolation (equivalent to quaternion slerp along the shortest arc).
  [[nodiscard]] static SO3 interpolate(const SO3& a, const SO3& b, double t);

  /// so(3) hat operator: the skew-symmetric matrix with hat(a) b = a x b.
  [[nodiscard]] static Mat3 hat(const Vec3& omega);

  /// so(3) vee operator; uses the antisymmetric part of @p m.
  [[nodiscard]] static Vec3 vee(const Mat3& m);

  /// Left Jacobian J_l(omega) of SO(3).
  [[nodiscard]] static Mat3 left_jacobian(const Vec3& omega);

  /// Inverse of the left Jacobian, valid for |omega| < 2*pi.
  [[nodiscard]] static Mat3 left_jacobian_inverse(const Vec3& omega);

 private:
  Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
};

}  // namespace roboticscore::geometry
