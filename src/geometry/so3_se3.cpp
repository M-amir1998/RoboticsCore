#include <algorithm>
#include <cmath>

#include <Eigen/SVD>

#include "roboticscore/core/error.hpp"
#include "roboticscore/geometry/se3.hpp"
#include "roboticscore/geometry/so3.hpp"

namespace roboticscore::geometry {
namespace {

// Below kSmallAngle the closed forms for exp/log degenerate to 0/0 and series are used.
constexpr double kSmallAngle = 1e-6;
// The Jacobian coefficients (t - sin t) / t^3 and (1 - (t/2) cot(t/2)) / t^2 cancel
// catastrophically well above kSmallAngle; their 3-term series are accurate to ~1e-18 here.
constexpr double kSeriesAngle = 1e-2;

}  // namespace

// ----------------------------------------------------------------------------- SO(3)

SO3::SO3(const Eigen::Quaterniond& q) {
  const double n = q.norm();
  detail::require(std::isfinite(n) && n > 1e-12, "SO3: quaternion must have non-zero norm");
  q_ = Eigen::Quaterniond(q.coeffs() / n);
}

SO3 SO3::exp(const Vec3& omega) {
  const double theta = omega.norm();
  double real = 0.0;
  double imag_scale = 0.0;  // sin(theta/2) / theta
  if (theta < kSmallAngle) {
    const double t2 = theta * theta;
    real = 1.0 - t2 / 8.0;
    imag_scale = 0.5 - t2 / 48.0;
  } else {
    real = std::cos(0.5 * theta);
    imag_scale = std::sin(0.5 * theta) / theta;
  }
  const Vec3 imag = imag_scale * omega;
  return SO3(Eigen::Quaterniond(real, imag.x(), imag.y(), imag.z()));
}

Vec3 SO3::log() const {
  // Choose the hemisphere w >= 0 so the rotation angle lies in [0, pi].
  const double sign = q_.w() < 0.0 ? -1.0 : 1.0;
  const double w = sign * q_.w();
  const Vec3 v = sign * q_.vec();
  const double n = v.norm();
  if (n < kSmallAngle) {
    // 2 atan(n / w) / n = (2 / w) (1 - n^2 / (3 w^2) + ...)
    return (2.0 / w) * (1.0 - n * n / (3.0 * w * w)) * v;
  }
  const double theta = 2.0 * std::atan2(n, w);
  return (theta / n) * v;
}

SO3 SO3::from_matrix(const Mat3& m) {
  const double det = m.determinant();
  detail::require(std::isfinite(det) && det > 1e-12,
                  "SO3::from_matrix: matrix must have positive determinant");
  const Eigen::JacobiSVD<Mat3> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3 r = svd.matrixU() * svd.matrixV().transpose();
  if (r.determinant() < 0.0) {
    Mat3 u = svd.matrixU();
    u.col(2) *= -1.0;
    r = u * svd.matrixV().transpose();
  }
  return SO3(Eigen::Quaterniond(r));
}

SO3 SO3::from_axis_angle(const Vec3& axis, double angle) {
  const double n = axis.norm();
  detail::require(n > 1e-12, "SO3::from_axis_angle: axis must be non-zero");
  return SO3(Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis / n)));
}

SO3 SO3::from_euler_zyx(double yaw, double pitch, double roll) {
  return rot_z(yaw) * rot_y(pitch) * rot_x(roll);
}

Vec3 SO3::to_euler_zyx() const {
  const Mat3 r = matrix();
  // atan2 with the hypotenuse is well conditioned near |pitch| = pi/2, unlike asin(-r20).
  const double cos_pitch = std::hypot(r(0, 0), r(1, 0));
  const double pitch = std::atan2(-r(2, 0), cos_pitch);
  if (cos_pitch < 1e-12) {
    // Gimbal lock: yaw and roll rotate about the same axis; put everything into yaw.
    const double yaw = std::atan2(-r(0, 1), r(1, 1));
    return {yaw, pitch, 0.0};
  }
  const double roll = std::atan2(r(2, 1), r(2, 2));
  const double yaw = std::atan2(r(1, 0), r(0, 0));
  return {yaw, pitch, roll};
}

SO3 SO3::rot_x(double angle) {
  return SO3(Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vec3::UnitX())));
}
SO3 SO3::rot_y(double angle) {
  return SO3(Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vec3::UnitY())));
}
SO3 SO3::rot_z(double angle) {
  return SO3(Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vec3::UnitZ())));
}

SO3 SO3::inverse() const {
  return SO3(q_.conjugate());
}

SO3 SO3::operator*(const SO3& other) const {
  return SO3(q_ * other.q_);
}

SO3 SO3::interpolate(const SO3& a, const SO3& b, double t) {
  return a * SO3::exp(t * (a.inverse() * b).log());
}

Mat3 SO3::hat(const Vec3& omega) {
  Mat3 m;
  m << 0.0, -omega.z(), omega.y(),  //
      omega.z(), 0.0, -omega.x(),   //
      -omega.y(), omega.x(), 0.0;
  return m;
}

Vec3 SO3::vee(const Mat3& m) {
  return 0.5 * Vec3(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), m(1, 0) - m(0, 1));
}

Mat3 SO3::left_jacobian(const Vec3& omega) {
  const double theta = omega.norm();
  const double t2 = theta * theta;
  const Mat3 w = hat(omega);
  double a = 0.0;  // (1 - cos t) / t^2, evaluated as 2 sin^2(t/2) / t^2 to avoid cancellation
  double b = 0.0;  // (t - sin t) / t^3
  if (theta < kSmallAngle) {
    a = 0.5 - t2 / 24.0;
  } else {
    const double s = std::sin(0.5 * theta);
    a = 2.0 * s * s / t2;
  }
  if (theta < kSeriesAngle) {
    b = 1.0 / 6.0 - t2 / 120.0 + t2 * t2 / 5040.0;
  } else {
    b = (theta - std::sin(theta)) / (t2 * theta);
  }
  return Mat3::Identity() + a * w + b * w * w;
}

Mat3 SO3::left_jacobian_inverse(const Vec3& omega) {
  const double theta = omega.norm();
  const double t2 = theta * theta;
  const Mat3 w = hat(omega);
  double c = 0.0;  // (1 - (t/2) cot(t/2)) / t^2
  if (theta < kSeriesAngle) {
    c = 1.0 / 12.0 + t2 / 720.0 + t2 * t2 / 30240.0;
  } else {
    const double half = 0.5 * theta;
    c = (1.0 - half * std::cos(half) / std::sin(half)) / t2;
  }
  return Mat3::Identity() - 0.5 * w + c * w * w;
}

// ----------------------------------------------------------------------------- SE(3)

SE3 SE3::exp(const Vec6& xi) {
  const Vec3 v = xi.head<3>();
  const Vec3 omega = xi.tail<3>();
  return {SO3::exp(omega), SO3::left_jacobian(omega) * v};
}

Vec6 SE3::log() const {
  const Vec3 omega = rotation_.log();
  Vec6 xi;
  xi.head<3>() = SO3::left_jacobian_inverse(omega) * translation_;
  xi.tail<3>() = omega;
  return xi;
}

SE3 SE3::from_matrix(const Mat4& m) {
  detail::require(
      m.row(3).head<3>().cwiseAbs().maxCoeff() < 1e-12 && std::abs(m(3, 3) - 1.0) < 1e-12,
      "SE3::from_matrix: last row must be (0, 0, 0, 1)");
  return {SO3::from_matrix(m.topLeftCorner<3, 3>()), m.topRightCorner<3, 1>()};
}

Mat4 SE3::matrix() const {
  Mat4 m = Mat4::Identity();
  m.topLeftCorner<3, 3>() = rotation_.matrix();
  m.topRightCorner<3, 1>() = translation_;
  return m;
}

SE3 SE3::inverse() const {
  const SO3 r_inv = rotation_.inverse();
  return {r_inv, -(r_inv * translation_)};
}

SE3 SE3::operator*(const SE3& other) const {
  return {rotation_ * other.rotation_, translation_ + rotation_ * other.translation_};
}

Mat6 SE3::adjoint() const {
  const Mat3 r = rotation_.matrix();
  Mat6 ad = Mat6::Zero();
  ad.topLeftCorner<3, 3>() = r;
  ad.topRightCorner<3, 3>() = SO3::hat(translation_) * r;
  ad.bottomRightCorner<3, 3>() = r;
  return ad;
}

SE3 SE3::interpolate(const SE3& a, const SE3& b, double t) {
  return a * SE3::exp(t * a.between(b).log());
}

Mat4 SE3::hat(const Vec6& xi) {
  Mat4 m = Mat4::Zero();
  m.topLeftCorner<3, 3>() = SO3::hat(xi.tail<3>());
  m.topRightCorner<3, 1>() = xi.head<3>();
  return m;
}

Vec6 SE3::vee(const Mat4& m) {
  Vec6 xi;
  xi.head<3>() = m.topRightCorner<3, 1>();
  xi.tail<3>() = SO3::vee(m.topLeftCorner<3, 3>());
  return xi;
}

}  // namespace roboticscore::geometry
