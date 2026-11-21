#include <cmath>

#include "roboticscore/core/error.hpp"
#include "roboticscore/geometry/se2.hpp"
#include "roboticscore/geometry/so2.hpp"

namespace roboticscore::geometry {
namespace {

// Below this angle the closed-form coefficients lose precision and Taylor series are used.
constexpr double kSmallAngle = 1e-6;

}  // namespace

// ----------------------------------------------------------------------------- SO(2)

SO2 SO2::from_matrix(const Mat2& m) {
  const double norm = std::hypot(m(0, 0), m(1, 0));
  detail::require(norm > 1e-12, "SO2::from_matrix: first column must be non-zero");
  return SO2(std::atan2(m(1, 0), m(0, 0)));
}

SO2 SO2::normalized() const {
  const double n = std::hypot(c_, s_);
  return {c_ / n, s_ / n};
}

// ----------------------------------------------------------------------------- SE(2)

Mat2 SE2::translation_jacobian(double omega) {
  double a = 0.0;  // sin(w) / w
  double b = 0.0;  // (1 - cos(w)) / w
  if (std::abs(omega) < kSmallAngle) {
    const double w2 = omega * omega;
    a = 1.0 - w2 / 6.0;
    b = omega / 2.0 - omega * w2 / 24.0;
  } else {
    const double s = std::sin(0.5 * omega);
    a = std::sin(omega) / omega;
    b = 2.0 * s * s / omega;  // (1 - cos w) / w without cancellation
  }
  Mat2 v;
  v << a, -b, b, a;
  return v;
}

SE2 SE2::exp(const Vec3& xi) {
  const double omega = xi(2);
  return {SO2(omega), translation_jacobian(omega) * xi.head<2>()};
}

Vec3 SE2::log() const {
  const double omega = rotation_.log();
  // V(w) = [[a, -b], [b, a]] has the closed-form inverse [[a, b], [-b, a]] / (a^2 + b^2).
  const Mat2 v = translation_jacobian(omega);
  const double a = v(0, 0);
  const double b = v(1, 0);
  const double det = a * a + b * b;
  const Vec2 rho((a * translation_.x() + b * translation_.y()) / det,
                 (-b * translation_.x() + a * translation_.y()) / det);
  return {rho.x(), rho.y(), omega};
}

SE2 SE2::from_matrix(const Mat3& m) {
  detail::require(
      std::abs(m(2, 0)) < 1e-12 && std::abs(m(2, 1)) < 1e-12 && std::abs(m(2, 2) - 1.0) < 1e-12,
      "SE2::from_matrix: last row must be (0, 0, 1)");
  return {SO2::from_matrix(m.topLeftCorner<2, 2>()), m.topRightCorner<2, 1>()};
}

Mat3 SE2::matrix() const {
  Mat3 m = Mat3::Identity();
  m.topLeftCorner<2, 2>() = rotation_.matrix();
  m.topRightCorner<2, 1>() = translation_;
  return m;
}

Mat3 SE2::adjoint() const {
  Mat3 ad = Mat3::Identity();
  ad.topLeftCorner<2, 2>() = rotation_.matrix();
  ad(0, 2) = translation_.y();
  ad(1, 2) = -translation_.x();
  return ad;
}

SE2 SE2::interpolate(const SE2& a, const SE2& b, double t) {
  return a * SE2::exp(t * a.between(b).log());
}

Mat3 SE2::hat(const Vec3& xi) {
  Mat3 m = Mat3::Zero();
  m.topLeftCorner<2, 2>() = SO2::hat(xi(2));
  m.topRightCorner<2, 1>() = xi.head<2>();
  return m;
}

Vec3 SE2::vee(const Mat3& m) {
  return {m(0, 2), m(1, 2), SO2::vee(m.topLeftCorner<2, 2>())};
}

}  // namespace roboticscore::geometry
