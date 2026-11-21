#pragma once

/// @file spline.hpp
/// @brief Interpolating cubic splines, clamped B-splines and Bezier curves.
///
/// CubicSpline interpolates (x_i, y_i) with C2 continuity by solving the tridiagonal system
/// for the knot second derivatives M_i (Thomas algorithm, O(n)):
///   h_{i-1} M_{i-1} + 2 (h_{i-1} + h_i) M_i + h_i M_{i+1}
///       = 6 ((y_{i+1} - y_i) / h_i - (y_i - y_{i-1}) / h_{i-1})
/// with natural (M_0 = M_n = 0) or clamped (prescribed end slopes) boundary conditions.
///
/// CubicSpline2D parameterises a planar curve by cumulative chord length, which approximates
/// arc length and gives well-behaved headings and curvature for path tracking.

#include <optional>
#include <vector>

#include "roboticscore/core/types.hpp"

namespace roboticscore::trajectory {

/// One-dimensional interpolating cubic spline.
class CubicSpline {
 public:
  /// Natural spline when the end slopes are not given, clamped otherwise.
  /// @throws std::invalid_argument for fewer than two knots, mismatched sizes or
  ///         non-increasing x.
  CubicSpline(std::vector<double> x, std::vector<double> y,
              std::optional<double> start_slope = std::nullopt,
              std::optional<double> end_slope = std::nullopt);

  /// k-th derivative (k = 0..3) at @p x; outside the knot range the end cubic is extrapolated.
  [[nodiscard]] double derivative(double x, int order) const;
  [[nodiscard]] double operator()(double x) const { return derivative(x, 0); }

  [[nodiscard]] const std::vector<double>& knots() const noexcept { return x_; }

 private:
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> m_;  // second derivatives at the knots
};

/// Planar curve through waypoints parameterised by cumulative chord length s.
class CubicSpline2D {
 public:
  /// @throws std::invalid_argument for fewer than two waypoints or repeated consecutive points.
  explicit CubicSpline2D(const std::vector<Vec2>& waypoints);

  [[nodiscard]] double length() const noexcept { return s_.back(); }
  [[nodiscard]] Vec2 position(double s) const;
  [[nodiscard]] Vec2 tangent(double s) const;  ///< d(position)/ds (not normalised)
  [[nodiscard]] double heading(double s) const;
  /// Signed curvature (positive = turning left).
  [[nodiscard]] double curvature(double s) const;

 private:
  std::vector<double> s_;
  CubicSpline x_;
  CubicSpline y_;
};

/// Clamped uniform B-spline of a given degree (curve starts/ends at the end control points).
class BSpline {
 public:
  /// @throws std::invalid_argument if degree < 1 or there are fewer than degree + 1 points.
  BSpline(std::vector<Vec2> control_points, int degree = 3);

  /// Point at parameter u in [0, 1] (De Boor's algorithm).
  [[nodiscard]] Vec2 evaluate(double u) const;
  /// First derivative with respect to u.
  [[nodiscard]] Vec2 derivative(double u) const;

  [[nodiscard]] int degree() const noexcept { return degree_; }
  [[nodiscard]] const std::vector<double>& knots() const noexcept { return knots_; }
  [[nodiscard]] const std::vector<Vec2>& control_points() const noexcept { return points_; }

 private:
  std::vector<Vec2> points_;
  int degree_;
  std::vector<double> knots_;
};

/// Bezier curve of arbitrary degree.
class BezierCurve {
 public:
  /// @throws std::invalid_argument for fewer than two control points.
  explicit BezierCurve(std::vector<Vec2> control_points);

  /// Point at t in [0, 1] (De Casteljau's algorithm, numerically stable).
  [[nodiscard]] Vec2 evaluate(double t) const;
  /// Derivative curve (hodograph) of degree n - 1.
  [[nodiscard]] BezierCurve derivative() const;
  /// Splits at t into two curves that together trace the original.
  [[nodiscard]] std::pair<BezierCurve, BezierCurve> split(double t) const;

  [[nodiscard]] int degree() const noexcept { return static_cast<int>(points_.size()) - 1; }
  [[nodiscard]] const std::vector<Vec2>& control_points() const noexcept { return points_; }

 private:
  std::vector<Vec2> points_;
};

}  // namespace roboticscore::trajectory
