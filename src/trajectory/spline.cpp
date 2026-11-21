#include "roboticscore/trajectory/spline.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/interpolation.hpp"

namespace roboticscore::trajectory {
namespace {

// Solves a tridiagonal system with sub-diagonal a, diagonal b, super-diagonal c (Thomas).
std::vector<double> solve_tridiagonal(std::vector<double> a, std::vector<double> b,
                                      std::vector<double> c, std::vector<double> d) {
  const std::size_t n = b.size();
  for (std::size_t i = 1; i < n; ++i) {
    const double w = a[i] / b[i - 1];
    b[i] -= w * c[i - 1];
    d[i] -= w * d[i - 1];
  }
  std::vector<double> x(n);
  x[n - 1] = d[n - 1] / b[n - 1];
  for (std::size_t i = n - 1; i-- > 0;) {
    x[i] = (d[i] - c[i] * x[i + 1]) / b[i];
  }
  return x;
}

std::vector<double> chord_lengths(const std::vector<Vec2>& pts) {
  detail::require(pts.size() >= 2, "CubicSpline2D: at least two waypoints are required");
  std::vector<double> s(pts.size(), 0.0);
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const double d = (pts[i] - pts[i - 1]).norm();
    detail::require(d > 1e-12, "CubicSpline2D: consecutive waypoints must differ");
    s[i] = s[i - 1] + d;
  }
  return s;
}

std::vector<double> component(const std::vector<Vec2>& pts, int axis) {
  std::vector<double> v(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    v[i] = pts[i](axis);
  }
  return v;
}

}  // namespace

// ----------------------------------------------------------------------------- CubicSpline

CubicSpline::CubicSpline(std::vector<double> x, std::vector<double> y,
                         std::optional<double> start_slope, std::optional<double> end_slope)
    : x_(std::move(x)), y_(std::move(y)) {
  detail::require(x_.size() >= 2, "CubicSpline: at least two knots are required");
  detail::require(x_.size() == y_.size(), "CubicSpline: x and y must have equal size");
  for (std::size_t i = 1; i < x_.size(); ++i) {
    detail::require(x_[i] > x_[i - 1], "CubicSpline: knots must be strictly increasing");
  }
  const std::size_t n = x_.size();
  std::vector<double> h(n - 1);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    h[i] = x_[i + 1] - x_[i];
  }
  std::vector<double> a(n, 0.0);
  std::vector<double> b(n, 1.0);
  std::vector<double> c(n, 0.0);
  std::vector<double> d(n, 0.0);
  for (std::size_t i = 1; i + 1 < n; ++i) {
    a[i] = h[i - 1];
    b[i] = 2.0 * (h[i - 1] + h[i]);
    c[i] = h[i];
    d[i] = 6.0 * ((y_[i + 1] - y_[i]) / h[i] - (y_[i] - y_[i - 1]) / h[i - 1]);
  }
  if (start_slope) {
    b[0] = 2.0 * h[0];
    c[0] = h[0];
    d[0] = 6.0 * ((y_[1] - y_[0]) / h[0] - *start_slope);
  }
  if (end_slope) {
    a[n - 1] = h[n - 2];
    b[n - 1] = 2.0 * h[n - 2];
    d[n - 1] = 6.0 * (*end_slope - (y_[n - 1] - y_[n - 2]) / h[n - 2]);
  }
  m_ = solve_tridiagonal(std::move(a), std::move(b), std::move(c), std::move(d));
}

double CubicSpline::derivative(double x, int order) const {
  detail::require(order >= 0, "CubicSpline::derivative: order must be non-negative");
  const std::size_t i = math::find_segment(x_, x);
  const double h = x_[i + 1] - x_[i];
  const double a = x_[i + 1] - x;  // distance to the right knot
  const double b = x - x_[i];      // distance to the left knot
  const double mi = m_[i];
  const double mj = m_[i + 1];
  const double ci = y_[i] / h - mi * h / 6.0;
  const double cj = y_[i + 1] / h - mj * h / 6.0;
  switch (order) {
    case 0:
      return mi * a * a * a / (6.0 * h) + mj * b * b * b / (6.0 * h) + ci * a + cj * b;
    case 1:
      return -mi * a * a / (2.0 * h) + mj * b * b / (2.0 * h) - ci + cj;
    case 2:
      return mi * a / h + mj * b / h;
    case 3:
      return (mj - mi) / h;
    default:
      return 0.0;
  }
}

// --------------------------------------------------------------------------- CubicSpline2D

CubicSpline2D::CubicSpline2D(const std::vector<Vec2>& waypoints)
    : s_(chord_lengths(waypoints)),
      x_(s_, component(waypoints, 0)),
      y_(s_, component(waypoints, 1)) {}

Vec2 CubicSpline2D::position(double s) const {
  return {x_(s), y_(s)};
}

Vec2 CubicSpline2D::tangent(double s) const {
  return {x_.derivative(s, 1), y_.derivative(s, 1)};
}

double CubicSpline2D::heading(double s) const {
  const Vec2 t = tangent(s);
  return std::atan2(t.y(), t.x());
}

double CubicSpline2D::curvature(double s) const {
  const double dx = x_.derivative(s, 1);
  const double dy = y_.derivative(s, 1);
  const double ddx = x_.derivative(s, 2);
  const double ddy = y_.derivative(s, 2);
  const double denom = std::pow(dx * dx + dy * dy, 1.5);
  return denom > 1e-12 ? (dx * ddy - dy * ddx) / denom : 0.0;
}

// --------------------------------------------------------------------------------- BSpline

BSpline::BSpline(std::vector<Vec2> control_points, int degree)
    : points_(std::move(control_points)), degree_(degree) {
  detail::require(degree >= 1, "BSpline: degree must be at least 1");
  detail::require(points_.size() >= static_cast<std::size_t>(degree) + 1,
                  "BSpline: need at least degree + 1 control points");
  const std::size_t n = points_.size();
  const std::size_t p = static_cast<std::size_t>(degree);
  // Clamped uniform knot vector with n + p + 1 entries.
  knots_.assign(n + p + 1, 0.0);
  const std::size_t interior = n - p - 1;
  for (std::size_t i = 0; i < interior; ++i) {
    knots_[p + 1 + i] = static_cast<double>(i + 1) / static_cast<double>(interior + 1);
  }
  for (std::size_t i = n; i < knots_.size(); ++i) {
    knots_[i] = 1.0;
  }
}

Vec2 BSpline::evaluate(double u) const {
  u = std::clamp(u, 0.0, 1.0);
  const auto p = static_cast<std::size_t>(degree_);
  const std::size_t n = points_.size();
  // Knot span k with knots_[k] <= u < knots_[k + 1] (last span for u == 1).
  std::size_t k = p;
  while (k < n - 1 && u >= knots_[k + 1]) {
    ++k;
  }
  std::vector<Vec2> d(points_.begin() + static_cast<std::ptrdiff_t>(k - p),
                      points_.begin() + static_cast<std::ptrdiff_t>(k + 1));
  for (std::size_t r = 1; r <= p; ++r) {
    for (std::size_t j = p; j >= r; --j) {
      const std::size_t i = k - p + j;
      const double denom = knots_[i + p + 1 - r] - knots_[i];
      const double alpha = denom > 0.0 ? (u - knots_[i]) / denom : 0.0;
      d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
    }
  }
  return d[p];
}

Vec2 BSpline::derivative(double u) const {
  if (degree_ == 1 && points_.size() == 2) {
    return points_[1] - points_[0];
  }
  // Derivative of a B-spline is a B-spline of degree p - 1 with control points
  // Q_i = p (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1}) on the knot vector without its ends.
  const auto p = static_cast<std::size_t>(degree_);
  std::vector<Vec2> q;
  for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
    const double denom = knots_[i + p + 1] - knots_[i + 1];
    q.push_back(denom > 0.0 ? Vec2(static_cast<double>(p) * (points_[i + 1] - points_[i]) / denom)
                            : Vec2::Zero());
  }
  if (degree_ == 1) {
    // Piecewise-constant derivative: pick the active segment.
    u = std::clamp(u, 0.0, 1.0);
    std::size_t k = 1;
    while (k < points_.size() - 1 && u >= knots_[k + 1])
      ++k;
    return q[k - 1];
  }
  // The derivative spline is again clamped uniform with the same interior knots.
  return BSpline(std::move(q), degree_ - 1).evaluate(u);
}

// ----------------------------------------------------------------------------- BezierCurve

BezierCurve::BezierCurve(std::vector<Vec2> control_points) : points_(std::move(control_points)) {
  detail::require(points_.size() >= 2, "BezierCurve: at least two control points are required");
}

Vec2 BezierCurve::evaluate(double t) const {
  std::vector<Vec2> b = points_;
  for (std::size_t r = 1; r < b.size(); ++r) {
    for (std::size_t i = 0; i + r < b.size(); ++i) {
      b[i] = (1.0 - t) * b[i] + t * b[i + 1];
    }
  }
  return b[0];
}

BezierCurve BezierCurve::derivative() const {
  const double n = static_cast<double>(points_.size() - 1);
  std::vector<Vec2> d;
  for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
    d.push_back(n * (points_[i + 1] - points_[i]));
  }
  if (d.size() == 1) {
    d.push_back(d.front());  // constant derivative represented as a degree-1 curve
  }
  return BezierCurve(std::move(d));
}

std::pair<BezierCurve, BezierCurve> BezierCurve::split(double t) const {
  std::vector<Vec2> b = points_;
  std::vector<Vec2> left{b.front()};
  std::vector<Vec2> right{b.back()};
  for (std::size_t r = 1; r < points_.size(); ++r) {
    for (std::size_t i = 0; i + r < points_.size(); ++i) {
      b[i] = (1.0 - t) * b[i] + t * b[i + 1];
    }
    left.push_back(b.front());
    right.push_back(b[points_.size() - 1 - r]);
  }
  std::reverse(right.begin(), right.end());
  return {BezierCurve(std::move(left)), BezierCurve(std::move(right))};
}

}  // namespace roboticscore::trajectory
