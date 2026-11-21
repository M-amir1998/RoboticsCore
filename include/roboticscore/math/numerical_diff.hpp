#pragma once

/// @file numerical_diff.hpp
/// @brief Central finite differences for derivatives and Jacobians.
///
/// Step selection: for central differences the truncation error is O(h^2) and the
/// round-off error O(eps/h); the balance is h ~ eps^(1/3) * scale, which is the default.
/// Steps are scaled by max(1, |x_i|) so both small and large coordinates are handled.

#include <algorithm>
#include <cmath>
#include <utility>

#include "roboticscore/core/error.hpp"
#include "roboticscore/core/types.hpp"

namespace roboticscore::math {

/// Default relative step for central differences, cbrt(DBL_EPSILON).
inline constexpr double kCentralDifferenceStep = 6.0554544523933395e-06;

/// Central-difference derivative of a scalar function.
/// @throws std::invalid_argument if @p relative_step is not positive.
template <typename F>
[[nodiscard]] double numerical_derivative(F&& f, double x,
                                          double relative_step = kCentralDifferenceStep) {
  detail::require(relative_step > 0.0, "numerical_derivative: step must be positive");
  const double h = relative_step * std::max(1.0, std::abs(x));
  return (f(x + h) - f(x - h)) / (2.0 * h);
}

/// Central-difference Jacobian of f: R^n -> R^m with a custom output difference.
///
/// @p difference(a, b) must return a - b in the tangent space of the output, which lets
/// callers handle wrapped angles (e.g. bearings near +-pi) correctly.
template <typename F, typename Diff>
[[nodiscard]] MatX numerical_jacobian(F&& f, const VecX& x, Diff&& difference,
                                      double relative_step = kCentralDifferenceStep) {
  detail::require(relative_step > 0.0, "numerical_jacobian: step must be positive");
  const VecX f0 = f(x);
  MatX jacobian(f0.size(), x.size());
  VecX xp = x;
  VecX xm = x;
  for (Index i = 0; i < x.size(); ++i) {
    const double h = relative_step * std::max(1.0, std::abs(x(i)));
    xp(i) = x(i) + h;
    xm(i) = x(i) - h;
    const VecX fp = f(xp);
    const VecX fm = f(xm);
    detail::require(fp.size() == f0.size() && fm.size() == f0.size(),
                    "numerical_jacobian: function output size must not depend on input");
    jacobian.col(i) = difference(fp, fm) / (2.0 * h);
    xp(i) = x(i);
    xm(i) = x(i);
  }
  return jacobian;
}

/// Central-difference Jacobian of f: R^n -> R^m using Euclidean output differences.
template <typename F>
[[nodiscard]] MatX numerical_jacobian(F&& f, const VecX& x,
                                      double relative_step = kCentralDifferenceStep) {
  return numerical_jacobian(
      std::forward<F>(f), x, [](const VecX& a, const VecX& b) -> VecX { return a - b; },
      relative_step);
}

}  // namespace roboticscore::math
