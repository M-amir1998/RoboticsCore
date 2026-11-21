#pragma once

/// @file integration.hpp
/// @brief Fixed-step ODE integrators and quadrature rules.
///
/// The ODE steppers are templates over the state type so they work with Eigen fixed-size
/// vectors, dynamic vectors and scalars without allocation or type erasure.

#include <span>

#include "roboticscore/core/error.hpp"

namespace roboticscore::math {

/// One explicit Euler step of dx/dt = f(t, x). Local error O(dt^2), global O(dt).
template <typename State, typename F>
[[nodiscard]] State euler_step(F&& f, const State& x, double t, double dt) {
  return x + dt * f(t, x);
}

/// One classical 4th-order Runge-Kutta step of dx/dt = f(t, x). Global error O(dt^4).
template <typename State, typename F>
[[nodiscard]] State rk4_step(F&& f, const State& x, double t, double dt) {
  const State k1 = f(t, x);
  const State k2 = f(t + 0.5 * dt, State(x + (0.5 * dt) * k1));
  const State k3 = f(t + 0.5 * dt, State(x + (0.5 * dt) * k2));
  const State k4 = f(t + dt, State(x + dt * k3));
  return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/// Trapezoidal rule for samples (xs, ys) with non-uniform spacing.
/// @throws std::invalid_argument if sizes differ or fewer than two samples are given.
[[nodiscard]] double trapezoid(std::span<const double> xs, std::span<const double> ys);

/// Composite Simpson rule for a callable over [a, b] with an even number of intervals.
/// Exact for cubic polynomials; error O(h^4) for smooth integrands.
/// @throws std::invalid_argument if @p intervals is not a positive even number.
template <typename F>
[[nodiscard]] double simpson(F&& f, double a, double b, int intervals) {
  detail::require(intervals > 0 && intervals % 2 == 0,
                  "simpson: number of intervals must be a positive even number");
  const double h = (b - a) / intervals;
  double sum = f(a) + f(b);
  for (int i = 1; i < intervals; ++i) {
    sum += (i % 2 == 1 ? 4.0 : 2.0) * f(a + i * h);
  }
  return sum * h / 3.0;
}

}  // namespace roboticscore::math
