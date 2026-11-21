#pragma once

/// @file interpolation.hpp
/// @brief One-dimensional table interpolation helpers.

#include <cstddef>
#include <span>

namespace roboticscore::math {

/// Behaviour of interp1 outside the table range.
enum class Extrapolation {
  kClamp,   ///< Return the boundary value.
  kLinear,  ///< Extend the first/last segment linearly.
};

/// Piecewise-linear interpolation of the table (xs, ys) at @p x.
///
/// Complexity: O(log n) via binary search.
/// @throws std::invalid_argument if the table has fewer than two points, the spans differ in
///         size, or xs is not strictly increasing.
[[nodiscard]] double interp1(std::span<const double> xs, std::span<const double> ys, double x,
                             Extrapolation mode = Extrapolation::kClamp);

/// Index i of the segment [xs[i], xs[i+1]] containing @p x for a strictly increasing table,
/// clamped to [0, n-2]. Useful when several quantities share the same abscissa.
/// @throws std::invalid_argument if xs has fewer than two entries.
[[nodiscard]] std::size_t find_segment(std::span<const double> xs, double x);

/// Bilinear interpolation on the unit square with corner values f(0,0), f(1,0), f(0,1),
/// f(1,1) and local coordinates (tx, ty) in [0, 1]^2.
[[nodiscard]] constexpr double bilinear(double f00, double f10, double f01, double f11, double tx,
                                        double ty) noexcept {
  const double bottom = f00 + tx * (f10 - f00);
  const double top = f01 + tx * (f11 - f01);
  return bottom + ty * (top - bottom);
}

}  // namespace roboticscore::math
