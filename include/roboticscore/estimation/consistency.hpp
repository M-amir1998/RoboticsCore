#pragma once

/// @file consistency.hpp
/// @brief Filter consistency statistics (Bar-Shalom, Li & Kirubarajan, "Estimation with
///        Applications to Tracking and Navigation", 2001, ch. 5.4).
///
/// For a consistent filter the normalised estimation error squared
///   NEES_k = e_k^T P_k^{-1} e_k,  e_k = x_true - x_hat
/// is chi-square distributed with n degrees of freedom. Averaged over N independent Monte
/// Carlo runs (or K time steps for the time-averaged NIS), N * mean(NEES) ~ chi2(N n), which
/// gives the two-sided acceptance interval returned by average_chi2_bounds().

#include "roboticscore/core/types.hpp"

namespace roboticscore::estimation {

/// Normalised estimation error squared e^T P^{-1} e.
/// @throws NumericalError if P is not positive definite.
[[nodiscard]] double nees(const VecX& error, const MatX& covariance);

/// Two-sided acceptance interval for the average of @p samples independent chi-square
/// statistics with @p dof degrees of freedom each.
struct Chi2Interval {
  double lower = 0.0;
  double upper = 0.0;
  [[nodiscard]] bool contains(double value) const { return value >= lower && value <= upper; }
};

/// @throws std::invalid_argument for non-positive dof/samples or confidence outside (0, 1).
[[nodiscard]] Chi2Interval average_chi2_bounds(int dof, int samples, double confidence = 0.95);

}  // namespace roboticscore::estimation
