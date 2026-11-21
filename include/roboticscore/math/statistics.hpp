#pragma once

/// @file statistics.hpp
/// @brief Sample statistics, Gaussian densities and chi-square distribution functions used
///        by estimators (NIS/NEES consistency checks) and benchmarks (RMSE).

#include <span>

#include "roboticscore/core/types.hpp"

namespace roboticscore::math {

/// Arithmetic mean. @throws std::invalid_argument for an empty input.
[[nodiscard]] double mean(std::span<const double> values);

/// Unbiased sample variance (divides by n - 1). @throws std::invalid_argument if n < 2.
[[nodiscard]] double sample_variance(std::span<const double> values);

/// Root-mean-square of a set of errors, sqrt(sum e_i^2 / n).
/// @throws std::invalid_argument for an empty input.
[[nodiscard]] double rms(std::span<const double> errors);

/// Mean of the columns of @p samples (dimension x count).
/// @throws std::invalid_argument if there are no samples.
[[nodiscard]] VecX sample_mean(const MatX& samples);

/// Unbiased sample covariance of the columns of @p samples (divides by n - 1).
/// @throws std::invalid_argument if fewer than two samples are given.
[[nodiscard]] MatX sample_covariance(const MatX& samples);

/// Weighted mean of the columns of @p samples; weights need not be normalised.
/// @throws std::invalid_argument if sizes mismatch or the weights do not sum to a positive value.
[[nodiscard]] VecX weighted_mean(const MatX& samples, const VecX& weights);

/// Weighted covariance sum_i w_i (x_i - mu)(x_i - mu)^T / sum_i w_i about @p mean.
/// @throws std::invalid_argument if sizes mismatch or the weights do not sum to a positive value.
[[nodiscard]] MatX weighted_covariance(const MatX& samples, const VecX& weights, const VecX& mean);

/// Squared Mahalanobis distance r^T S^{-1} r, evaluated with a Cholesky factorisation.
/// @throws std::invalid_argument on dimension mismatch.
/// @throws NumericalError if @p covariance is not positive definite.
[[nodiscard]] double mahalanobis_squared(const VecX& residual, const MatX& covariance);

/// Log-density of N(residual; 0, covariance).
/// @throws NumericalError if @p covariance is not positive definite.
[[nodiscard]] double gaussian_log_pdf(const VecX& residual, const MatX& covariance);

/// Regularised lower incomplete gamma function P(a, x), a > 0, x >= 0.
/// Uses the power series for x < a + 1 and a continued fraction (modified Lentz) otherwise.
/// @throws std::invalid_argument for a <= 0 or x < 0.
[[nodiscard]] double regularized_gamma_p(double a, double x);

/// Chi-square cumulative distribution function with @p dof degrees of freedom.
/// @throws std::invalid_argument for dof <= 0.
[[nodiscard]] double chi_squared_cdf(double x, double dof);

/// Chi-square quantile (inverse CDF) for probability p in (0, 1).
/// Solved by bracketing and bisection on chi_squared_cdf (relative bracket width 1e-15).
/// @throws std::invalid_argument for p outside (0, 1) or dof <= 0.
[[nodiscard]] double chi_squared_quantile(double p, double dof);

}  // namespace roboticscore::math
