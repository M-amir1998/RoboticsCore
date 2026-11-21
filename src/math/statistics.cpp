#include "roboticscore/math/statistics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>

#include <Eigen/Cholesky>

#include "roboticscore/core/error.hpp"

namespace roboticscore::math {
namespace {

constexpr double kTiny = 1e-300;
constexpr double kEps = std::numeric_limits<double>::epsilon();
constexpr int kMaxSeriesIterations = 10000;

Eigen::LLT<MatX> factorize_spd(const MatX& covariance, const char* who) {
  Eigen::LLT<MatX> llt(covariance);
  if (llt.info() != Eigen::Success) {
    throw NumericalError(std::string(who) + ": covariance is not positive definite");
  }
  return llt;
}

// log(x^a e^-x / Gamma(a)), the common prefactor of P(a, x) and Q(a, x).
double gamma_prefactor_log(double a, double x) {
  return -x + a * std::log(x) - std::lgamma(a);
}

double gamma_p_series(double a, double x) {
  double term = 1.0 / a;
  double sum = term;
  for (int n = 1; n < kMaxSeriesIterations; ++n) {
    term *= x / (a + n);
    sum += term;
    if (std::abs(term) < std::abs(sum) * kEps) {
      break;
    }
  }
  return sum * std::exp(gamma_prefactor_log(a, x));
}

// Q(a, x) via the continued fraction evaluated with the modified Lentz method.
double gamma_q_continued_fraction(double a, double x) {
  double b = x + 1.0 - a;
  double c = 1.0 / kTiny;
  double d = 1.0 / b;
  double h = d;
  for (int i = 1; i < kMaxSeriesIterations; ++i) {
    const double an = -i * (i - a);
    b += 2.0;
    d = an * d + b;
    if (std::abs(d) < kTiny) {
      d = kTiny;
    }
    c = b + an / c;
    if (std::abs(c) < kTiny) {
      c = kTiny;
    }
    d = 1.0 / d;
    const double delta = d * c;
    h *= delta;
    if (std::abs(delta - 1.0) < kEps) {
      break;
    }
  }
  return std::exp(gamma_prefactor_log(a, x)) * h;
}

}  // namespace

double mean(std::span<const double> values) {
  detail::require(!values.empty(), "mean: input must not be empty");
  double sum = 0.0;
  for (const double v : values) {
    sum += v;
  }
  return sum / static_cast<double>(values.size());
}

double sample_variance(std::span<const double> values) {
  detail::require(values.size() >= 2, "sample_variance: at least two values are required");
  const double mu = mean(values);
  double ss = 0.0;
  for (const double v : values) {
    ss += (v - mu) * (v - mu);
  }
  return ss / static_cast<double>(values.size() - 1);
}

double rms(std::span<const double> errors) {
  detail::require(!errors.empty(), "rms: input must not be empty");
  double ss = 0.0;
  for (const double e : errors) {
    ss += e * e;
  }
  return std::sqrt(ss / static_cast<double>(errors.size()));
}

VecX sample_mean(const MatX& samples) {
  detail::require(samples.cols() > 0, "sample_mean: at least one sample is required");
  return samples.rowwise().mean();
}

MatX sample_covariance(const MatX& samples) {
  detail::require(samples.cols() >= 2, "sample_covariance: at least two samples are required");
  const MatX centered = samples.colwise() - samples.rowwise().mean();
  return (centered * centered.transpose()) / static_cast<double>(samples.cols() - 1);
}

VecX weighted_mean(const MatX& samples, const VecX& weights) {
  detail::require(samples.cols() == weights.size(),
                  "weighted_mean: one weight per sample is required");
  const double total = weights.sum();
  detail::require(total > 0.0, "weighted_mean: weights must sum to a positive value");
  return (samples * weights) / total;
}

MatX weighted_covariance(const MatX& samples, const VecX& weights, const VecX& mean) {
  detail::require(samples.cols() == weights.size(),
                  "weighted_covariance: one weight per sample is required");
  detail::require(samples.rows() == mean.size(), "weighted_covariance: mean dimension mismatch");
  const double total = weights.sum();
  detail::require(total > 0.0, "weighted_covariance: weights must sum to a positive value");
  const MatX centered = samples.colwise() - mean;
  return (centered * weights.asDiagonal() * centered.transpose()) / total;
}

double mahalanobis_squared(const VecX& residual, const MatX& covariance) {
  detail::require(covariance.rows() == residual.size() && covariance.cols() == residual.size(),
                  "mahalanobis_squared: dimension mismatch");
  const auto llt = factorize_spd(covariance, "mahalanobis_squared");
  const VecX whitened = llt.matrixL().solve(residual);
  return whitened.squaredNorm();
}

double gaussian_log_pdf(const VecX& residual, const MatX& covariance) {
  detail::require(covariance.rows() == residual.size() && covariance.cols() == residual.size(),
                  "gaussian_log_pdf: dimension mismatch");
  const auto llt = factorize_spd(covariance, "gaussian_log_pdf");
  const VecX whitened = llt.matrixL().solve(residual);
  const double log_det = 2.0 * llt.matrixL().toDenseMatrix().diagonal().array().log().sum();
  const auto d = static_cast<double>(residual.size());
  return -0.5 * (d * std::log(2.0 * std::numbers::pi) + log_det + whitened.squaredNorm());
}

double regularized_gamma_p(double a, double x) {
  detail::require(a > 0.0, "regularized_gamma_p: a must be positive");
  detail::require(x >= 0.0, "regularized_gamma_p: x must be non-negative");
  if (x == 0.0) {
    return 0.0;
  }
  if (x < a + 1.0) {
    return gamma_p_series(a, x);
  }
  return 1.0 - gamma_q_continued_fraction(a, x);
}

double chi_squared_cdf(double x, double dof) {
  detail::require(dof > 0.0, "chi_squared_cdf: degrees of freedom must be positive");
  if (x <= 0.0) {
    return 0.0;
  }
  return regularized_gamma_p(0.5 * dof, 0.5 * x);
}

double chi_squared_quantile(double p, double dof) {
  detail::require(p > 0.0 && p < 1.0, "chi_squared_quantile: p must lie in (0, 1)");
  detail::require(dof > 0.0, "chi_squared_quantile: degrees of freedom must be positive");
  double lo = 0.0;
  double hi = std::max(1.0, dof);
  while (chi_squared_cdf(hi, dof) < p) {
    lo = hi;
    hi *= 2.0;
  }
  // The CDF is monotone, so bisection is guaranteed to converge. The stopping rule is relative
  // because quantiles for tiny p can be extremely small (p = 1e-10, dof = 1 gives ~1.6e-20).
  for (int i = 0; i < 2000 && (hi - lo) > 1e-15 * hi; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (chi_squared_cdf(mid, dof) < p) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return 0.5 * (lo + hi);
}

}  // namespace roboticscore::math
