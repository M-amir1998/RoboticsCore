#include "roboticscore/estimation/particle_filter.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/linear_algebra.hpp"

namespace roboticscore::estimation {
namespace {

// log(sum(exp(v))) evaluated stably; -inf if all entries are -inf.
double log_sum_exp(const VecX& v) {
  const double m = v.maxCoeff();
  if (!std::isfinite(m)) {
    return m;
  }
  return m + std::log((v.array() - m).exp().sum());
}

}  // namespace

ParticleFilter::ParticleFilter(MatX particles, StateSpace space)
    : particles_(std::move(particles)), space_(std::move(space)) {
  detail::require(particles_.cols() > 0, "ParticleFilter: at least one particle is required");
  detail::require(particles_.allFinite(), "ParticleFilter: particles must be finite");
  log_weights_ =
      VecX::Constant(particles_.cols(), -std::log(static_cast<double>(particles_.cols())));
}

ParticleFilter ParticleFilter::from_gaussian(const VecX& mean, const MatX& covariance,
                                             std::size_t count, math::Random& rng,
                                             StateSpace space) {
  detail::require(count > 0, "ParticleFilter::from_gaussian: count must be positive");
  const MatX l = math::cholesky_lower(covariance);
  MatX particles(mean.size(), static_cast<Index>(count));
  for (Index i = 0; i < particles.cols(); ++i) {
    particles.col(i) = space.canonical(mean + l * rng.standard_normal(mean.size()));
  }
  return ParticleFilter(std::move(particles), std::move(space));
}

void ParticleFilter::predict(const MotionModel& motion, math::Random& rng) {
  for (Index i = 0; i < particles_.cols(); ++i) {
    motion(particles_.col(i), rng);
    if (space_.normalize) {
      particles_.col(i) = space_.normalize(particles_.col(i));
    }
  }
}

ParticleUpdateResult ParticleFilter::update(const LogLikelihood& log_likelihood) {
  VecX updated = log_weights_;
  for (Index i = 0; i < particles_.cols(); ++i) {
    const double ll = log_likelihood(particles_.col(i));
    updated(i) = std::isnan(ll) ? -std::numeric_limits<double>::infinity() : updated(i) + ll;
  }
  ParticleUpdateResult result;
  const double evidence = log_sum_exp(updated);
  if (!std::isfinite(evidence)) {
    result.degenerate = true;
    result.log_evidence = -std::numeric_limits<double>::infinity();
    result.effective_sample_size = effective_sample_size();
    return result;
  }
  log_weights_ = updated.array() - evidence;
  result.log_evidence = evidence;
  result.effective_sample_size = effective_sample_size();
  return result;
}

VecX ParticleFilter::weights() const {
  return log_weights_.array().exp();
}

double ParticleFilter::effective_sample_size() const {
  return 1.0 / (2.0 * log_weights_.array()).exp().sum();
}

void ParticleFilter::resample(ResamplingMethod method, math::Random& rng) {
  const VecX w = weights();
  const auto ancestors = estimation::resample(
      std::span<const double>(w.data(), static_cast<std::size_t>(w.size())), size(), method, rng);
  MatX next(particles_.rows(), particles_.cols());
  for (std::size_t j = 0; j < ancestors.size(); ++j) {
    next.col(static_cast<Index>(j)) = particles_.col(static_cast<Index>(ancestors[j]));
  }
  particles_ = std::move(next);
  log_weights_.setConstant(-std::log(static_cast<double>(particles_.cols())));
}

bool ParticleFilter::resample_if_needed(double threshold_ratio, ResamplingMethod method,
                                        math::Random& rng) {
  detail::require(threshold_ratio >= 0.0 && threshold_ratio <= 1.0,
                  "ParticleFilter::resample_if_needed: threshold must lie in [0, 1]");
  if (effective_sample_size() < threshold_ratio * static_cast<double>(size())) {
    resample(method, rng);
    return true;
  }
  return false;
}

VecX ParticleFilter::mean() const {
  return space_.canonical(space_.average(particles_, weights()));
}

MatX ParticleFilter::covariance() const {
  const VecX mu = mean();
  const VecX w = weights();
  MatX cov = MatX::Zero(particles_.rows(), particles_.rows());
  for (Index i = 0; i < particles_.cols(); ++i) {
    const VecX d = space_.diff(particles_.col(i), mu);
    cov += w(i) * d * d.transpose();
  }
  return math::symmetrize(cov);
}

}  // namespace roboticscore::estimation
