#pragma once

/// @file particle_filter.hpp
/// @brief Sequential importance resampling (SIR) particle filter.
///
/// Particles are stored column-wise in a dense matrix. Weights are kept in the log domain and
/// normalised with log-sum-exp, so likelihoods far in the tails do not underflow. The filter
/// is generic: motion and measurement models are callables, and the state space (headings,
/// ...) is described with estimation::StateSpace.
///
/// Complexity per step: O(N d) for prediction and weighting plus O(N) for resampling.

#include <cstddef>
#include <functional>

#include "roboticscore/core/types.hpp"
#include "roboticscore/estimation/models.hpp"
#include "roboticscore/estimation/resampling.hpp"
#include "roboticscore/math/random.hpp"

namespace roboticscore::estimation {

/// Outcome of a particle weight update.
struct ParticleUpdateResult {
  /// True when every particle received zero likelihood (or NaN). The update is then
  /// discarded and the previous weights are kept, so the caller can decide how to recover
  /// (e.g. inject random particles).
  bool degenerate = false;
  /// log p(z_k | z_1:k-1) estimated by the weighted mean likelihood.
  double log_evidence = 0.0;
  /// Effective sample size after the update.
  double effective_sample_size = 0.0;
};

/// Generic SIR particle filter.
class ParticleFilter {
 public:
  /// Moves one particle in place, drawing process noise from the supplied generator.
  using MotionModel = std::function<void(Eigen::Ref<VecX> particle, math::Random& rng)>;
  /// Log-likelihood log p(z | particle) of the current measurement.
  using LogLikelihood = std::function<double(const Eigen::Ref<const VecX>& particle)>;

  /// Filter over the given particles (columns) with uniform weights.
  /// @throws std::invalid_argument if there are no particles.
  explicit ParticleFilter(MatX particles, StateSpace space = {});

  /// @p count particles drawn from N(mean, covariance).
  [[nodiscard]] static ParticleFilter from_gaussian(const VecX& mean, const MatX& covariance,
                                                    std::size_t count, math::Random& rng,
                                                    StateSpace space = {});

  /// Propagates every particle through @p motion (and normalises it).
  void predict(const MotionModel& motion, math::Random& rng);

  /// Multiplies weights by exp(log_likelihood(particle)) and renormalises.
  ParticleUpdateResult update(const LogLikelihood& log_likelihood);

  /// Effective sample size 1 / sum w_i^2 in [1, N].
  [[nodiscard]] double effective_sample_size() const;

  /// Resamples N particles and resets the weights to uniform.
  void resample(ResamplingMethod method, math::Random& rng);

  /// Resamples only if ESS < threshold_ratio * N. Returns true if it resampled.
  bool resample_if_needed(double threshold_ratio, ResamplingMethod method, math::Random& rng);

  /// Weighted mean (via the state space's mean function).
  [[nodiscard]] VecX mean() const;

  /// Weighted covariance about mean() using state-space residuals.
  [[nodiscard]] MatX covariance() const;

  [[nodiscard]] const MatX& particles() const noexcept { return particles_; }
  [[nodiscard]] const VecX& log_weights() const noexcept { return log_weights_; }
  /// Normalised weights summing to one.
  [[nodiscard]] VecX weights() const;
  [[nodiscard]] std::size_t size() const noexcept {
    return static_cast<std::size_t>(particles_.cols());
  }
  [[nodiscard]] Index dimension() const noexcept { return particles_.rows(); }

 private:
  MatX particles_;
  VecX log_weights_;  // normalised: logsumexp(log_weights_) == 0
  StateSpace space_;
};

}  // namespace roboticscore::estimation
