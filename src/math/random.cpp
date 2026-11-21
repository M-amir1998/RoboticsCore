#include "roboticscore/math/random.hpp"

#include <cmath>
#include <limits>
#include <numbers>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/linear_algebra.hpp"

namespace roboticscore::math {

Random::Random(std::uint64_t seed) : seed_(seed), engine_(seed) {}

std::uint64_t Random::next_u64() {
  return engine_();
}

double Random::uniform01() {
  // Top 53 bits scaled by 2^-53 gives every representable multiple of 2^-53 in [0, 1).
  constexpr double kScale = 1.0 / 9007199254740992.0;  // 2^-53
  return static_cast<double>(next_u64() >> 11U) * kScale;
}

double Random::uniform(double lo, double hi) {
  detail::require(hi >= lo, "Random::uniform: hi must be >= lo");
  return lo + (hi - lo) * uniform01();
}

std::size_t Random::uniform_index(std::size_t n) {
  detail::require(n > 0, "Random::uniform_index: n must be positive");
  const auto range = static_cast<std::uint64_t>(n);
  // Reject the incomplete final block so every residue is equally likely.
  const std::uint64_t limit = std::numeric_limits<std::uint64_t>::max()
                              - (std::numeric_limits<std::uint64_t>::max() % range);
  std::uint64_t r = next_u64();
  while (r >= limit) {
    r = next_u64();
  }
  return static_cast<std::size_t>(r % range);
}

double Random::normal(double mean, double stddev) {
  detail::require(stddev >= 0.0, "Random::normal: stddev must be non-negative");
  if (has_spare_normal_) {
    has_spare_normal_ = false;
    return mean + stddev * spare_normal_;
  }
  // u1 in (0, 1] so the logarithm is finite.
  const double u1 = 1.0 - uniform01();
  const double u2 = uniform01();
  const double radius = std::sqrt(-2.0 * std::log(u1));
  const double angle = 2.0 * std::numbers::pi * u2;
  spare_normal_ = radius * std::sin(angle);
  has_spare_normal_ = true;
  return mean + stddev * radius * std::cos(angle);
}

VecX Random::standard_normal(Index dimension) {
  detail::require(dimension >= 0, "Random::standard_normal: dimension must be non-negative");
  VecX z(dimension);
  for (Index i = 0; i < dimension; ++i) {
    z(i) = normal();
  }
  return z;
}

VecX Random::multivariate_normal(const VecX& mean, const MatX& covariance) {
  detail::require(covariance.rows() == mean.size() && covariance.cols() == mean.size(),
                  "Random::multivariate_normal: covariance dimension mismatch");
  const MatX l = cholesky_lower(covariance);
  return mean + l * standard_normal(mean.size());
}

Random Random::derive(std::uint64_t stream) const {
  return Random(splitmix64(seed_ ^ splitmix64(stream)));
}

}  // namespace roboticscore::math
