#pragma once

/// @file random.hpp
/// @brief Explicitly seeded, platform-independent random number generation.
///
/// The C++ standard fully specifies the output of std::mt19937_64 but NOT the algorithms of
/// std::uniform_real_distribution or std::normal_distribution, which differ between libstdc++,
/// libc++ and the MSVC STL. To make simulations and regression tests reproducible across
/// compilers, Random derives every variate from the raw engine output with algorithms
/// implemented here. There is no hidden global generator: every stochastic API takes a
/// Random& explicitly.

#include <cstddef>
#include <cstdint>
#include <random>

#include "roboticscore/core/types.hpp"

namespace roboticscore::math {

/// Deterministic pseudo-random generator with portable distributions.
class Random {
 public:
  /// Seeds the underlying Mersenne Twister. The same seed always yields the same stream.
  explicit Random(std::uint64_t seed);

  /// Raw 64-bit engine output.
  [[nodiscard]] std::uint64_t next_u64();

  /// Uniform double in [0, 1) with 53 random bits.
  [[nodiscard]] double uniform01();

  /// Uniform double in [lo, hi).
  /// @throws std::invalid_argument if hi < lo.
  [[nodiscard]] double uniform(double lo, double hi);

  /// Unbiased uniform integer in [0, n) (rejection sampling).
  /// @throws std::invalid_argument if n == 0.
  [[nodiscard]] std::size_t uniform_index(std::size_t n);

  /// Normal variate via the Box-Muller transform.
  /// @throws std::invalid_argument if stddev < 0.
  [[nodiscard]] double normal(double mean = 0.0, double stddev = 1.0);

  /// Vector of independent standard normal variates.
  [[nodiscard]] VecX standard_normal(Index dimension);

  /// Sample from N(mean, covariance) using a Cholesky factor of the covariance.
  /// @throws std::invalid_argument on dimension mismatch.
  /// @throws NumericalError if the covariance is not positive semidefinite.
  [[nodiscard]] VecX multivariate_normal(const VecX& mean, const MatX& covariance);

  /// Returns an independent generator derived deterministically from this generator's seed
  /// and @p stream. Useful to give each simulated sensor its own reproducible noise stream.
  [[nodiscard]] Random derive(std::uint64_t stream) const;

  /// The seed this generator was constructed with.
  [[nodiscard]] std::uint64_t seed() const noexcept { return seed_; }

 private:
  std::uint64_t seed_;
  std::mt19937_64 engine_;
  bool has_spare_normal_ = false;
  double spare_normal_ = 0.0;
};

/// SplitMix64 finaliser: a bijective 64-bit mixing function used to derive seeds.
[[nodiscard]] constexpr std::uint64_t splitmix64(std::uint64_t x) noexcept {
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30U)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27U)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31U);
}

}  // namespace roboticscore::math
