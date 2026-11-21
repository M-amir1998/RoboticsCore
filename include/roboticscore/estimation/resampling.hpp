#pragma once

/// @file resampling.hpp
/// @brief Importance resampling schemes for sequential Monte Carlo (Douc, Cappe & Moulines,
///        "Comparison of Resampling Schemes for Particle Filtering", ISPA 2005).
///
/// All schemes are unbiased: the expected number of copies of particle i is N w_i.
///   multinomial  N independent draws                         O(N log N)
///   stratified   one uniform draw per stratum [j/N, (j+1)/N)  O(N)
///   systematic   one shared offset u0 + j/N                   O(N), copies in {floor, ceil}(N w_i)
///   residual     floor(N w_i) deterministic copies, remainder multinomial

#include <cstddef>
#include <span>
#include <vector>

#include "roboticscore/math/random.hpp"

namespace roboticscore::estimation {

enum class ResamplingMethod {
  kMultinomial,
  kStratified,
  kSystematic,
  kResidual,
};

/// Draws @p count ancestor indices according to @p weights (non-negative, not necessarily
/// normalised). Indices are returned in non-decreasing order.
/// @throws std::invalid_argument if weights are empty, negative, non-finite or sum to zero.
[[nodiscard]] std::vector<std::size_t> resample(std::span<const double> weights, std::size_t count,
                                                ResamplingMethod method, math::Random& rng);

/// Effective sample size 1 / sum(w_i^2) of normalised weights (Kong, Liu & Wong, 1994).
/// @throws std::invalid_argument for invalid weights (see resample()).
[[nodiscard]] double effective_sample_size(std::span<const double> weights);

}  // namespace roboticscore::estimation
