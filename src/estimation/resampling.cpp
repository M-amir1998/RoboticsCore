#include "roboticscore/estimation/resampling.hpp"

#include <algorithm>
#include <cmath>

#include "roboticscore/core/error.hpp"

namespace roboticscore::estimation {
namespace {

// Validates weights and returns them normalised to sum to one.
std::vector<double> normalized(std::span<const double> weights) {
  detail::require(!weights.empty(), "resample: weights must not be empty");
  double total = 0.0;
  for (const double w : weights) {
    detail::require(std::isfinite(w) && w >= 0.0,
                    "resample: weights must be finite and non-negative");
    total += w;
  }
  detail::require(total > 0.0, "resample: weights must not all be zero");
  std::vector<double> out(weights.begin(), weights.end());
  for (double& w : out) {
    w /= total;
  }
  return out;
}

std::vector<double> cumulative(const std::vector<double>& w) {
  std::vector<double> cdf(w.size());
  double acc = 0.0;
  for (std::size_t i = 0; i < w.size(); ++i) {
    acc += w[i];
    cdf[i] = acc;
  }
  cdf.back() = 1.0;  // guard against round-off so every position in [0, 1) is covered
  return cdf;
}

// Maps sorted positions in [0, 1) to ancestor indices with one linear sweep.
std::vector<std::size_t> sweep(const std::vector<double>& cdf, const std::vector<double>& u) {
  std::vector<std::size_t> out(u.size());
  std::size_t i = 0;
  for (std::size_t j = 0; j < u.size(); ++j) {
    while (u[j] >= cdf[i] && i + 1 < cdf.size()) {
      ++i;
    }
    out[j] = i;
  }
  return out;
}

std::vector<std::size_t> multinomial(const std::vector<double>& w, std::size_t count,
                                     math::Random& rng) {
  std::vector<double> u(count);
  for (double& x : u) {
    x = rng.uniform01();
  }
  std::sort(u.begin(), u.end());
  return sweep(cumulative(w), u);
}

std::vector<std::size_t> evenly_spaced(const std::vector<double>& w, std::size_t count,
                                       bool shared_offset, math::Random& rng) {
  const auto n = static_cast<double>(count);
  const double offset = shared_offset ? rng.uniform01() : 0.0;
  std::vector<double> u(count);
  for (std::size_t j = 0; j < count; ++j) {
    const double jitter = shared_offset ? offset : rng.uniform01();
    u[j] = (static_cast<double>(j) + jitter) / n;
  }
  return sweep(cumulative(w), u);
}

std::vector<std::size_t> residual_resampling(const std::vector<double>& w, std::size_t count,
                                             math::Random& rng) {
  const auto n = static_cast<double>(count);
  std::vector<std::size_t> out;
  out.reserve(count);
  std::vector<double> remainder(w.size());
  double remainder_mass = 0.0;
  for (std::size_t i = 0; i < w.size(); ++i) {
    const double expected = w[i] * n;
    const double copies = std::floor(expected);
    out.insert(out.end(), static_cast<std::size_t>(copies), i);
    remainder[i] = expected - copies;
    remainder_mass += remainder[i];
  }
  if (out.size() < count && remainder_mass > 0.0) {
    for (double& r : remainder) {
      r /= remainder_mass;
    }
    const auto extra = multinomial(remainder, count - out.size(), rng);
    out.insert(out.end(), extra.begin(), extra.end());
  }
  // Round-off can leave the count one short (or long); pad with the heaviest particle.
  const auto heaviest = static_cast<std::size_t>(std::max_element(w.begin(), w.end()) - w.begin());
  out.resize(count, heaviest);
  std::sort(out.begin(), out.end());
  return out;
}

}  // namespace

std::vector<std::size_t> resample(std::span<const double> weights, std::size_t count,
                                  ResamplingMethod method, math::Random& rng) {
  const std::vector<double> w = normalized(weights);
  if (count == 0) {
    return {};
  }
  switch (method) {
    case ResamplingMethod::kMultinomial:
      return multinomial(w, count, rng);
    case ResamplingMethod::kStratified:
      return evenly_spaced(w, count, /*shared_offset=*/false, rng);
    case ResamplingMethod::kSystematic:
      return evenly_spaced(w, count, /*shared_offset=*/true, rng);
    case ResamplingMethod::kResidual:
      return residual_resampling(w, count, rng);
  }
  return {};
}

double effective_sample_size(std::span<const double> weights) {
  double sum_sq = 0.0;
  for (const double w : normalized(weights)) {
    sum_sq += w * w;
  }
  return 1.0 / sum_sq;
}

}  // namespace roboticscore::estimation
