#include "roboticscore/math/angles.hpp"

#include <cstddef>

#include "roboticscore/core/error.hpp"

namespace roboticscore::math {
namespace {

struct Resultant {
  double sum_sin = 0.0;
  double sum_cos = 0.0;
  double total_weight = 0.0;
};

Resultant compute_resultant(std::span<const double> angles, std::span<const double> weights) {
  detail::require(!angles.empty(), "circular statistics: angles must not be empty");
  detail::require(weights.empty() || weights.size() == angles.size(),
                  "circular statistics: weights must be empty or match angles in size");
  Resultant r;
  for (std::size_t i = 0; i < angles.size(); ++i) {
    const double w = weights.empty() ? 1.0 : weights[i];
    r.sum_sin += w * std::sin(angles[i]);
    r.sum_cos += w * std::cos(angles[i]);
    r.total_weight += w;
  }
  return r;
}

}  // namespace

double mean_resultant_length(std::span<const double> angles, std::span<const double> weights) {
  const Resultant r = compute_resultant(angles, weights);
  if (r.total_weight <= 0.0) {
    return 0.0;
  }
  return std::hypot(r.sum_sin, r.sum_cos) / r.total_weight;
}

double circular_mean(std::span<const double> angles, std::span<const double> weights) {
  const Resultant r = compute_resultant(angles, weights);
  return wrap_to_pi(std::atan2(r.sum_sin, r.sum_cos));
}

}  // namespace roboticscore::math
