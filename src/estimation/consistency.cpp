#include "roboticscore/estimation/consistency.hpp"

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/statistics.hpp"

namespace roboticscore::estimation {

double nees(const VecX& error, const MatX& covariance) {
  return math::mahalanobis_squared(error, covariance);
}

Chi2Interval average_chi2_bounds(int dof, int samples, double confidence) {
  detail::require(dof > 0 && samples > 0, "average_chi2_bounds: dof and samples must be positive");
  detail::require(confidence > 0.0 && confidence < 1.0,
                  "average_chi2_bounds: confidence must lie in (0, 1)");
  const double total_dof = static_cast<double>(dof) * static_cast<double>(samples);
  const double tail = 0.5 * (1.0 - confidence);
  return {math::chi_squared_quantile(tail, total_dof) / samples,
          math::chi_squared_quantile(1.0 - tail, total_dof) / samples};
}

}  // namespace roboticscore::estimation
