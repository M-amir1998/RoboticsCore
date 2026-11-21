// Numerical robustness of the math utilities: extreme magnitudes, near-singular inputs and
// discontinuities. These complement the behavioural unit tests in tests/unit/math.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/linear_algebra.hpp"
#include "roboticscore/math/numerical_diff.hpp"
#include "roboticscore/math/statistics.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::math {
namespace {

TEST(AngleNumerics, LargeMagnitudesStayInRange) {
  for (const double a : {1e6, -1e6, 1e12, -1e12, 1e15}) {
    const double w = wrap_to_pi(a);
    EXPECT_GE(w, -kPi);
    EXPECT_LT(w, kPi);
    // Documented bound: error ~ |a| * 4e-17 from the floating-point value of 2*pi.
    EXPECT_NEAR(std::sin(w), std::sin(a), std::abs(a) * 1e-16 + 1e-12);
  }
}

TEST(AngleNumerics, SubnormalAndSignedZero) {
  const double tiny = std::numeric_limits<double>::denorm_min();
  EXPECT_EQ(wrap_to_pi(tiny), tiny);
  EXPECT_EQ(wrap_to_pi(-tiny), -tiny);
  EXPECT_EQ(wrap_to_pi(-0.0), 0.0);
  EXPECT_GE(wrap_to_2pi(-tiny), 0.0);
}

TEST(LinearAlgebraNumerics, NearSingularCovarianceRepair) {
  // Covariance with eigenvalues 1 and -1e-18 (round-off made it slightly indefinite).
  MatX p(2, 2);
  p << 0.5, 0.5, 0.5, 0.5 - 1e-18;
  const MatX repaired = nearest_positive_definite(p, 1e-12);
  EXPECT_TRUE(is_positive_definite(repaired));
  EXPECT_MATRIX_NEAR(repaired, p, 1e-11);
}

TEST(LinearAlgebraNumerics, IllConditionedSpdSolveIsAccurate) {
  MatX a(3, 3);
  a << 1.0, 0.0, 0.0, 0.0, 1e-8, 0.0, 0.0, 0.0, 1e8;
  const Vec3 x(1.0, -2.0, 3.0);
  const VecX b = a * x;
  EXPECT_MATRIX_NEAR(solve_spd(a, b), x, 1e-9);
  EXPECT_NEAR(condition_number(a), 1e16, 1e4);
}

TEST(StatisticsNumerics, ChiSquaredExtremeDegreesOfFreedom) {
  // Large dof: chi2 approaches N(k, 2k); the median is ~ k(1 - 2/(9k))^3.
  const double k = 1000.0;
  EXPECT_NEAR(chi_squared_quantile(0.5, k), k * std::pow(1.0 - 2.0 / (9.0 * k), 3.0), 0.05);
  // Tiny probabilities stay finite and positive.
  const double q = chi_squared_quantile(1e-10, 1.0);
  EXPECT_GT(q, 0.0);
  EXPECT_NEAR(chi_squared_cdf(q, 1.0), 1e-10, 1e-20);
}

TEST(StatisticsNumerics, GaussianLogPdfDoesNotUnderflow) {
  // A density far in the tail underflows to 0 in linear space but not in log space.
  const VecX r = VecX::Constant(3, 60.0);
  const MatX s = MatX::Identity(3, 3) * 1e-2;
  const double lp = gaussian_log_pdf(r, s);
  EXPECT_TRUE(std::isfinite(lp));
  EXPECT_LT(lp, -5e5);
}

TEST(NumericalDiffNumerics, ScalesStepForLargeArguments) {
  // With an absolute step of 1e-6, x = 1e10 would lose all significant digits.
  const auto f = [](double x) { return x * x; };
  EXPECT_NEAR(numerical_derivative(f, 1e10), 2e10, 1e2);
  EXPECT_NEAR(numerical_derivative(f, 1e-10), 2e-10, 1e-9);
}

}  // namespace
}  // namespace roboticscore::math
