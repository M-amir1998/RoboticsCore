#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/integration.hpp"
#include "roboticscore/math/interpolation.hpp"
#include "roboticscore/math/linear_algebra.hpp"
#include "roboticscore/math/numerical_diff.hpp"
#include "roboticscore/math/statistics.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::math {
namespace {

// --- interpolation ----------------------------------------------------------------------

TEST(Interp1, InteriorAndClamp) {
  const std::vector<double> xs = {0.0, 1.0, 3.0};
  const std::vector<double> ys = {0.0, 2.0, 6.0};
  EXPECT_DOUBLE_EQ(interp1(xs, ys, 0.5), 1.0);
  EXPECT_DOUBLE_EQ(interp1(xs, ys, 2.0), 4.0);
  EXPECT_DOUBLE_EQ(interp1(xs, ys, 1.0), 2.0);
  EXPECT_DOUBLE_EQ(interp1(xs, ys, -1.0), 0.0);
  EXPECT_DOUBLE_EQ(interp1(xs, ys, 10.0), 6.0);
}

TEST(Interp1, LinearExtrapolation) {
  const std::vector<double> xs = {0.0, 1.0};
  const std::vector<double> ys = {1.0, 3.0};
  EXPECT_DOUBLE_EQ(interp1(xs, ys, 2.0, Extrapolation::kLinear), 5.0);
  EXPECT_DOUBLE_EQ(interp1(xs, ys, -1.0, Extrapolation::kLinear), -1.0);
}

TEST(Interp1, RejectsMalformedTables) {
  const std::vector<double> one = {0.0};
  const std::vector<double> xs = {0.0, 0.0};
  const std::vector<double> ys = {1.0, 2.0};
  const std::vector<double> short_ys = {1.0};
  EXPECT_THROW((void)interp1(one, one, 0.0), std::invalid_argument);
  EXPECT_THROW((void)interp1(xs, ys, 0.0), std::invalid_argument);
  EXPECT_THROW((void)interp1(ys, short_ys, 0.0), std::invalid_argument);
}

TEST(Bilinear, CornersAndCenter) {
  EXPECT_DOUBLE_EQ(bilinear(1, 2, 3, 4, 0, 0), 1.0);
  EXPECT_DOUBLE_EQ(bilinear(1, 2, 3, 4, 1, 1), 4.0);
  EXPECT_DOUBLE_EQ(bilinear(1, 2, 3, 4, 0.5, 0.5), 2.5);
}

// --- numerical differentiation -----------------------------------------------------------

TEST(NumericalDerivative, MatchesAnalytic) {
  const auto f = [](double x) {
    return std::sin(x) * std::exp(0.3 * x);
  };
  const auto df = [](double x) {
    return std::cos(x) * std::exp(0.3 * x) + 0.3 * std::sin(x) * std::exp(0.3 * x);
  };
  for (const double x : {-2.0, 0.0, 0.7, 15.0}) {
    EXPECT_NEAR(numerical_derivative(f, x), df(x), 1e-8 * std::max(1.0, std::abs(df(x))));
  }
}

TEST(NumericalJacobian, MatchesAnalyticRangeBearing) {
  // h(x) = [range, bearing] to the origin from position x.
  const auto h = [](const VecX& x) {
    VecX z(2);
    z << x.norm(), std::atan2(-x(1), -x(0));
    return z;
  };
  VecX x(2);
  x << 3.0, -4.0;
  MatX expected(2, 2);
  const double r2 = x.squaredNorm();
  const double r = std::sqrt(r2);
  // d/dx atan2(-y, -x) = -y / r^2, d/dy atan2(-y, -x) = x / r^2.
  expected << x(0) / r, x(1) / r, -x(1) / r2, x(0) / r2;
  EXPECT_MATRIX_NEAR(numerical_jacobian(h, x), expected, 1e-8);
}

TEST(NumericalJacobian, CustomDifferenceHandlesAngleWrap) {
  // Output is an angle sitting exactly on the +-pi discontinuity.
  const auto f = [](const VecX& x) {
    VecX z(1);
    z << std::atan2(x(1), x(0));
    return z;
  };
  VecX x(2);
  x << -1.0, 0.0;
  const auto wrap_diff = [](const VecX& a, const VecX& b) {
    VecX d = a - b;
    d(0) = std::remainder(d(0), 2.0 * std::acos(-1.0));
    return d;
  };
  const MatX j = numerical_jacobian(f, x, wrap_diff);
  EXPECT_NEAR(j(0, 0), 0.0, 1e-8);
  EXPECT_NEAR(j(0, 1), -1.0, 1e-8);
}

// --- integration -------------------------------------------------------------------------

TEST(Integrators, Rk4IsFourthOrder) {
  // dx/dt = -x, x(0) = 1 -> x(1) = e^-1.
  const auto f = [](double, double x) {
    return -x;
  };
  const auto integrate = [&](int steps) {
    double x = 1.0;
    const double dt = 1.0 / steps;
    for (int i = 0; i < steps; ++i) {
      x = rk4_step(f, x, i * dt, dt);
    }
    return std::abs(x - std::exp(-1.0));
  };
  const double e1 = integrate(10);
  const double e2 = integrate(20);
  EXPECT_NEAR(std::log2(e1 / e2), 4.0, 0.1);
}

TEST(Integrators, EulerIsFirstOrder) {
  const auto f = [](double, double x) {
    return -x;
  };
  const auto integrate = [&](int steps) {
    double x = 1.0;
    const double dt = 1.0 / steps;
    for (int i = 0; i < steps; ++i) {
      x = euler_step(f, x, i * dt, dt);
    }
    return std::abs(x - std::exp(-1.0));
  };
  EXPECT_NEAR(std::log2(integrate(100) / integrate(200)), 1.0, 0.05);
}

TEST(Integrators, Rk4WorksWithEigenState) {
  // Harmonic oscillator: energy approximately conserved over one period.
  const auto f = [](double, const Vec2& s) {
    return Vec2(s(1), -s(0));
  };
  Vec2 s(1.0, 0.0);
  const int steps = 200;
  const double dt = 2.0 * std::acos(-1.0) / steps;
  for (int i = 0; i < steps; ++i) {
    s = rk4_step(f, s, i * dt, dt);
  }
  EXPECT_MATRIX_NEAR(s, Vec2(1.0, 0.0), 1e-7);
}

TEST(Quadrature, TrapezoidAndSimpson) {
  const std::vector<double> xs = {0.0, 0.5, 2.0};
  const std::vector<double> ys = {0.0, 1.0, 4.0};  // y = 2x
  EXPECT_DOUBLE_EQ(trapezoid(xs, ys), 4.0);
  // Simpson is exact for cubics.
  EXPECT_NEAR(simpson([](double x) { return x * x * x - x; }, 0.0, 2.0, 2), 2.0, 1e-14);
  EXPECT_THROW((void)simpson([](double x) { return x; }, 0.0, 1.0, 3), std::invalid_argument);
}

// --- statistics --------------------------------------------------------------------------

TEST(Statistics, ScalarMoments) {
  const std::vector<double> v = {1.0, 2.0, 3.0, 4.0};
  EXPECT_DOUBLE_EQ(mean(v), 2.5);
  EXPECT_DOUBLE_EQ(sample_variance(v), 5.0 / 3.0);
  const std::vector<double> e = {3.0, -4.0};
  EXPECT_DOUBLE_EQ(rms(e), std::sqrt(12.5));
}

TEST(Statistics, SampleAndWeightedCovariance) {
  MatX samples(2, 4);
  samples << 1, 2, 3, 4,  //
      2, 4, 6, 8;
  const VecX mu = sample_mean(samples);
  EXPECT_MATRIX_NEAR(mu, Vec2(2.5, 5.0), 1e-15);
  MatX expected(2, 2);
  expected << 5.0 / 3.0, 10.0 / 3.0, 10.0 / 3.0, 20.0 / 3.0;
  EXPECT_MATRIX_NEAR(sample_covariance(samples), expected, 1e-14);

  const VecX w = VecX::Constant(4, 0.25);
  EXPECT_MATRIX_NEAR(weighted_mean(samples, w), mu, 1e-15);
  EXPECT_MATRIX_NEAR(weighted_covariance(samples, w, mu), expected * 3.0 / 4.0, 1e-14);
}

TEST(Statistics, MahalanobisAndGaussianLogPdf) {
  Mat2 s;
  s << 4.0, 0.0, 0.0, 1.0;
  const Vec2 r(2.0, 1.0);
  EXPECT_NEAR(mahalanobis_squared(r, s), 2.0, 1e-14);
  const double expected = -0.5 * (2.0 * std::log(2.0 * std::acos(-1.0)) + std::log(4.0) + 2.0);
  EXPECT_NEAR(gaussian_log_pdf(r, s), expected, 1e-13);
  Mat2 indefinite;
  indefinite << 1.0, 2.0, 2.0, 1.0;
  EXPECT_THROW((void)mahalanobis_squared(r, indefinite), NumericalError);
}

TEST(Statistics, ChiSquaredMatchesTables) {
  // Reference values from standard chi-square tables (upper 5% points).
  EXPECT_NEAR(chi_squared_quantile(0.95, 1), 3.841459, 1e-6);
  EXPECT_NEAR(chi_squared_quantile(0.95, 2), 5.991465, 1e-6);
  EXPECT_NEAR(chi_squared_quantile(0.95, 3), 7.814728, 1e-6);
  EXPECT_NEAR(chi_squared_quantile(0.99, 10), 23.209251, 1e-5);
  EXPECT_NEAR(chi_squared_quantile(0.025, 100), 74.221927, 1e-5);
  // For dof = 2, CDF is 1 - exp(-x/2).
  EXPECT_NEAR(chi_squared_cdf(3.0, 2), 1.0 - std::exp(-1.5), 1e-14);
  for (const double p : {0.01, 0.5, 0.975}) {
    EXPECT_NEAR(chi_squared_cdf(chi_squared_quantile(p, 7), 7), p, 1e-12);
  }
}

// --- linear algebra ----------------------------------------------------------------------

TEST(LinearAlgebra, SymmetryAndDefiniteness) {
  MatX m(2, 2);
  m << 2.0, 1.0, 1.0 + 1e-15, 2.0;
  EXPECT_TRUE(is_symmetric(m));
  EXPECT_TRUE(is_positive_definite(m));
  MatX indefinite(2, 2);
  indefinite << 1.0, 2.0, 2.0, 1.0;
  EXPECT_FALSE(is_positive_definite(indefinite));
  const MatX fixed = nearest_positive_definite(indefinite, 1e-6);
  EXPECT_TRUE(is_positive_definite(fixed));
  EXPECT_MATRIX_NEAR(symmetrize(fixed), fixed, 1e-14);
}

TEST(LinearAlgebra, CholeskyWithJitterOnSemidefinite) {
  // Rank-1 PSD matrix: plain LLT fails, jitter makes it factorisable.
  const Vec2 v(1.0, 2.0);
  const MatX psd = v * v.transpose();
  const MatX l = cholesky_lower(psd);
  EXPECT_MATRIX_NEAR(l * l.transpose(), psd, 1e-6);
  MatX negative = -MatX::Identity(2, 2);
  EXPECT_THROW((void)cholesky_lower(negative), NumericalError);
}

TEST(LinearAlgebra, ConditionNumberAndSolve) {
  MatX a(2, 2);
  a << 4.0, 0.0, 0.0, 1e-3;
  EXPECT_NEAR(condition_number(a), 4000.0, 1e-9);
  const VecX b = Vec2(8.0, 1e-3);
  EXPECT_MATRIX_NEAR(solve_spd(a, b), Vec2(2.0, 1.0), 1e-12);
  const MatX singular = MatX::Zero(2, 2);
  EXPECT_TRUE(std::isinf(condition_number(singular)));
  EXPECT_THROW((void)solve_spd(singular, b), NumericalError);
}

}  // namespace
}  // namespace roboticscore::math
