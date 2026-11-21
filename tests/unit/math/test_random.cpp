#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/math/random.hpp"
#include "roboticscore/math/statistics.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::math {
namespace {

TEST(Random, EngineMatchesStandardReferenceValue) {
  // [rand.predef]: the 10000th invocation of a default-constructed mt19937_64 produces
  // 9981545732273789042. Random seeds the same engine type, so it is portable by construction.
  std::mt19937_64 reference;
  for (int i = 0; i < 9999; ++i) {
    (void)reference();
  }
  EXPECT_EQ(reference(), 9981545732273789042ULL);
  Random rng(std::mt19937_64::default_seed);
  for (int i = 0; i < 9999; ++i) {
    (void)rng.next_u64();
  }
  EXPECT_EQ(rng.next_u64(), 9981545732273789042ULL);
}

TEST(Random, SameSeedSameSequence) {
  Random a(42);
  Random b(42);
  for (int i = 0; i < 100; ++i) {
    ASSERT_EQ(a.normal(), b.normal());
    ASSERT_EQ(a.uniform01(), b.uniform01());
  }
  Random c(43);
  EXPECT_NE(Random(42).next_u64(), c.next_u64());
}

TEST(Random, DerivedStreamsAreDeterministicAndDistinct) {
  const Random base(7);
  Random s1 = base.derive(1);
  Random s1_again = base.derive(1);
  Random s2 = base.derive(2);
  const std::uint64_t v1 = s1.next_u64();
  EXPECT_EQ(v1, s1_again.next_u64());
  EXPECT_NE(v1, s2.next_u64());
}

TEST(Random, UniformRangeAndMoments) {
  Random rng(1);
  std::vector<double> v(20000);
  for (double& x : v) {
    x = rng.uniform(-2.0, 4.0);
    ASSERT_GE(x, -2.0);
    ASSERT_LT(x, 4.0);
  }
  EXPECT_NEAR(mean(v), 1.0, 0.05);
  EXPECT_NEAR(sample_variance(v), 3.0, 0.1);  // (b - a)^2 / 12
}

TEST(Random, UniformIndexCoversRangeWithoutBias) {
  Random rng(3);
  std::vector<int> counts(5, 0);
  const int n = 50000;
  for (int i = 0; i < n; ++i) {
    ++counts[rng.uniform_index(5)];
  }
  for (const int c : counts) {
    EXPECT_NEAR(static_cast<double>(c) / n, 0.2, 0.01);
  }
  EXPECT_THROW((void)rng.uniform_index(0), std::invalid_argument);
}

TEST(Random, NormalMoments) {
  Random rng(11);
  std::vector<double> v(50000);
  for (double& x : v) {
    x = rng.normal(3.0, 2.0);
  }
  EXPECT_NEAR(mean(v), 3.0, 0.03);
  EXPECT_NEAR(std::sqrt(sample_variance(v)), 2.0, 0.03);
  EXPECT_THROW((void)rng.normal(0.0, -1.0), std::invalid_argument);
}

TEST(Random, MultivariateNormalCovariance) {
  Random rng(5);
  Mat2 cov;
  cov << 2.0, 0.6, 0.6, 0.5;
  const Vec2 mu(1.0, -1.0);
  const int n = 40000;
  MatX samples(2, n);
  for (int i = 0; i < n; ++i) {
    samples.col(i) = rng.multivariate_normal(mu, cov);
  }
  EXPECT_MATRIX_NEAR(sample_mean(samples), mu, 0.03);
  EXPECT_MATRIX_NEAR(sample_covariance(samples), cov, 0.05);
}

TEST(Random, ZeroCovarianceReturnsMean) {
  Random rng(5);
  const Vec2 mu(1.0, 2.0);
  EXPECT_MATRIX_NEAR(rng.multivariate_normal(mu, Mat2::Zero()), mu, 1e-100);
}

}  // namespace
}  // namespace roboticscore::math
