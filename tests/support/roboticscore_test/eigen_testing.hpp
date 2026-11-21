#pragma once

// Shared GoogleTest helpers for Eigen matrices and deterministic randomized tests.

#include <cstdint>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace roboticscore::testing {

/// Element-wise absolute comparison with a readable failure message.
template <typename A, typename B>
::testing::AssertionResult matrices_near(const char* a_expr, const char* b_expr, const char*,
                                         const Eigen::MatrixBase<A>& a,
                                         const Eigen::MatrixBase<B>& b, double tol) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) {
    return ::testing::AssertionFailure()
           << a_expr << " is " << a.rows() << "x" << a.cols() << " but " << b_expr << " is "
           << b.rows() << "x" << b.cols();
  }
  const double err = (a.template cast<double>() - b.template cast<double>()).cwiseAbs().maxCoeff();
  if (err <= tol) {
    return ::testing::AssertionSuccess();
  }
  std::ostringstream os;
  const Eigen::IOFormat fmt(Eigen::FullPrecision, 0, ", ", "\n", "[", "]");
  os << "max |" << a_expr << " - " << b_expr << "| = " << err << " > " << tol << "\n"
     << a_expr << " =\n"
     << a.format(fmt) << "\n"
     << b_expr << " =\n"
     << b.format(fmt);
  return ::testing::AssertionFailure() << os.str();
}

/// Seed for randomized property tests. Fixed so failures are reproducible; every test that
/// draws random inputs prints its seed through SCOPED_TRACE on failure.
inline constexpr std::uint64_t kPropertySeed = 20240607ULL;

}  // namespace roboticscore::testing

#define EXPECT_MATRIX_NEAR(a, b, tol) \
  EXPECT_PRED_FORMAT3(::roboticscore::testing::matrices_near, a, b, tol)
#define ASSERT_MATRIX_NEAR(a, b, tol) \
  ASSERT_PRED_FORMAT3(::roboticscore::testing::matrices_near, a, b, tol)
