#pragma once

/// @file linear_algebra.hpp
/// @brief Robotics-specific helpers on top of Eigen: covariance hygiene and safe
///        factorisations. General linear algebra is left to Eigen itself.

#include "roboticscore/core/types.hpp"

namespace roboticscore::math {

/// Returns (M + M^T) / 2. Used after covariance updates to remove round-off asymmetry.
[[nodiscard]] MatX symmetrize(const MatX& m);

/// True if M is square and |M - M^T|_max <= tol * max(1, |M|_max).
[[nodiscard]] bool is_symmetric(const MatX& m, double tol = 1e-12);

/// True if M is symmetric and its Cholesky factorisation succeeds.
[[nodiscard]] bool is_positive_definite(const MatX& m);

/// Projects a symmetric matrix onto the cone of matrices with eigenvalues >= min_eigenvalue
/// by clipping its spectrum (the Frobenius-nearest such matrix).
/// @throws std::invalid_argument if M is not square or min_eigenvalue < 0.
[[nodiscard]] MatX nearest_positive_definite(const MatX& m, double min_eigenvalue = 1e-12);

/// Lower Cholesky factor L of a symmetric positive (semi)definite matrix, adding diagonal
/// jitter (starting at initial_jitter * mean(diag), growing x10 per attempt) only if the plain
/// factorisation fails. This keeps sigma-point generation robust to round-off.
/// @throws std::invalid_argument if M is not square.
/// @throws NumericalError if no factorisation succeeds within max_attempts.
[[nodiscard]] MatX cholesky_lower(const MatX& m, double initial_jitter = 1e-12,
                                  int max_attempts = 8);

/// 2-norm condition number sigma_max / sigma_min (infinity for singular matrices).
[[nodiscard]] double condition_number(const MatX& m);

/// Solves A x = b for symmetric positive definite A via Cholesky (LDLT fallback for
/// semidefinite systems).
/// @throws std::invalid_argument on dimension mismatch.
/// @throws NumericalError if A is indefinite or singular.
[[nodiscard]] VecX solve_spd(const MatX& a, const VecX& b);

/// Skew-symmetric 2x2 generator for planar rotations, [[0, -w], [w, 0]].
[[nodiscard]] inline Mat2 skew2(double w) {
  Mat2 s;
  s << 0.0, -w, w, 0.0;
  return s;
}

}  // namespace roboticscore::math
