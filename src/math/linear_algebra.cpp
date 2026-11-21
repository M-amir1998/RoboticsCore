#include "roboticscore/math/linear_algebra.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include "roboticscore/core/error.hpp"

namespace roboticscore::math {

MatX symmetrize(const MatX& m) {
  detail::require(m.rows() == m.cols(), "symmetrize: matrix must be square");
  return 0.5 * (m + m.transpose());
}

bool is_symmetric(const MatX& m, double tol) {
  if (m.rows() != m.cols()) {
    return false;
  }
  if (m.size() == 0) {
    return true;
  }
  const double scale = std::max(1.0, m.cwiseAbs().maxCoeff());
  return (m - m.transpose()).cwiseAbs().maxCoeff() <= tol * scale;
}

bool is_positive_definite(const MatX& m) {
  if (!is_symmetric(m, 1e-9)) {
    return false;
  }
  const Eigen::LLT<MatX> llt(m);
  return llt.info() == Eigen::Success;
}

MatX nearest_positive_definite(const MatX& m, double min_eigenvalue) {
  detail::require(m.rows() == m.cols(), "nearest_positive_definite: matrix must be square");
  detail::require(min_eigenvalue >= 0.0,
                  "nearest_positive_definite: min_eigenvalue must be non-negative");
  const Eigen::SelfAdjointEigenSolver<MatX> eig(symmetrize(m));
  const VecX clipped = eig.eigenvalues().cwiseMax(min_eigenvalue);
  return eig.eigenvectors() * clipped.asDiagonal() * eig.eigenvectors().transpose();
}

MatX cholesky_lower(const MatX& m, double initial_jitter, int max_attempts) {
  detail::require(m.rows() == m.cols(), "cholesky_lower: matrix must be square");
  const MatX sym = symmetrize(m);
  Eigen::LLT<MatX> llt(sym);
  if (llt.info() == Eigen::Success) {
    return llt.matrixL();
  }
  const double diag_scale = std::max(sym.diagonal().cwiseAbs().mean(), 1e-300);
  double jitter = initial_jitter * diag_scale;
  const auto n = sym.rows();
  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    llt.compute(sym + jitter * MatX::Identity(n, n));
    if (llt.info() == Eigen::Success) {
      return llt.matrixL();
    }
    jitter *= 10.0;
  }
  throw NumericalError("cholesky_lower: matrix is not positive semidefinite");
}

double condition_number(const MatX& m) {
  if (m.size() == 0) {
    return 1.0;
  }
  const Eigen::JacobiSVD<MatX> svd(m);
  const VecX& s = svd.singularValues();
  const double smin = s(s.size() - 1);
  if (smin <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return s(0) / smin;
}

VecX solve_spd(const MatX& a, const VecX& b) {
  detail::require(a.rows() == a.cols() && a.rows() == b.size(), "solve_spd: dimension mismatch");
  const Eigen::LLT<MatX> llt(a);
  if (llt.info() == Eigen::Success) {
    return llt.solve(b);
  }
  const Eigen::LDLT<MatX> ldlt(a);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    throw NumericalError("solve_spd: matrix is not positive semidefinite");
  }
  const VecX x = ldlt.solve(b);
  if (!x.allFinite() || !(a * x).isApprox(b, 1e-8)) {
    throw NumericalError("solve_spd: system is singular");
  }
  return x;
}

}  // namespace roboticscore::math
