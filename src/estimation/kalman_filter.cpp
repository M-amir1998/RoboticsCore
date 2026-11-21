#include "roboticscore/estimation/kalman_filter.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/linear_algebra.hpp"

namespace roboticscore::estimation {

void validate_covariance(const MatX& p, Index n, const char* who) {
  const std::string prefix(who);
  detail::require(p.rows() == n && p.cols() == n, prefix + ": covariance dimension mismatch");
  detail::require(p.allFinite(), prefix + ": covariance must be finite");
  detail::require(math::is_symmetric(p, 1e-9), prefix + ": covariance must be symmetric");
  if (n > 0) {
    const Eigen::SelfAdjointEigenSolver<MatX> eig(math::symmetrize(p), Eigen::EigenvaluesOnly);
    const double scale = std::max(1.0, p.cwiseAbs().maxCoeff());
    detail::require(eig.eigenvalues().minCoeff() >= -1e-12 * scale,
                    prefix + ": covariance must be positive semidefinite");
  }
}

UpdateResult kalman_update(VecX& x, MatX& p, const VecX& y, const MatX& h, const MatX& r,
                           double gate) {
  const Index n = x.size();
  const Index m = y.size();
  detail::require(h.rows() == m && h.cols() == n, "kalman_update: H dimension mismatch");
  detail::require(r.rows() == m && r.cols() == m, "kalman_update: R dimension mismatch");

  UpdateResult result;
  result.stats.innovation = y;
  const MatX ph_t = p * h.transpose();
  result.stats.covariance = math::symmetrize(h * ph_t + r);
  const Eigen::LLT<MatX> llt(result.stats.covariance);
  if (llt.info() != Eigen::Success) {
    throw NumericalError("kalman_update: innovation covariance is not positive definite");
  }
  result.stats.nis = y.dot(llt.solve(y));
  if (result.stats.nis > gate) {
    result.accepted = false;
    return result;
  }
  // K = P H^T S^{-1}, computed as the solution of S K^T = H P.
  const MatX k = llt.solve(ph_t.transpose()).transpose();
  x += k * y;
  const MatX i_kh = MatX::Identity(n, n) - k * h;
  p = math::symmetrize(i_kh * p * i_kh.transpose() + k * r * k.transpose());
  return result;
}

KalmanFilter::KalmanFilter(VecX x0, const MatX& p0) {
  reset(std::move(x0), p0);
}

void KalmanFilter::reset(VecX x, const MatX& p) {
  detail::require(x.allFinite(), "KalmanFilter: state must be finite");
  validate_covariance(p, x.size(), "KalmanFilter");
  x_ = std::move(x);
  p_ = math::symmetrize(p);
}

void KalmanFilter::predict(const MatX& a, const MatX& q) {
  const Index n = x_.size();
  detail::require(a.rows() == n && a.cols() == n, "KalmanFilter::predict: A dimension mismatch");
  detail::require(q.rows() == n && q.cols() == n, "KalmanFilter::predict: Q dimension mismatch");
  x_ = a * x_;
  p_ = math::symmetrize(a * p_ * a.transpose() + q);
}

void KalmanFilter::predict(const MatX& a, const MatX& b, const VecX& u, const MatX& q) {
  detail::require(b.rows() == x_.size() && b.cols() == u.size(),
                  "KalmanFilter::predict: B dimension mismatch");
  predict(a, q);
  x_ += b * u;
}

UpdateResult KalmanFilter::update(const VecX& z, const MatX& h, const MatX& r, double gate) {
  detail::require(h.rows() == z.size(), "KalmanFilter::update: H/z dimension mismatch");
  detail::require(h.cols() == x_.size(), "KalmanFilter::update: H/x dimension mismatch");
  const VecX y = z - h * x_;
  return kalman_update(x_, p_, y, h, r, gate);
}

}  // namespace roboticscore::estimation
