#include "roboticscore/estimation/unscented_kalman_filter.hpp"

#include <cmath>
#include <utility>

#include <Eigen/Cholesky>

#include "roboticscore/core/error.hpp"
#include "roboticscore/estimation/kalman_filter.hpp"
#include "roboticscore/math/linear_algebra.hpp"

namespace roboticscore::estimation {

SigmaPoints merwe_sigma_points(const VecX& mean, const MatX& covariance,
                               const SigmaPointParams& params, const StateSpace& space) {
  const Index n = mean.size();
  detail::require(covariance.rows() == n && covariance.cols() == n,
                  "merwe_sigma_points: covariance dimension mismatch");
  detail::require(params.alpha > 0.0, "merwe_sigma_points: alpha must be positive");
  const auto nd = static_cast<double>(n);
  const double lambda = params.alpha * params.alpha * (nd + params.kappa) - nd;
  const double scale = nd + lambda;
  detail::require(scale > 0.0, "merwe_sigma_points: n + lambda must be positive");

  const MatX l = math::cholesky_lower(scale * covariance);
  SigmaPoints sp;
  sp.points.resize(n, 2 * n + 1);
  sp.points.col(0) = mean;
  for (Index i = 0; i < n; ++i) {
    sp.points.col(1 + i) = space.canonical(mean + l.col(i));
    sp.points.col(1 + n + i) = space.canonical(mean - l.col(i));
  }
  sp.mean_weights = VecX::Constant(2 * n + 1, 0.5 / scale);
  sp.covariance_weights = sp.mean_weights;
  sp.mean_weights(0) = lambda / scale;
  sp.covariance_weights(0) = lambda / scale + (1.0 - params.alpha * params.alpha + params.beta);
  return sp;
}

UnscentedKalmanFilter::UnscentedKalmanFilter(VecX x0, MatX p0, SigmaPointParams params,
                                             StateSpace space)
    : params_(params), space_(std::move(space)) {
  reset(std::move(x0), std::move(p0));
}

void UnscentedKalmanFilter::reset(VecX x, MatX p) {
  detail::require(x.allFinite(), "UnscentedKalmanFilter: state must be finite");
  validate_covariance(p, x.size(), "UnscentedKalmanFilter");
  x_ = std::move(x);
  p_ = math::symmetrize(p);
}

void UnscentedKalmanFilter::predict(const ProcessModel& model, const VecX& u, double dt) {
  detail::require(static_cast<bool>(model.f) && static_cast<bool>(model.noise),
                  "UnscentedKalmanFilter::predict: model.f and model.noise are required");
  const Index n = x_.size();
  const SigmaPoints sp = merwe_sigma_points(x_, p_, params_, space_);
  MatX propagated(n, sp.points.cols());
  for (Index i = 0; i < sp.points.cols(); ++i) {
    const VecX xi = model.f(sp.points.col(i), u, dt);
    detail::require(xi.size() == n, "UnscentedKalmanFilter::predict: f changed the dimension");
    propagated.col(i) = space_.canonical(xi);
  }
  const VecX mean = space_.canonical(space_.average(propagated, sp.mean_weights));
  MatX cov = model.noise(x_, u, dt);
  detail::require(cov.rows() == n && cov.cols() == n,
                  "UnscentedKalmanFilter::predict: noise dimension mismatch");
  for (Index i = 0; i < propagated.cols(); ++i) {
    const VecX d = space_.diff(propagated.col(i), mean);
    cov += sp.covariance_weights(i) * d * d.transpose();
  }
  x_ = mean;
  p_ = math::symmetrize(cov);
}

UpdateResult UnscentedKalmanFilter::update(const MeasurementModel& model, const VecX& z,
                                           double gate) {
  detail::require(static_cast<bool>(model.h), "UnscentedKalmanFilter::update: model.h is empty");
  const Index m = z.size();
  detail::require(model.noise.rows() == m && model.noise.cols() == m,
                  "UnscentedKalmanFilter::update: R dimension mismatch");
  const SigmaPoints sp = merwe_sigma_points(x_, p_, params_, space_);
  const Index count = sp.points.cols();

  MatX zs(m, count);
  for (Index i = 0; i < count; ++i) {
    const VecX zi = model.h(sp.points.col(i));
    detail::require(zi.size() == m, "UnscentedKalmanFilter::update: measurement size mismatch");
    zs.col(i) = zi;
  }
  const VecX z_mean = model.mean ? model.mean(zs, sp.mean_weights) : VecX(zs * sp.mean_weights);
  const auto z_diff = [&](const VecX& a, const VecX& b) -> VecX {
    return model.residual ? model.residual(a, b) : VecX(a - b);
  };

  MatX s = model.noise;
  MatX cross = MatX::Zero(x_.size(), m);
  for (Index i = 0; i < count; ++i) {
    const VecX dz = z_diff(zs.col(i), z_mean);
    const VecX dx = space_.diff(sp.points.col(i), x_);
    s += sp.covariance_weights(i) * dz * dz.transpose();
    cross += sp.covariance_weights(i) * dx * dz.transpose();
  }

  UpdateResult result;
  result.stats.innovation = z_diff(z, z_mean);
  result.stats.covariance = math::symmetrize(s);
  const Eigen::LLT<MatX> llt(result.stats.covariance);
  if (llt.info() != Eigen::Success) {
    throw NumericalError("UnscentedKalmanFilter::update: innovation covariance is not PD");
  }
  result.stats.nis = result.stats.innovation.dot(llt.solve(result.stats.innovation));
  if (result.stats.nis > gate) {
    result.accepted = false;
    return result;
  }
  const MatX k = llt.solve(cross.transpose()).transpose();  // K = Pxz S^{-1}
  x_ = space_.canonical(x_ + k * result.stats.innovation);
  p_ = math::symmetrize(p_ - k * result.stats.covariance * k.transpose());
  return result;
}

}  // namespace roboticscore::estimation
