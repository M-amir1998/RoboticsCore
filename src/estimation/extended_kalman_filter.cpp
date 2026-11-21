#include "roboticscore/estimation/extended_kalman_filter.hpp"

#include <utility>

#include "roboticscore/core/error.hpp"
#include "roboticscore/estimation/kalman_filter.hpp"
#include "roboticscore/math/linear_algebra.hpp"
#include "roboticscore/math/numerical_diff.hpp"

namespace roboticscore::estimation {

ExtendedKalmanFilter::ExtendedKalmanFilter(VecX x0, MatX p0) {
  reset(std::move(x0), std::move(p0));
}

void ExtendedKalmanFilter::reset(VecX x, MatX p) {
  detail::require(x.allFinite(), "ExtendedKalmanFilter: state must be finite");
  validate_covariance(p, x.size(), "ExtendedKalmanFilter");
  x_ = std::move(x);
  p_ = math::symmetrize(p);
}

void ExtendedKalmanFilter::predict(const ProcessModel& model, const VecX& u, double dt) {
  detail::require(static_cast<bool>(model.f), "ExtendedKalmanFilter::predict: model.f is empty");
  detail::require(static_cast<bool>(model.noise),
                  "ExtendedKalmanFilter::predict: model.noise is empty");
  const Index n = x_.size();

  MatX f_jac;
  if (model.jacobian) {
    f_jac = model.jacobian(x_, u, dt);
  } else {
    const auto f = [&](const VecX& x) {
      return model.f(x, u, dt);
    };
    f_jac = model.residual ? math::numerical_jacobian(f, x_, model.residual)
                           : math::numerical_jacobian(f, x_);
  }
  const MatX q = model.noise(x_, u, dt);
  detail::require(f_jac.rows() == n && f_jac.cols() == n,
                  "ExtendedKalmanFilter::predict: Jacobian dimension mismatch");
  detail::require(q.rows() == n && q.cols() == n,
                  "ExtendedKalmanFilter::predict: noise dimension mismatch");

  VecX x_next = model.f(x_, u, dt);
  detail::require(x_next.size() == n, "ExtendedKalmanFilter::predict: f changed the dimension");
  if (model.normalize) {
    x_next = model.normalize(x_next);
  }
  x_ = std::move(x_next);
  p_ = math::symmetrize(f_jac * p_ * f_jac.transpose() + q);
}

UpdateResult ExtendedKalmanFilter::update(const MeasurementModel& model, const VecX& z,
                                          double gate) {
  detail::require(static_cast<bool>(model.h), "ExtendedKalmanFilter::update: model.h is empty");
  const VecX z_pred = model.h(x_);
  detail::require(z_pred.size() == z.size(),
                  "ExtendedKalmanFilter::update: measurement dimension mismatch");

  MatX h_jac;
  if (model.jacobian) {
    h_jac = model.jacobian(x_);
  } else {
    h_jac = model.residual ? math::numerical_jacobian(model.h, x_, model.residual)
                           : math::numerical_jacobian(model.h, x_);
  }
  const VecX y = model.residual ? model.residual(z, z_pred) : VecX(z - z_pred);
  UpdateResult result = kalman_update(x_, p_, y, h_jac, model.noise, gate);
  if (result.accepted && normalize_) {
    x_ = normalize_(x_);
  }
  return result;
}

}  // namespace roboticscore::estimation
