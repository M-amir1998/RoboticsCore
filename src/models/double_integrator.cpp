#include "roboticscore/models/double_integrator.hpp"

#include "roboticscore/core/error.hpp"

namespace roboticscore::models {

DoubleIntegrator::DoubleIntegrator(int dimension, double dt) : dimension_(dimension), dt_(dt) {
  detail::require(dimension >= 1, "DoubleIntegrator: dimension must be at least 1");
  detail::require(dt > 0.0, "DoubleIntegrator: dt must be positive");
  const Index n = dimension;
  const MatX eye = MatX::Identity(n, n);
  a_ = MatX::Identity(2 * n, 2 * n);
  a_.topRightCorner(n, n) = dt * eye;
  b_ = MatX::Zero(2 * n, n);
  b_.topRows(n) = 0.5 * dt * dt * eye;
  b_.bottomRows(n) = dt * eye;
}

VecX DoubleIntegrator::step(const VecX& x, const VecX& u) const {
  detail::require(x.size() == a_.rows() && u.size() == b_.cols(),
                  "DoubleIntegrator::step: dimension mismatch");
  return a_ * x + b_ * u;
}

MatX DoubleIntegrator::process_noise(double q) const {
  detail::require(q >= 0.0, "DoubleIntegrator::process_noise: density must be non-negative");
  const Index n = dimension_;
  const MatX eye = MatX::Identity(n, n);
  MatX qd(2 * n, 2 * n);
  const double dt2 = dt_ * dt_;
  qd.topLeftCorner(n, n) = (dt2 * dt_ / 3.0) * eye;
  qd.topRightCorner(n, n) = (dt2 / 2.0) * eye;
  qd.bottomLeftCorner(n, n) = (dt2 / 2.0) * eye;
  qd.bottomRightCorner(n, n) = dt_ * eye;
  return q * qd;
}

MatX DoubleIntegrator::position_measurement() const {
  const Index n = dimension_;
  MatX h = MatX::Zero(n, 2 * n);
  h.leftCols(n) = MatX::Identity(n, n);
  return h;
}

}  // namespace roboticscore::models
