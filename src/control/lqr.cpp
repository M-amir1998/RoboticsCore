#include "roboticscore/control/lqr.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/linear_algebra.hpp"

namespace roboticscore::control {
namespace {

void validate_lqr(const MatX& a, const MatX& b, const MatX& q, const MatX& r) {
  const Index n = a.rows();
  detail::require(a.cols() == n, "LQR: A must be square");
  detail::require(b.rows() == n, "LQR: B must have as many rows as A");
  detail::require(q.rows() == n && q.cols() == n, "LQR: Q must be n x n");
  detail::require(r.rows() == b.cols() && r.cols() == b.cols(), "LQR: R must be m x m");
  detail::require(math::is_symmetric(q, 1e-9), "LQR: Q must be symmetric");
  detail::require(math::is_positive_definite(r), "LQR: R must be positive definite");
  const Eigen::SelfAdjointEigenSolver<MatX> eig(q, Eigen::EigenvaluesOnly);
  detail::require(eig.eigenvalues().minCoeff() >= -1e-12, "LQR: Q must be positive semidefinite");
}

MatX gain_from(const MatX& a, const MatX& b, const MatX& r, const MatX& p) {
  const MatX s = r + b.transpose() * p * b;
  return s.ldlt().solve(b.transpose() * p * a);
}

double dare_residual(const MatX& a, const MatX& b, const MatX& q, const MatX& r, const MatX& p) {
  const MatX k = gain_from(a, b, r, p);
  const MatX res = a.transpose() * p * a - a.transpose() * p * b * k + q - p;
  return res.norm() / std::max(1.0, p.norm());
}

}  // namespace

DareResult solve_dare(const MatX& a, const MatX& b, const MatX& q, const MatX& r,
                      int max_iterations, double tolerance) {
  validate_lqr(a, b, q, r);
  const Index n = a.rows();
  const MatX eye = MatX::Identity(n, n);
  MatX ak = a;
  MatX gk = b * r.llt().solve(b.transpose());
  MatX hk = q;
  DareResult result;
  for (int it = 1; it <= max_iterations; ++it) {
    result.iterations = it;
    const Eigen::PartialPivLU<MatX> w(eye + gk * hk);
    const MatX w_inv_a = w.solve(ak);
    const MatX w_inv_g = w.solve(gk);
    const MatX h_next = hk + ak.transpose() * hk * w_inv_a;
    const MatX g_next = gk + ak * w_inv_g * ak.transpose();
    const MatX a_next = ak * w_inv_a;
    const double change = (h_next - hk).norm() / std::max(1.0, h_next.norm());
    hk = math::symmetrize(h_next);
    gk = math::symmetrize(g_next);
    ak = a_next;
    if (!hk.allFinite()) {
      break;  // diverged: no stabilising solution
    }
    if (change < tolerance) {
      break;
    }
  }
  result.p = hk;
  if (hk.allFinite()) {
    result.k = gain_from(a, b, r, hk);
    result.residual = dare_residual(a, b, q, r, hk);
    // Converged only if the residual is small and the closed loop is stable.
    const Eigen::EigenSolver<MatX> eig(a - b * result.k, false);
    const double spectral_radius = eig.eigenvalues().cwiseAbs().maxCoeff();
    result.converged = result.residual < 1e-8 && spectral_radius < 1.0;
  }
  return result;
}

std::vector<MatX> finite_horizon_lqr(const MatX& a, const MatX& b, const MatX& q, const MatX& r,
                                     const MatX& qf, int horizon) {
  validate_lqr(a, b, q, r);
  detail::require(qf.rows() == a.rows() && qf.cols() == a.rows(), "finite_horizon_lqr: Qf size");
  detail::require(horizon >= 1, "finite_horizon_lqr: horizon must be at least 1");
  std::vector<MatX> gains(static_cast<std::size_t>(horizon));
  MatX p = qf;
  for (int k = horizon - 1; k >= 0; --k) {
    const MatX kk = gain_from(a, b, r, p);
    gains[static_cast<std::size_t>(k)] = kk;
    p = math::symmetrize(q + a.transpose() * p * (a - b * kk));
  }
  return gains;
}

LqrTracker::LqrTracker(const models::BicycleParams& vehicle, const LqrTrackerConfig& config)
    : PathTracker(vehicle, config.speed_pid), config_(config) {
  detail::require(config.q_lateral > 0.0 && config.q_heading >= 0.0 && config.r_steering > 0.0,
                  "LqrTracker: weights must be positive");
  detail::require(config.min_gain_speed > 0.0, "LqrTracker: min_gain_speed must be positive");
}

VehicleCommand LqrTracker::compute(const models::BicycleState& state,
                                   const trajectory::ReferencePath& path, double dt) {
  detail::require(dt > 0.0, "LqrTracker::compute: dt must be positive");
  const double v = state(3);
  const auto proj = track_point(Vec2(state(0), state(1)), state(2), path);
  status_.target = proj.point;
  // Gain-schedule on the current speed; use the reference direction when nearly stopped.
  double direction = status_.reference_speed < 0.0 ? -1.0 : 1.0;
  if (std::abs(v) > 1e-3) {
    direction = v > 0.0 ? 1.0 : -1.0;
  }
  const double v_gain = direction * std::max(std::abs(v), config_.min_gain_speed);
  const double delta_ff =
      config_.curvature_feedforward ? std::atan(vehicle_.wheelbase * proj.curvature) : 0.0;
  const double c_ff = std::cos(delta_ff);
  MatX a(2, 2);
  a << 1.0, v_gain * dt, 0.0, 1.0;
  MatX b(2, 1);
  b << 0.0, v_gain * dt / (vehicle_.wheelbase * c_ff * c_ff);
  MatX q = MatX::Zero(2, 2);
  q(0, 0) = config_.q_lateral;
  q(1, 1) = config_.q_heading;
  const MatX r = MatX::Constant(1, 1, config_.r_steering);
  const DareResult dare = solve_dare(a, b, q, r);
  gain_ = dare.k;
  const Vec2 e(status_.cross_track_error, status_.heading_error);
  const double delta_fb = -(gain_ * e)(0);
  VehicleCommand cmd;
  cmd.steering = clamp_steering(delta_ff + delta_fb);
  cmd.acceleration = speed_control(v, dt);
  return cmd;
}

}  // namespace roboticscore::control
