#include "roboticscore/optimization/qp_solver.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>

#include "roboticscore/core/error.hpp"

namespace roboticscore::optimization {

QpResult solve_qp(const QpProblem& prob, const QpSettings& s, const VecX* warm_x,
                  const VecX* warm_y) {
  const Index n = prob.q.size();
  const Index m = prob.a.rows();
  detail::require(prob.p.rows() == n && prob.p.cols() == n, "solve_qp: P must be n x n");
  detail::require(prob.a.cols() == n || m == 0, "solve_qp: A must have n columns");
  detail::require(prob.l.size() == m && prob.u.size() == m, "solve_qp: bounds must have m rows");
  detail::require((prob.l.array() <= prob.u.array()).all(), "solve_qp: l must not exceed u");
  detail::require(s.rho > 0.0 && s.sigma > 0.0 && s.alpha > 0.0 && s.alpha < 2.0,
                  "solve_qp: invalid ADMM parameters");
  detail::require(s.max_iterations > 0, "solve_qp: max_iterations must be positive");

  // Per-constraint penalty: larger for equality rows, as in OSQP.
  VecX rho = VecX::Constant(m, s.rho);
  for (Index i = 0; i < m; ++i) {
    if (prob.u(i) - prob.l(i) < 1e-9) {
      rho(i) = s.rho * s.equality_rho_scale;
    }
  }
  MatX kkt = prob.p;
  kkt.diagonal().array() += s.sigma;
  if (m > 0) {
    kkt += prob.a.transpose() * rho.asDiagonal() * prob.a;
  }
  const Eigen::LDLT<MatX> ldlt(0.5 * (kkt + kkt.transpose()));
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    throw NumericalError("solve_qp: KKT matrix is not positive definite (is P PSD?)");
  }

  QpResult r;
  r.x = (warm_x != nullptr && warm_x->size() == n) ? *warm_x : VecX(VecX::Zero(n));
  r.y = (warm_y != nullptr && warm_y->size() == m) ? *warm_y : VecX(VecX::Zero(m));
  r.z = m > 0 ? VecX((prob.a * r.x).cwiseMax(prob.l).cwiseMin(prob.u)) : VecX(VecX::Zero(0));

  for (int it = 1; it <= s.max_iterations; ++it) {
    r.iterations = it;
    VecX rhs = s.sigma * r.x - prob.q;
    if (m > 0) {
      rhs += prob.a.transpose() * (rho.cwiseProduct(r.z) - r.y);
    }
    const VecX x_tilde = ldlt.solve(rhs);
    const VecX x_next = s.alpha * x_tilde + (1.0 - s.alpha) * r.x;
    if (m > 0) {
      const VecX z_tilde = prob.a * x_tilde;
      const VecX z_relaxed = s.alpha * z_tilde + (1.0 - s.alpha) * r.z;
      const VecX z_next = (z_relaxed + r.y.cwiseQuotient(rho)).cwiseMax(prob.l).cwiseMin(prob.u);
      r.y += rho.cwiseProduct(z_relaxed - z_next);
      r.z = z_next;
    }
    r.x = x_next;

    // Convergence check every few iterations (residuals cost a matrix-vector product).
    if (it % 5 == 0 || it == s.max_iterations) {
      const VecX ax = m > 0 ? VecX(prob.a * r.x) : VecX(VecX::Zero(0));
      const VecX px = prob.p * r.x;
      const VecX aty = m > 0 ? VecX(prob.a.transpose() * r.y) : VecX(VecX::Zero(n));
      r.primal_residual = m > 0 ? (ax - r.z).lpNorm<Eigen::Infinity>() : 0.0;
      r.dual_residual = (px + prob.q + aty).lpNorm<Eigen::Infinity>();
      const double eps_prim =
          s.eps_abs
          + s.eps_rel
                * (m > 0 ? std::max(ax.lpNorm<Eigen::Infinity>(), r.z.lpNorm<Eigen::Infinity>())
                         : 0.0);
      const double eps_dual =
          s.eps_abs
          + s.eps_rel
                * std::max({px.lpNorm<Eigen::Infinity>(), aty.lpNorm<Eigen::Infinity>(),
                            prob.q.lpNorm<Eigen::Infinity>()});
      if (r.primal_residual <= eps_prim && r.dual_residual <= eps_dual) {
        r.status = QpStatus::kSolved;
        break;
      }
    }
  }
  r.objective = 0.5 * r.x.dot(prob.p * r.x) + prob.q.dot(r.x);
  return r;
}

}  // namespace roboticscore::optimization
