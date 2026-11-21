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

  // Ruiz equilibration (OSQP, sec. 5.1): solve the scaled problem
  //   P' = c D P D, q' = c D q, A' = E A D, l' = E l, u' = E u
  // and recover x = D x', y = E y' / c. Badly scaled MPC problems converge far faster.
  VecX dscale = VecX::Ones(n);
  VecX escale = VecX::Ones(m);
  double cost_scale = 1.0;
  MatX ps = prob.p;
  VecX qs = prob.q;
  MatX as = prob.a;
  const auto inv_sqrt_clamped = [](double v) { return 1.0 / std::sqrt(std::clamp(v, 1e-4, 1e4)); };
  for (int k = 0; k < s.scaling_iterations; ++k) {
    VecX d(n);
    for (Index j = 0; j < n; ++j) {
      double norm = ps.col(j).cwiseAbs().maxCoeff();
      if (m > 0) norm = std::max(norm, as.col(j).cwiseAbs().maxCoeff());
      d(j) = inv_sqrt_clamped(norm);
    }
    VecX e(m);
    for (Index i = 0; i < m; ++i) {
      e(i) = inv_sqrt_clamped(as.row(i).cwiseAbs().maxCoeff());
    }
    ps = d.asDiagonal() * ps * d.asDiagonal();
    qs = d.cwiseProduct(qs);
    if (m > 0) as = e.asDiagonal() * as * d.asDiagonal();
    dscale = dscale.cwiseProduct(d);
    escale = escale.cwiseProduct(e);
    double p_norm = 0.0;
    for (Index j = 0; j < n; ++j)
      p_norm += ps.col(j).cwiseAbs().maxCoeff();
    p_norm /= static_cast<double>(std::max<Index>(n, 1));
    const double gamma =
        1.0 / std::clamp(std::max(p_norm, qs.lpNorm<Eigen::Infinity>()), 1e-4, 1e4);
    ps *= gamma;
    qs *= gamma;
    cost_scale *= gamma;
  }
  const VecX ls = escale.cwiseProduct(prob.l);
  const VecX us = escale.cwiseProduct(prob.u);

  // Per-constraint penalty: larger for equality rows, as in OSQP.
  double rho_scalar = s.rho;
  VecX rho(m);
  Eigen::LDLT<MatX> ldlt;
  const auto factorize = [&]() {
    for (Index i = 0; i < m; ++i) {
      rho(i) = prob.u(i) - prob.l(i) < 1e-9 ? rho_scalar * s.equality_rho_scale : rho_scalar;
    }
    MatX kkt = ps;
    kkt.diagonal().array() += s.sigma;
    if (m > 0) {
      kkt += as.transpose() * rho.asDiagonal() * as;
    }
    ldlt.compute(0.5 * (kkt + kkt.transpose()));
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
      throw NumericalError("solve_qp: KKT matrix is not positive definite (is P PSD?)");
    }
  };
  factorize();

  // Scaled iterates.
  VecX xs = (warm_x != nullptr && warm_x->size() == n) ? VecX(warm_x->cwiseQuotient(dscale))
                                                       : VecX(VecX::Zero(n));
  VecX ys = (warm_y != nullptr && warm_y->size() == m)
                ? VecX(cost_scale * warm_y->cwiseQuotient(escale))
                : VecX(VecX::Zero(m));
  VecX zs = m > 0 ? VecX((as * xs).cwiseMax(ls).cwiseMin(us)) : VecX(VecX::Zero(0));

  QpResult r;
  const auto unscale = [&]() {
    r.x = dscale.cwiseProduct(xs);
    r.y = escale.cwiseProduct(ys) / cost_scale;
    r.z = zs.cwiseQuotient(escale);
  };

  for (int it = 1; it <= s.max_iterations; ++it) {
    r.iterations = it;
    VecX rhs = s.sigma * xs - qs;
    if (m > 0) {
      rhs += as.transpose() * (rho.cwiseProduct(zs) - ys);
    }
    const VecX x_tilde = ldlt.solve(rhs);
    const VecX x_next = s.alpha * x_tilde + (1.0 - s.alpha) * xs;
    if (m > 0) {
      const VecX z_tilde = as * x_tilde;
      const VecX z_relaxed = s.alpha * z_tilde + (1.0 - s.alpha) * zs;
      const VecX z_next = (z_relaxed + ys.cwiseQuotient(rho)).cwiseMax(ls).cwiseMin(us);
      ys += rho.cwiseProduct(z_relaxed - z_next);
      zs = z_next;
    }
    xs = x_next;

    // Convergence is judged on the original (unscaled) problem every few iterations.
    if (it % 5 == 0 || it == s.max_iterations) {
      unscale();
      const VecX ax = m > 0 ? VecX(prob.a * r.x) : VecX(VecX::Zero(0));
      const VecX px = prob.p * r.x;
      const VecX aty = m > 0 ? VecX(prob.a.transpose() * r.y) : VecX(VecX::Zero(n));
      r.primal_residual = m > 0 ? (ax - r.z).lpNorm<Eigen::Infinity>() : 0.0;
      r.dual_residual = (px + prob.q + aty).lpNorm<Eigen::Infinity>();
      const double prim_scale =
          m > 0 ? std::max(ax.lpNorm<Eigen::Infinity>(), r.z.lpNorm<Eigen::Infinity>()) : 0.0;
      const double dual_scale =
          std::max({px.lpNorm<Eigen::Infinity>(), aty.lpNorm<Eigen::Infinity>(),
                    prob.q.lpNorm<Eigen::Infinity>()});
      if (r.primal_residual <= s.eps_abs + s.eps_rel * prim_scale
          && r.dual_residual <= s.eps_abs + s.eps_rel * dual_scale) {
        r.status = QpStatus::kSolved;
        break;
      }
      if (s.adaptive_rho && m > 0 && it % s.adaptive_rho_interval == 0) {
        // Same rule on the scaled problem, where rho actually acts.
        const VecX axs = as * xs;
        const VecX pxs = ps * xs;
        const VecX atys = as.transpose() * ys;
        const double prim =
            (axs - zs).lpNorm<Eigen::Infinity>()
            / std::max({axs.lpNorm<Eigen::Infinity>(), zs.lpNorm<Eigen::Infinity>(), 1e-12});
        const double dual =
            (pxs + qs + atys).lpNorm<Eigen::Infinity>()
            / std::max({pxs.lpNorm<Eigen::Infinity>(), atys.lpNorm<Eigen::Infinity>(),
                        qs.lpNorm<Eigen::Infinity>(), 1e-12});
        const double rho_new =
            std::clamp(rho_scalar * std::sqrt(prim / std::max(dual, 1e-12)), 1e-6, 1e6);
        if (rho_new > 5.0 * rho_scalar || rho_new < 0.2 * rho_scalar) {
          rho_scalar = rho_new;
          factorize();
        }
      }
    }
  }
  unscale();
  r.objective = 0.5 * r.x.dot(prob.p * r.x) + prob.q.dot(r.x);
  return r;
}

}  // namespace roboticscore::optimization
