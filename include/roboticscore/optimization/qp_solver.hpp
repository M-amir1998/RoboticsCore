#pragma once

/// @file qp_solver.hpp
/// @brief Dense convex quadratic programming with the ADMM operator splitting of Stellato,
///        Banjac, Goulart, Bemporad & Boyd, "OSQP: an operator splitting solver for quadratic
///        programs", Math. Prog. Comp. 2020:
///
///   minimize 0.5 x^T P x + q^T x   subject to   l <= A x <= u      (P symmetric PSD)
///
/// Each iteration solves one linear system with the fixed matrix P + sigma I + A^T diag(rho) A
/// (factorised once with LDLT), then projects onto the box [l, u]. Equality rows (l == u) use
/// a larger penalty as recommended in the paper. This is a small, dependency-free dense
/// implementation of that iteration for MPC-sized problems; it is not the OSQP library and has
/// no infeasibility certificates, sparse factorisation or solution polishing.
///
/// Convergence: primal residual |Ax - z|_inf and dual residual |Px + q + A^T y|_inf below
/// eps_abs + eps_rel * (scale of the respective terms).
///
/// The problem is first equilibrated with Ruiz scaling of variables, constraints and cost
/// (paper, sec. 5.1); termination is always judged on the unscaled problem.
///
/// Step-size adaptation (paper, sec. 5.2): every adaptive_rho_interval iterations rho is
/// rescaled by sqrt(normalised primal residual / normalised dual residual); the KKT matrix is
/// refactorised only when rho changes by more than a factor of 5.

#include "roboticscore/core/types.hpp"

namespace roboticscore::optimization {

/// A convex QP in OSQP form.
struct QpProblem {
  MatX p;  ///< n x n, symmetric positive semidefinite
  VecX q;  ///< n
  MatX a;  ///< m x n (m may be 0)
  VecX l;  ///< m, may contain -infinity
  VecX u;  ///< m, may contain +infinity
};

/// ADMM settings.
struct QpSettings {
  double rho = 0.1;
  double sigma = 1e-6;
  double alpha = 1.6;  ///< Over-relaxation in (0, 2).
  double eps_abs = 1e-6;
  double eps_rel = 1e-6;
  int max_iterations = 4000;
  double equality_rho_scale = 1e3;
  bool adaptive_rho = true;
  int adaptive_rho_interval = 25;
  int scaling_iterations = 10;  ///< Ruiz equilibration passes (0 disables scaling).
};

enum class QpStatus { kSolved, kMaxIterations };

/// Solution and diagnostics.
struct QpResult {
  QpStatus status = QpStatus::kMaxIterations;
  VecX x;
  VecX y;  ///< Constraint multipliers (positive for active upper bounds).
  VecX z;  ///< A x projected onto [l, u].
  int iterations = 0;
  double primal_residual = 0.0;
  double dual_residual = 0.0;
  double objective = 0.0;

  [[nodiscard]] bool solved() const noexcept { return status == QpStatus::kSolved; }
};

/// Solves the QP, optionally warm-started from a previous primal/dual solution.
/// @throws std::invalid_argument on dimension mismatch, l > u or invalid settings.
/// @throws NumericalError if the KKT matrix cannot be factorised (P not PSD).
[[nodiscard]] QpResult solve_qp(const QpProblem& problem, const QpSettings& settings = {},
                                const VecX* warm_x = nullptr, const VecX* warm_y = nullptr);

}  // namespace roboticscore::optimization
