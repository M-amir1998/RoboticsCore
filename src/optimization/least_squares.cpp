#include "roboticscore/optimization/least_squares.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Cholesky>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/numerical_diff.hpp"

namespace roboticscore::optimization {
namespace {

MatX jacobian_of(const LeastSquaresProblem& p, const VecX& x) {
  return p.jacobian ? p.jacobian(x) : math::numerical_jacobian(p.residual, x);
}

VecX apply_plus(const LeastSquaresProblem& p, const VecX& x, const VecX& dx) {
  return p.plus ? p.plus(x, dx) : VecX(x + dx);
}

double cost_of(const VecX& r) {
  return 0.5 * r.squaredNorm();
}

}  // namespace

SolverResult gauss_newton(const LeastSquaresProblem& problem, VecX x0,
                          const SolverOptions& options) {
  detail::require(static_cast<bool>(problem.residual), "gauss_newton: residual is required");
  SolverResult result;
  result.x = std::move(x0);
  VecX r = problem.residual(result.x);
  result.initial_cost = result.final_cost = cost_of(r);
  result.cost_history.push_back(result.final_cost);
  for (int it = 0; it < options.max_iterations; ++it) {
    const MatX j = jacobian_of(problem, result.x);
    const VecX g = j.transpose() * r;
    if (g.lpNorm<Eigen::Infinity>() <= options.gradient_tolerance) {
      result.status = SolverStatus::kGradientTolerance;
      return result;
    }
    const Eigen::LDLT<MatX> ldlt(j.transpose() * j);
    // LDLT "succeeds" on singular PSD matrices; reject rank-deficient normal equations
    // explicitly since Gauss-Newton has no unique step there (use LM instead).
    const VecX pivots = ldlt.vectorD().cwiseAbs();
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()
        || pivots.minCoeff() <= 1e-12 * std::max(pivots.maxCoeff(), 1e-300)) {
      result.status = SolverStatus::kNumericalFailure;
      return result;
    }
    const VecX dx = ldlt.solve(-g);
    if (!dx.allFinite()) {
      result.status = SolverStatus::kNumericalFailure;
      return result;
    }
    result.x = apply_plus(problem, result.x, dx);
    r = problem.residual(result.x);
    const double new_cost = cost_of(r);
    const double old_cost = result.final_cost;
    result.final_cost = new_cost;
    result.iterations = it + 1;
    result.cost_history.push_back(new_cost);
    if (dx.norm() <= options.step_tolerance * (result.x.norm() + options.step_tolerance)) {
      result.status = SolverStatus::kStepTolerance;
      return result;
    }
    if (std::abs(old_cost - new_cost) <= options.cost_tolerance * std::max(old_cost, 1e-300)) {
      result.status = SolverStatus::kCostTolerance;
      return result;
    }
  }
  result.status = SolverStatus::kMaxIterations;
  return result;
}

SolverResult levenberg_marquardt(const LeastSquaresProblem& problem, VecX x0,
                                 const SolverOptions& options) {
  detail::require(static_cast<bool>(problem.residual), "levenberg_marquardt: residual is required");
  SolverResult result;
  result.x = std::move(x0);
  VecX r = problem.residual(result.x);
  result.initial_cost = result.final_cost = cost_of(r);
  result.cost_history.push_back(result.final_cost);
  MatX j = jacobian_of(problem, result.x);
  MatX jtj = j.transpose() * j;
  VecX g = j.transpose() * r;
  double lambda = options.initial_lambda * std::max(jtj.diagonal().maxCoeff(), 1e-12);
  double nu = 2.0;
  for (int it = 0; it < options.max_iterations; ++it) {
    result.iterations = it + 1;
    if (g.lpNorm<Eigen::Infinity>() <= options.gradient_tolerance) {
      result.status = SolverStatus::kGradientTolerance;
      return result;
    }
    // Marquardt scaling with a floor so zero-curvature directions remain damped.
    VecX d = jtj.diagonal().cwiseMax(1e-12 * std::max(1.0, jtj.diagonal().maxCoeff()));
    MatX damped = jtj;
    damped.diagonal() += lambda * d;
    const Eigen::LDLT<MatX> ldlt(damped);
    const VecX dx = ldlt.solve(-g);
    if (ldlt.info() != Eigen::Success || !dx.allFinite()) {
      result.status = SolverStatus::kNumericalFailure;
      return result;
    }
    if (dx.norm() <= options.step_tolerance * (result.x.norm() + options.step_tolerance)) {
      result.status = SolverStatus::kStepTolerance;
      return result;
    }
    const VecX x_new = apply_plus(problem, result.x, dx);
    const VecX r_new = problem.residual(x_new);
    const double new_cost = cost_of(r_new);
    // Predicted reduction of the quadratic model: -g^T dx - 0.5 dx^T J^T J dx.
    const double predicted = -g.dot(dx) - 0.5 * dx.dot(jtj * dx);
    const double rho = predicted > 0.0 ? (result.final_cost - new_cost) / predicted : -1.0;
    if (rho > 0.0) {
      const double old_cost = result.final_cost;
      result.x = x_new;
      r = r_new;
      result.final_cost = new_cost;
      result.cost_history.push_back(new_cost);
      j = jacobian_of(problem, result.x);
      jtj = j.transpose() * j;
      g = j.transpose() * r;
      lambda *= std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * rho - 1.0, 3));
      nu = 2.0;
      if (std::abs(old_cost - new_cost) <= options.cost_tolerance * std::max(old_cost, 1e-300)) {
        result.status = SolverStatus::kCostTolerance;
        return result;
      }
    } else {
      lambda *= nu;
      nu *= 2.0;
      if (!std::isfinite(lambda) || lambda > 1e32) {
        result.status = SolverStatus::kNumericalFailure;
        return result;
      }
    }
  }
  result.status = SolverStatus::kMaxIterations;
  return result;
}

}  // namespace roboticscore::optimization
