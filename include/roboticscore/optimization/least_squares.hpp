#pragma once

/// @file least_squares.hpp
/// @brief Dense Gauss-Newton and Levenberg-Marquardt for small nonlinear least-squares
///        problems  min_x 0.5 |r(x)|^2.
///
/// These solvers are meant for problems with up to a few hundred parameters (calibration,
/// small alignment problems, tests). Large sparse problems such as pose graphs should use a
/// sparse solver (see slam::PoseGraph) or a dedicated library such as Ceres Solver or g2o.
///
///   Gauss-Newton:          (J^T J) dx = -J^T r
///   Levenberg-Marquardt:   (J^T J + lambda D) dx = -J^T r,  D = diag(J^T J)
/// LM adapts lambda with the gain ratio rho = actual / predicted cost reduction using
/// Nielsen's update (Madsen, Nielsen & Tingleff, "Methods for Non-Linear Least Squares
/// Problems", 2004, sec. 3.2). Parameters on a manifold can supply a plus operator x (+) dx.

#include <functional>
#include <vector>

#include "roboticscore/core/types.hpp"

namespace roboticscore::optimization {

/// Problem description.
struct LeastSquaresProblem {
  std::function<VecX(const VecX& x)> residual;
  /// dr/dx; central finite differences are used when empty.
  std::function<MatX(const VecX& x)> jacobian;
  /// x (+) dx for manifold parameters; vector addition when empty.
  std::function<VecX(const VecX& x, const VecX& dx)> plus;
};

/// Stopping criteria and LM parameters.
struct SolverOptions {
  int max_iterations = 100;
  double gradient_tolerance = 1e-10;  ///< Stop when |J^T r|_inf falls below this.
  double step_tolerance = 1e-12;      ///< Stop when |dx| <= tol * (|x| + tol).
  double cost_tolerance = 1e-15;      ///< Stop on relative cost decrease below this.
  double initial_lambda = 1e-4;       ///< LM damping relative to max(diag(J^T J)).
};

enum class SolverStatus {
  kGradientTolerance,
  kStepTolerance,
  kCostTolerance,
  kMaxIterations,
  kNumericalFailure,  ///< Normal equations could not be solved (rank deficient for GN).
};

/// Solver outcome. Costs are 0.5 |r|^2.
struct SolverResult {
  VecX x;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  int iterations = 0;
  SolverStatus status = SolverStatus::kMaxIterations;
  std::vector<double> cost_history;  ///< Cost after every accepted step (index 0 = initial).

  [[nodiscard]] bool converged() const {
    return status == SolverStatus::kGradientTolerance || status == SolverStatus::kStepTolerance
           || status == SolverStatus::kCostTolerance;
  }
};

/// @throws std::invalid_argument if the problem has no residual function.
[[nodiscard]] SolverResult gauss_newton(const LeastSquaresProblem& problem, VecX x0,
                                        const SolverOptions& options = {});

/// @throws std::invalid_argument if the problem has no residual function.
[[nodiscard]] SolverResult levenberg_marquardt(const LeastSquaresProblem& problem, VecX x0,
                                               const SolverOptions& options = {});

}  // namespace roboticscore::optimization
