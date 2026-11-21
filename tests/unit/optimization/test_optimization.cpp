#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/optimization/least_squares.hpp"
#include "roboticscore/optimization/qp_solver.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::optimization {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

LeastSquaresProblem rosenbrock() {
  LeastSquaresProblem p;
  p.residual = [](const VecX& x) -> VecX { return Vec2(10.0 * (x(1) - x(0) * x(0)), 1.0 - x(0)); };
  p.jacobian = [](const VecX& x) -> MatX {
    Mat2 j;
    j << -20.0 * x(0), 10.0, -1.0, 0.0;
    return j;
  };
  return p;
}

TEST(LeastSquares, GaussNewtonAndLmSolveRosenbrock) {
  const auto p = rosenbrock();
  const SolverResult gn = gauss_newton(p, Vec2(-1.2, 1.0));
  EXPECT_TRUE(gn.converged());
  EXPECT_MATRIX_NEAR(gn.x, Vec2(1, 1), 1e-8);
  const SolverResult lm = levenberg_marquardt(p, Vec2(-1.2, 1.0));
  EXPECT_TRUE(lm.converged());
  EXPECT_MATRIX_NEAR(lm.x, Vec2(1, 1), 1e-8);
  // LM accepted steps never increase the cost.
  for (std::size_t i = 1; i < lm.cost_history.size(); ++i) {
    EXPECT_LE(lm.cost_history[i], lm.cost_history[i - 1]);
  }
}

TEST(LeastSquares, ExponentialFitWithNumericalJacobian) {
  // y = a exp(b t) + c, noise free.
  const double a = 2.0;
  const double b = -0.7;
  const double c = 0.3;
  std::vector<double> t;
  std::vector<double> y;
  for (int i = 0; i < 30; ++i) {
    t.push_back(0.2 * i);
    y.push_back(a * std::exp(b * t.back()) + c);
  }
  LeastSquaresProblem p;
  p.residual = [&](const VecX& x) -> VecX {
    VecX r(static_cast<Index>(t.size()));
    for (std::size_t i = 0; i < t.size(); ++i) {
      r(static_cast<Index>(i)) = x(0) * std::exp(x(1) * t[i]) + x(2) - y[i];
    }
    return r;
  };
  const SolverResult r = levenberg_marquardt(p, Vec3(1.0, -0.1, 0.0));
  EXPECT_TRUE(r.converged());
  EXPECT_MATRIX_NEAR(r.x, Vec3(a, b, c), 1e-6);
  EXPECT_LT(r.final_cost, 1e-14);
}

TEST(LeastSquares, RankDeficiencyAndManifoldPlus) {
  // r = x0 + x1 - 1: J^T J is singular, plain Gauss-Newton cannot solve it.
  LeastSquaresProblem p;
  p.residual = [](const VecX& x) -> VecX { return VecX::Constant(1, x(0) + x(1) - 1.0); };
  EXPECT_EQ(gauss_newton(p, Vec2(0, 0)).status, SolverStatus::kNumericalFailure);
  const SolverResult lm = levenberg_marquardt(p, Vec2(0, 0));
  EXPECT_NEAR(lm.x(0) + lm.x(1), 1.0, 1e-8);

  // Fit an angle to wrapped observations near +-pi using a wrapping plus operator.
  const std::vector<double> obs = {math::kPi - 0.05, -math::kPi + 0.03, math::kPi - 0.01};
  LeastSquaresProblem angle;
  angle.residual = [&](const VecX& x) -> VecX {
    VecX r(3);
    for (int i = 0; i < 3; ++i)
      r(i) = math::angle_difference(x(0), obs[static_cast<std::size_t>(i)]);
    return r;
  };
  angle.plus = [](const VecX& x, const VecX& dx) -> VecX {
    return VecX::Constant(1, math::wrap_to_pi(x(0) + dx(0)));
  };
  const SolverResult ar = gauss_newton(angle, VecX::Constant(1, 3.0));
  EXPECT_TRUE(ar.converged());
  EXPECT_NEAR(math::angle_difference(ar.x(0), math::kPi - 0.01), 0.0, 1e-9);
}

// ------------------------------------------------------------------------------------ QP

// Checks first-order optimality of a convex QP solution within tolerance.
void expect_kkt(const QpProblem& q, const QpResult& r, double tol) {
  ASSERT_TRUE(r.solved());
  const VecX ax = q.a * r.x;
  for (Index i = 0; i < ax.size(); ++i) {
    EXPECT_GE(ax(i), q.l(i) - tol);
    EXPECT_LE(ax(i), q.u(i) + tol);
    // Complementarity: positive multipliers only at active upper bounds, negative at lower.
    if (r.y(i) > tol) {
      EXPECT_NEAR(ax(i), q.u(i), 10 * tol);
    }
    if (r.y(i) < -tol) {
      EXPECT_NEAR(ax(i), q.l(i), 10 * tol);
    }
  }
  EXPECT_LT((q.p * r.x + q.q + q.a.transpose() * r.y).lpNorm<Eigen::Infinity>(), 10 * tol);
}

TEST(QpSolver, UnconstrainedMatchesLinearSolve) {
  QpProblem q;
  q.p = (Mat2() << 4, 1, 1, 2).finished();
  q.q = Vec2(1, 1);
  q.a = MatX(0, 2);
  q.l = VecX(0);
  q.u = VecX(0);
  const QpResult r = solve_qp(q);
  ASSERT_TRUE(r.solved());
  EXPECT_MATRIX_NEAR(r.x, VecX(q.p.ldlt().solve(-q.q)), 1e-5);
}

TEST(QpSolver, BoxAndEqualityConstraints) {
  // min (x - 3)^2 + (y + 1)^2  s.t.  0 <= x <= 1,  x + y = 0.5
  QpProblem q;
  q.p = 2.0 * MatX::Identity(2, 2);
  q.q = Vec2(-6, 2);
  q.a = (MatX(2, 2) << 1, 0, 1, 1).finished();
  q.l = Vec2(0.0, 0.5);
  q.u = Vec2(1.0, 0.5);
  const QpResult r = solve_qp(q);
  expect_kkt(q, r, 1e-4);
  EXPECT_MATRIX_NEAR(r.x, Vec2(1.0, -0.5), 1e-4);
  QpProblem bad = q;
  bad.l(0) = 2.0;
  EXPECT_THROW((void)solve_qp(bad), std::invalid_argument);
}

TEST(QpSolverProperty, RandomBoxConstrainedQpsSatisfyKkt) {
  math::Random rng(testing::kPropertySeed);
  for (int trial = 0; trial < 40; ++trial) {
    const Index n = 4;
    MatX m(n, n);
    for (Index i = 0; i < n; ++i)
      for (Index j = 0; j < n; ++j)
        m(i, j) = rng.normal();
    QpProblem q;
    q.p = m * m.transpose() + 0.1 * MatX::Identity(n, n);
    q.q = rng.standard_normal(n) * 3.0;
    // Box bounds plus one general inequality row.
    q.a = MatX::Zero(n + 1, n);
    q.a.topRows(n) = MatX::Identity(n, n);
    q.a.row(n) = rng.standard_normal(n).transpose();
    q.l = VecX::Constant(n + 1, -1.0);
    q.u = VecX::Constant(n + 1, 1.0);
    q.u(n) = kInf;
    QpSettings s;
    s.eps_abs = 1e-7;
    s.eps_rel = 1e-7;
    s.max_iterations = 20000;
    const QpResult r = solve_qp(q, s);
    SCOPED_TRACE(trial);
    expect_kkt(q, r, 1e-4);
    // Warm start from the solution converges immediately.
    const QpResult w = solve_qp(q, s, &r.x, &r.y);
    EXPECT_LE(w.iterations, 10);
  }
}

TEST(QpSolver, RejectsIndefiniteHessian) {
  QpProblem q;
  q.p = (Mat2() << 1, 0, 0, -5).finished();
  q.q = Vec2::Zero();
  q.a = MatX(0, 2);
  q.l = VecX(0);
  q.u = VecX(0);
  EXPECT_THROW((void)solve_qp(q), NumericalError);
}

}  // namespace
}  // namespace roboticscore::optimization
