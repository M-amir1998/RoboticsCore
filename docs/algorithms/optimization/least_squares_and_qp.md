# Nonlinear least squares and quadratic programming

Headers: [`include/roboticscore/optimization/least_squares.hpp`](../../../include/roboticscore/optimization/least_squares.hpp), [`include/roboticscore/optimization/qp_solver.hpp`](../../../include/roboticscore/optimization/qp_solver.hpp) · sources: [`src/optimization/`](../../../src/optimization/)

## Purpose

- `gauss_newton` and `levenberg_marquardt`: dense solvers for small nonlinear least-squares
  problems $\min_x \tfrac12\lVert r(x)\rVert^2$ (calibration, small alignment problems, tests),
  with optional manifold parameters.
- `solve_qp`: a dense convex QP solver for MPC-sized problems. It is an internal,
  dependency-free implementation of the ADMM iteration described for OSQP, with Ruiz scaling
  and adaptive step size. It is **not** the OSQP library.

## Mathematical formulation

### Gauss-Newton and Levenberg-Marquardt

With Jacobian $J = \partial r/\partial x$, gradient $g = J^\top r$ and cost $F = \tfrac12\lVert r\rVert^2$:

$$
\text{GN: } (J^\top J)\,\Delta x = -g, \qquad \text{LM: } (J^\top J + \lambda D)\,\Delta x = -g, \quad D = \mathrm{diag}(J^\top J).
$$

The update is $x \leftarrow x \oplus \Delta x$ (vector addition unless a `plus` operator is
given). LM uses the gain ratio $\rho = (F(x) - F(x \oplus \Delta x)) / L$ with predicted
reduction $L = -g^\top\Delta x - \tfrac12 \Delta x^\top J^\top J \Delta x$ and Nielsen's
damping update: if $\rho > 0$ accept, $\lambda \leftarrow \lambda \max(\tfrac13, 1 - (2\rho - 1)^3)$,
$\nu \leftarrow 2$; otherwise reject, $\lambda \leftarrow \nu\lambda$, $\nu \leftarrow 2\nu$.

### ADMM QP

$$
\min_x\ \tfrac12 x^\top P x + q^\top x \quad \text{s.t.} \quad l \le A x \le u, \qquad P \succeq 0 .
$$

With $\sigma > 0$, per-row penalties $\rho$ and relaxation $\alpha \in (0, 2)$, one iteration is

$$
\begin{aligned}
\tilde x &= (P + \sigma I + A^\top \mathrm{diag}(\rho) A)^{-1}\big(\sigma x^k - q + A^\top(\rho \circ z^k - y^k)\big), \qquad \tilde z = A\tilde x \\
x^{k+1} &= \alpha\tilde x + (1 - \alpha) x^k \\
z^{k+1} &= \Pi_{[l,u]}\big(\alpha\tilde z + (1 - \alpha) z^k + y^k / \rho\big) \\
y^{k+1} &= y^k + \rho \circ \big(\alpha\tilde z + (1 - \alpha) z^k - z^{k+1}\big)
\end{aligned}
$$

Termination (on the unscaled problem): $\lVert Ax - z\rVert_\infty \le \epsilon_{\mathrm{abs}} + \epsilon_{\mathrm{rel}}\max(\lVert Ax\rVert_\infty, \lVert z\rVert_\infty)$ and
$\lVert Px + q + A^\top y\rVert_\infty \le \epsilon_{\mathrm{abs}} + \epsilon_{\mathrm{rel}}\max(\lVert Px\rVert_\infty, \lVert A^\top y\rVert_\infty, \lVert q\rVert_\infty)$.

## Assumptions

- Least squares: $r$ is smooth; Gauss-Newton needs $J$ of full column rank near the iterates;
  problems have up to a few hundred parameters (large sparse problems such as pose graphs use
  `slam::PoseGraph`).
- QP: $P$ symmetric positive semidefinite, problem feasible and bounded; dense data.

## Algorithm

**Gauss-Newton.** Each iteration: stop if $\lVert g\rVert_\infty \le$ `gradient_tolerance`;
factorise $J^\top J$ with LDLT and return `kNumericalFailure` if it is not positive or its
smallest pivot is below $10^{-12}$ times the largest; take the full step (no line search);
stop on `step_tolerance` ($\lVert\Delta x\rVert \le \epsilon(\lVert x\rVert + \epsilon)$) or on a
relative cost change below `cost_tolerance`.

**Levenberg-Marquardt.** $\lambda_0 = \tau \max\mathrm{diag}(J^\top J)$ with $\tau$ =
`initial_lambda`; $D$ is floored at $10^{-12}\max(1, \max\mathrm{diag}(J^\top J))$ so flat
directions stay damped. Accepted steps never increase the cost; `kNumericalFailure` is returned
if $\lambda$ exceeds $10^{32}$.

**QP.**
1. Ruiz equilibration (`scaling_iterations` passes): column scaling $D_j = 1/\sqrt{\max(\lVert P_{:,j}\rVert_\infty, \lVert A_{:,j}\rVert_\infty)}$,
   row scaling $E_i = 1/\sqrt{\lVert A_{i,:}\rVert_\infty}$ (norms clamped to $[10^{-4}, 10^4]$) and
   cost scaling $c$; the solver works on $cDPD$, $cDq$, $EAD$, $El$, $Eu$ and recovers
   $x = D\bar x$, $y = E\bar y / c$.
2. Rows with $u_i - l_i < 10^{-9}$ are equalities and get $\rho_i$ = `equality_rho_scale` $\cdot\,\rho$.
3. Factorise the KKT matrix with LDLT; throw `NumericalError` if it is not positive definite.
4. Iterate ADMM; every 5th iteration test termination on the unscaled problem.
5. Adaptive $\rho$ (at iterations that are multiples of 5 and of `adaptive_rho_interval`):
   $\rho \leftarrow \rho\sqrt{\hat r_{\mathrm{prim}} / \hat r_{\mathrm{dual}}}$ from normalised scaled residuals,
   clamped to $[10^{-6}, 10^6]$; refactorise only if $\rho$ changes by more than a factor 5.

## Implementation notes

- Without a user Jacobian, `math::numerical_jacobian` (central differences) is used. It perturbs
  $x$ additively even when a `plus` operator is supplied.
- `SolverResult::cost_history` holds the cost after every accepted step (index 0 = initial);
  for LM `iterations` also counts rejected trials.
- QP warm start: `warm_x`, `warm_y` are scaled into the equilibrated problem; $z$ starts at
  $\Pi_{[l,u]}(Ax)$. A warm start at the solution converges within a few iterations (the
  property test allows at most 10).
- `QpResult` reports `x`, `y` (positive at active upper bounds), `z`, residuals, `objective`
  and `status` (`kSolved` or `kMaxIterations`); $m = 0$ (unconstrained) is allowed.

## Complexity

Least squares, per iteration: $O(mn^2 + n^3)$ for $J^\top J$ and LDLT, plus $2n$ residual
evaluations for a numerical Jacobian. QP: $O(n^3 + mn^2)$ per (re)factorisation,
$O(n^2 + mn)$ per ADMM iteration, $O(k(n^2 + mn))$ for $k$ Ruiz passes.

## Configuration

| `SolverOptions` field | Default |
|---|---|
| `max_iterations` | `100` |
| `gradient_tolerance` | `1e-10` |
| `step_tolerance` | `1e-12` |
| `cost_tolerance` | `1e-15` |
| `initial_lambda` | `1e-4` |

| `QpSettings` field | Default |
|---|---|
| `rho` / `sigma` / `alpha` | `0.1` / `1e-6` / `1.6` |
| `eps_abs` / `eps_rel` | `1e-6` / `1e-6` |
| `max_iterations` | `4000` |
| `equality_rho_scale` | `1e3` |
| `adaptive_rho` / `adaptive_rho_interval` | `true` / `25` |
| `scaling_iterations` | `10` (0 disables scaling) |

The MPC tracker overrides `eps_abs = eps_rel = 1e-4` and `max_iterations = 400`.

## Example

```cpp
#include "roboticscore/optimization/least_squares.hpp"
#include "roboticscore/optimization/qp_solver.hpp"

using namespace roboticscore;
using namespace roboticscore::optimization;

LeastSquaresProblem rosen;                       // Rosenbrock as residuals
rosen.residual = [](const VecX& x) -> VecX { return Vec2(10.0 * (x(1) - x(0) * x(0)), 1.0 - x(0)); };
const SolverResult lm = levenberg_marquardt(rosen, Vec2(-1.2, 1.0));  // numerical Jacobian

// min (x - 3)^2 + (y + 1)^2  s.t.  0 <= x <= 1,  x + y = 0.5
QpProblem qp;
qp.p = 2.0 * MatX::Identity(2, 2);
qp.q = Vec2(-6.0, 2.0);
qp.a = (MatX(2, 2) << 1, 0, 1, 1).finished();
qp.l = Vec2(0.0, 0.5);
qp.u = Vec2(1.0, 0.5);
const QpResult r = solve_qp(qp);                 // r.solved(), r.x ~ (1, -0.5)
```

## Benchmark

There are no dedicated least-squares or QP benchmarks. The QP is part of every MPC step in the
path-tracking tables of [`docs/benchmarks/README.md`](../../benchmarks/README.md).

## Known limitations

- Gauss-Newton fails with `kNumericalFailure` on rank-deficient $J$ and, without line search,
  can diverge from poor starting points; use LM there.
- Both least-squares solvers are dense ($O(n^3)$ per iteration), have no robust loss functions
  and no bound constraints.
- ADMM is a first-order method: accuracy beyond about $10^{-6}$ needs many iterations, and the
  iteration count depends on problem conditioning; the result at the iteration limit is returned
  with `kMaxIterations`.
- No infeasibility or unboundedness detection, no solution polishing, no sparse factorisation.
- An indefinite $P$ is detected only through the KKT factorisation (`NumericalError`).

## References

- K. Madsen, H. B. Nielsen and O. Tingleff, *Methods for Non-Linear Least Squares Problems*,
  2nd ed., Informatics and Mathematical Modelling, Technical University of Denmark, 2004.
- J. Nocedal and S. J. Wright, *Numerical Optimization*, 2nd ed., Springer, 2006.
- B. Stellato, G. Banjac, P. Goulart, A. Bemporad and S. Boyd, "OSQP: an operator splitting
  solver for quadratic programs", *Mathematical Programming Computation*, 2020.
- S. Boyd, N. Parikh, E. Chu, B. Peleato and J. Eckstein, "Distributed Optimization and
  Statistical Learning via the Alternating Direction Method of Multipliers", *Foundations and
  Trends in Machine Learning*, 2011.
