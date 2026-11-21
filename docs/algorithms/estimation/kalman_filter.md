# Linear Kalman filter

Header: [`include/roboticscore/estimation/kalman_filter.hpp`](../../../include/roboticscore/estimation/kalman_filter.hpp), [`models.hpp`](../../../include/roboticscore/estimation/models.hpp), [`consistency.hpp`](../../../include/roboticscore/estimation/consistency.hpp)

## Purpose

`estimation::KalmanFilter` is the minimum-variance estimator for linear systems with
Gaussian noise. The same update routine, `kalman_update()`, also serves the EKF and the
EKF-SLAM back end. `consistency.hpp` provides NEES and chi-square acceptance intervals for
checking any of the Gaussian filters.

## Mathematical formulation

The system model is

$$x_k = A x_{k-1} + B u_k + w_k, \quad w_k \sim \mathcal{N}(0, Q), \qquad z_k = H x_k + v_k, \quad v_k \sim \mathcal{N}(0, R)$$

Prediction: $\hat{x}^- = A \hat{x} + B u$ and $P^- = A P A^\top + Q$.

Update, with innovation $y = z - H \hat{x}^-$:

$$S = H P^- H^\top + R, \qquad K = P^- H^\top S^{-1}, \qquad \hat{x} = \hat{x}^- + K y$$

The covariance update uses the Joseph form:

$$P = (I - K H) P^- (I - K H)^\top + K R K^\top$$

This form stays symmetric positive semidefinite under round-off and for suboptimal gains.

The normalised innovation squared is $\mathrm{NIS} = y^\top S^{-1} y \sim \chi^2_m$. If a gate
$\gamma$ is given and $\mathrm{NIS} > \gamma$, the measurement is rejected.

The normalised estimation error squared is $\mathrm{NEES}_k = e_k^\top P_k^{-1} e_k$ with
$e_k = x_k - \hat{x}_k$. For a consistent filter it is $\chi^2_n$ distributed. Averaged over
$N$ independent runs, $N \cdot \overline{\mathrm{NEES}} \sim \chi^2_{Nn}$, which gives the
two-sided interval

$$\left[ \tfrac{1}{N} \chi^2_{Nn}\!\left(\tfrac{1-c}{2}\right), \; \tfrac{1}{N} \chi^2_{Nn}\!\left(\tfrac{1+c}{2}\right) \right]$$

for confidence $c$.

## Assumptions

- Linear dynamics and measurements; additive, white, zero-mean Gaussian noise that is
  uncorrelated with the state.
- $Q$, $R$ and $P_0$ are known, symmetric and positive semidefinite. $S$ must be positive
  definite.
- Euclidean state space: angles are not wrapped. Use the EKF or UKF with a `StateSpace` for
  headings.

## Algorithm

1. `predict(A, Q)` or `predict(A, B, u, Q)`: propagate the mean and covariance, then
   symmetrise $P$.
2. `update(z, H, R, gate)`:
   1. Compute $y = z - H\hat{x}$ and $S = H P H^\top + R$, then symmetrise $S$.
   2. Factorise $S$ with Cholesky (LLT). If that fails, throw `NumericalError`.
   3. Compute $\mathrm{NIS} = y^\top S^{-1} y$. If it exceeds the gate, return
      `accepted = false` and leave the state unchanged.
   4. Solve $S K^\top = H P$ for $K$.
   5. Update $\hat{x}$, then $P$ in Joseph form, then symmetrise $P$.
3. Return `UpdateResult{accepted, {innovation, S, nis}}`.

## Implementation notes

- **Gain without an explicit inverse.** The gain is computed by a Cholesky solve rather than
  by inverting $S$. The same factorisation gives the NIS.
- **Covariance hygiene.** Every covariance-producing step finishes with explicit
  symmetrisation, $P \leftarrow (P + P^\top)/2$. A numerical test
  (`tests/numerical/test_estimation_control_numerics.cpp`) runs 20,000 steps with zero
  process noise and a very small $R$ and checks that $P$ stays finite, symmetric and PSD.
- **Input validation.** The constructor and `reset()` call `validate_covariance()`. It
  checks dimension, finiteness and symmetry (tolerance 1e-9), and PSD via the minimum
  eigenvalue, with a tolerance relative to the matrix scale.
- **Shared update.** `kalman_update(x, P, y, H, R, gate)` takes a precomputed innovation, so
  the EKF and EKF-SLAM can apply wrapped residuals before reusing it.
- **Verified by tests** (`tests/unit/estimation/test_kalman_filters.cpp`):
  - a scalar predict/update checked against hand computation (gain and NIS);
  - control input handling;
  - zero measurement noise snapping the state to the measurement;
  - outlier gating;
  - invalid-input exceptions;
  - Monte Carlo NEES over 200 runs lying inside the 99% `average_chi2_bounds()` interval;
  - chi-square bounds matching tabulated quantiles.

## Complexity

The header states $O(n^3 + m^3)$ per step for state dimension $n$ and measurement dimension
$m$. That cost comes from the dense products in prediction and in the Joseph form, plus the
$m \times m$ Cholesky factorisation. Memory is $O(n^2)$.

## Configuration

There is no configuration struct. The parameters are:

| Parameter | Meaning |
|---|---|
| `x0`, `p0` | Initial mean and covariance (validated) |
| `a`, `b`, `u`, `q` | Transition matrix, control matrix, control input, process noise |
| `z`, `h`, `r` | Measurement, measurement matrix, measurement noise |
| `gate` | NIS threshold; default `kNoGate` (infinity, no gating) |
| `average_chi2_bounds(dof, samples, confidence = 0.95)` | Acceptance interval for averaged NEES/NIS |

## Example

```cpp
#include "roboticscore/estimation/consistency.hpp"
#include "roboticscore/estimation/kalman_filter.hpp"
#include "roboticscore/math/statistics.hpp"

using namespace roboticscore;
using namespace roboticscore::estimation;

const double dt = 0.1;
MatX a(2, 2); a << 1.0, dt, 0.0, 1.0;                     // constant velocity
MatX q(2, 2); q << dt * dt * dt / 3, dt * dt / 2, dt * dt / 2, dt;
q *= 0.1;
MatX h(1, 2); h << 1.0, 0.0;                              // position measurement
const MatX r = MatX::Constant(1, 1, 0.05);

KalmanFilter kf(VecX::Zero(2), MatX::Identity(2, 2));
kf.predict(a, q);
const double gate = math::chi_squared_quantile(0.99, 1.0); // 99% gate, 1 dof
const UpdateResult res = kf.update(VecX::Constant(1, 0.12), h, r, gate);
if (!res.accepted) { /* outlier: estimate unchanged */ }

// Consistency check over N Monte Carlo runs of a 2-state filter:
const Chi2Interval bounds = average_chi2_bounds(2, 200, 0.99);
// bounds.contains(mean_nees) should hold for a consistent filter.
```

## Benchmark

No benchmark in [docs/benchmarks/README.md](../../benchmarks/README.md) covers the linear
Kalman filter on its own. For the nonlinear filters built on the same update, see the EKF
page.

## Known limitations

- **Model mismatch.** Optimality holds only for the linear Gaussian model. With a wrong
  $Q$ or $R$ the reported covariance no longer matches the true error. NEES/NIS tests are
  the tool to detect this.
- **Hard gating.** The NIS gate is a hard accept/reject rule. A correct measurement after a
  long prediction gap can be rejected, and nothing inflates $P$ to recover.
- **Dense cost.** All matrices are dense. Large sparse problems pay the full $O(n^3)$.
- **No square-root or information form.** There is no square-root or information-form
  variant for extremely ill-conditioned problems. Joseph form plus symmetrisation is the
  only safeguard.

## References

- R. E. Kalman, "A New Approach to Linear Filtering and Prediction Problems", Transactions
  of the ASME, Journal of Basic Engineering, 1960.
- Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and
  Navigation*, Wiley, 2001.
- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005.
