# Unscented Kalman filter

Header: [`include/roboticscore/estimation/unscented_kalman_filter.hpp`](../../../include/roboticscore/estimation/unscented_kalman_filter.hpp)

## Purpose

`estimation::UnscentedKalmanFilter` propagates a Gaussian through nonlinear process and
measurement models using a deterministic set of $2n+1$ sigma points. It needs no Jacobians
and captures the mean and covariance to second order. It uses the same `ProcessModel`,
`MeasurementModel` and `StateSpace` types as the EKF.

## Mathematical formulation

Scaled sigma points (Wan & van der Merwe) use $\lambda = \alpha^2 (n + \kappa) - n$ and the
lower Cholesky factor $L$ of $(n+\lambda)P$:

$$\mathcal{X}_0 = \hat{x}, \qquad \mathcal{X}_i = \hat{x} + L_{:,i}, \qquad \mathcal{X}_{n+i} = \hat{x} - L_{:,i}, \qquad i = 1..n$$

$$W^m_0 = \frac{\lambda}{n+\lambda}, \qquad W^c_0 = W^m_0 + 1 - \alpha^2 + \beta, \qquad W^m_i = W^c_i = \frac{1}{2(n+\lambda)}$$

**Prediction.** Propagate $\mathcal{Y}_i = f(\mathcal{X}_i, u, \Delta t)$. Then

$$\hat{x}^- = \mathrm{mean}(\mathcal{Y}, W^m), \qquad P^- = Q + \sum_i W^c_i\, d_i d_i^\top, \qquad d_i = \mathrm{residual}(\mathcal{Y}_i, \hat{x}^-)$$

**Update.** Redraw the sigma points from $(\hat{x}^-, P^-)$ and compute
$\mathcal{Z}_i = h(\mathcal{X}_i)$ and $\hat{z} = \mathrm{mean}_z(\mathcal{Z}, W^m)$. Then

$$S = R + \sum_i W^c_i\, \delta z_i \delta z_i^\top, \qquad P_{xz} = \sum_i W^c_i\, \delta x_i \delta z_i^\top, \qquad K = P_{xz} S^{-1}$$

$$\hat{x} = \mathrm{normalize}\big(\hat{x}^- + K\, \mathrm{residual}_z(z, \hat{z})\big), \qquad P = P^- - K S K^\top$$

The measurement is rejected if $\mathrm{NIS} = y^\top S^{-1} y$ exceeds the gate.

## Assumptions

- Additive Gaussian process and measurement noise. $Q$ is evaluated at the prior mean and
  added after the transform; the state is not augmented with noise.
- The belief is well approximated by a single Gaussian.
- For angular states and measurements, the user supplies `StateSpace` and
  `MeasurementModel::residual`/`mean`. Otherwise sigma points straddling $\pm\pi$ average to
  the wrong side of the circle.

## Algorithm

1. `predict(model, u, dt)`:
   1. Compute the sigma points with `merwe_sigma_points()`, normalising each one.
   2. Pass every point through $f$ and normalise the results.
   3. Compute the weighted mean with `StateSpace::mean`, then normalise it.
   4. Accumulate the covariance from residuals, add $Q$ and symmetrise.
2. `update(model, z, gate)`:
   1. Compute new sigma points from the predicted belief.
   2. Pass them through $h$ and compute the measurement mean (`model.mean` or arithmetic).
   3. Compute $S$ and $P_{xz}$ from residuals.
   4. Factorise $S$ with Cholesky and compute the NIS. If it is above the gate, reject the
      measurement and return.
   5. Solve for $K$, then update and normalise the mean, update and symmetrise the
      covariance.

## Implementation notes

- **Robust square root.** The matrix square root is `math::cholesky_lower()`. If the plain
  factorisation fails, it retries with growing diagonal jitter, so nearly singular
  covariances still produce sigma points. A numerical test uses $P = 10^{-14} I$.
- **Default parameters.** The defaults are $\alpha = 1$, $\beta = 2$, $\kappa = 0$, which
  keeps every covariance weight non-negative. The often quoted $\alpha = 10^{-3}$ makes
  $W^m_0$ large and negative and can destroy positive definiteness for $n > 3$ in double
  precision.
- **Covariance update.** The update uses $P - K S K^\top$ followed by symmetrisation, not
  the Joseph form; the UKF has no explicit $H$.
- **Angles.** Heading wrapping enters in three places: the sigma points, the state mean
  (circular mean via `pose_mean`) and the residuals. Bearing measurements use the circular
  mean supplied by `range_bearing_model`.
- **Verified by tests** (`test_ukf_particle.cpp`):
  - sigma points reproduce the input mean and covariance;
  - the UKF matches the linear KF on a linear system;
  - the UKF is closer than the EKF to the Monte Carlo mean in the polar-to-Cartesian
    example;
  - pose localisation across the $\pm\pi$ heading discontinuity runs without spurious
    $2\pi$ spread.

## Complexity

The theoretical cost per step is $O(n^3)$ for the factorisation plus $2n+1$ evaluations of
$f$ or $h$, plus $O(n^2 (2n+1))$ for the outer products. The cost per update also includes
an $m \times m$ factorisation.

## Configuration

`SigmaPointParams`:

| Field | Default | Meaning |
|---|---:|---|
| `alpha` | 1.0 | Spread of the sigma points around the mean (must be > 0) |
| `beta` | 2.0 | Prior-distribution term in $W^c_0$ (2 is optimal for Gaussians) |
| `kappa` | 0.0 | Secondary scaling; $n + \lambda$ must stay positive |

The constructor is
`UnscentedKalmanFilter(VecX x0, const MatX& p0, SigmaPointParams params = {}, StateSpace space = {})`.
`ProcessModel::f` and `ProcessModel::noise` are required. Any `jacobian` members are
ignored. `update()` takes an optional NIS `gate`.

## Example

```cpp
#include "roboticscore/estimation/unscented_kalman_filter.hpp"
#include "roboticscore/localization/robot_models.hpp"

using namespace roboticscore;
namespace est = roboticscore::estimation;
namespace loc = roboticscore::localization;

const auto process = loc::unicycle_process_model(loc::VelocityNoise{});
const auto meas = loc::range_bearing_model(Vec2(5.0, 5.0), 0.1, 0.02);
est::UnscentedKalmanFilter ukf(Vec3(0, 0, 0), Mat3::Identity() * 0.1,
                               est::SigmaPointParams{}, loc::pose_state_space());

ukf.predict(process, Vec2(1.0, 0.1), 0.1);
const est::UpdateResult r = ukf.update(meas, Vec2(7.0, 0.7));

// Sigma points can also be computed directly:
const est::SigmaPoints sp = est::merwe_sigma_points(ukf.state(), ukf.covariance(),
                                                    ukf.params(), loc::pose_state_space());
```

## Benchmark

Source: [docs/benchmarks/README.md](../../benchmarks/README.md). The scenario is a unicycle
robot with range-bearing landmarks, 300 steps at 10 Hz, medians over 20 seeds.

| Scenario | Filter | Position RMSE [m] | Mean NEES | Mean NIS | Time [ms/step] |
|---|---|---:|---:|---:|---:|
| low noise | UKF | 0.015 | 3.05 | 1.99 | 0.0268 |
| high measurement noise | UKF | 0.060 | 2.90 | 1.99 | 0.0272 |
| sparse landmarks | UKF | 0.038 | 2.74 | 2.00 | 0.0087 |
| outlier measurement | UKF | 0.371 | 2,739.73 | 1,881.78 | 0.0275 |

`BM_UkfPredictUpdate` has a median real time of 8,597.2 ns; `BM_EkfPredictUpdate` takes
1,778.6 ns for the same work. In these scenarios the UKF reaches the same accuracy and
consistency as the EKF. A UKF advantage is expected only with stronger nonlinearity, as in
the polar-to-Cartesian unit test.

## Known limitations

- **Unimodal Gaussian.** The UKF captures second-order effects but still represents the
  belief as one Gaussian. Multimodal or strongly skewed posteriors need a particle filter.
- **Positive definiteness.** Negative $W^c_0$ (small $\alpha$) can make $P$ indefinite.
  The jittered Cholesky only masks mild cases.
- **Covariance update form.** $P - K S K^\top$ is less robust to round-off than the Joseph
  form. Symmetrisation is applied, but positive semidefiniteness is not enforced.
- **No outlier protection by default.** As the outlier scenario shows, set a gate.
- **Model calls.** The UKF costs $2n+1$ model evaluations per step and has no analytic
  shortcut, so expensive models scale poorly with $n$.

## References

- S. J. Julier, J. K. Uhlmann, "A New Extension of the Kalman Filter to Nonlinear Systems",
  Proc. SPIE AeroSense, 1997.
- S. J. Julier, J. K. Uhlmann, "Unscented Filtering and Nonlinear Estimation", Proceedings
  of the IEEE, 2004.
- E. A. Wan, R. van der Merwe, "The Unscented Kalman Filter for Nonlinear Estimation",
  IEEE Adaptive Systems for Signal Processing, Communications, and Control Symposium, 2000.
- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005.
