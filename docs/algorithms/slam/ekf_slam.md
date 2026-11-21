# EKF-SLAM

Header: [`include/roboticscore/slam/ekf_slam.hpp`](../../../include/roboticscore/slam/ekf_slam.hpp)

## Purpose

`slam::EkfSlam` jointly estimates a planar robot pose and a map of 2D point landmarks
with one extended Kalman filter. Inputs are odometry commands $(v, \omega)$ and
range-bearing observations with known landmark identities. Landmarks are added to the
state the first time they are seen.

## Mathematical formulation

The joint state and covariance for $N$ landmarks are

$$\mathbf{x} = (x, y, \theta, m_{1,x}, m_{1,y}, \ldots, m_{N,x}, m_{N,y})^\top \in \mathbb{R}^{3+2N}$$

$P$ is partitioned into the robot block $P_{rr}$ ($3\times3$), the robot-map
cross-covariance $P_{rm} = P_{mr}^\top$, and the map block $P_{mm}$.

**Prediction.** Only the robot moves (unicycle arc model, Jacobians $F$ and $V$). With
$M$ from `VelocityNoise`:

$$P_{rr} \leftarrow F P_{rr} F^\top + V M V^\top, \qquad P_{rm} \leftarrow F P_{rm}$$

**Observation** of landmark $j$, with $\delta = m_j - (x, y)$ and $q = \delta^\top\delta$:

$$h(\mathbf{x}) = \big(\sqrt{q},\; \mathrm{wrap}(\operatorname{atan2}(\delta_y, \delta_x) - \theta)\big)^\top$$

The Jacobian $H$ is zero except for the robot columns and the two columns of landmark $j$:

- robot block $H_r$: rows $(-\delta_x/\sqrt{q},\ -\delta_y/\sqrt{q},\ 0)$ and
  $(\delta_y/q,\ -\delta_x/q,\ -1)$;
- landmark block $H_{m_j}$: rows $(\delta_x/\sqrt{q},\ \delta_y/\sqrt{q})$ and
  $(-\delta_y/q,\ \delta_x/q)$.

The bearing innovation is wrapped to $[-\pi, \pi)$. The update is the Kalman update in Joseph
form, followed by heading wrap.

**Initialisation** of a new landmark from $(r, \phi)$ uses the inverse observation model:

$$m = \big(x + r\cos(\theta+\phi),\; y + r\sin(\theta+\phi)\big)^\top, \qquad P_{mm} = G_r P_{rr} G_r^\top + G_z R G_z^\top, \qquad P_{m\mathbf{x}} = G_r P_{r\mathbf{x}}$$

Here $G_r = \partial m / \partial (x, y, \theta)$ and $G_z = \partial m / \partial (r, \phi)$.

## Assumptions

- Data association is known: each observation carries the correct landmark ID.
- Landmarks are static points. Range-bearing noise is Gaussian with fixed
  $R = \mathrm{diag}(\sigma_r^2, \sigma_\phi^2)$.
- The motion noise follows the velocity model of *Probabilistic Robotics*.
- Linearisation errors stay small. In particular, heading uncertainty stays small.

## Algorithm

1. `predict(u, dt)`:
   1. Move the robot mean with the exact unicycle step.
   2. Update $P_{rr}$ and the robot-map cross-covariance rows and columns. The map block is
      untouched.
2. `update(observations)`, sequentially for each observation:
   1. If the ID is unknown, augment the state and covariance with the inverse model (full
      cross-covariance to robot and map). Record ID to slot.
   2. Otherwise, build the sparse $2 \times (3+2N)$ Jacobian $H$ and compute the wrapped
      innovation. Apply `kalman_update()`, then wrap $\theta$.
3. Return the number of newly added landmarks.

## Implementation notes

- **Arbitrary IDs.** Landmark IDs are arbitrary non-negative integers. An
  `unordered_map` maps ID to slot in order of first sight, and `landmark_ids()` returns
  that order.
- **Augmentation is exact.** A new landmark gets its full correlation with the robot and
  with every existing landmark through $G_r P_{r\mathbf{x}}$, rather than an
  independent-landmark approximation. A unit test checks that the robot-landmark
  cross-covariance is populated at initialisation.
- **Shared update.** The update reuses the generic `estimation::kalman_update()` (Cholesky
  gain, Joseph form, symmetrisation). Observations are not gated.
- **Zero range.** If the robot sits on a landmark (range below 1e-9), that correction is
  skipped because the bearing is undefined. Negative ranges throw.
- **Verified by tests** (`tests/unit/slam/test_slam.cpp`): a 500-step circular run with
  seven landmarks and non-contiguous IDs maps every landmark within 0.35 m (covariance trace
  below 0.2), ends within 0.4 m of the true pose and keeps the joint covariance positive
  definite; repeated observations shrink landmark uncertainty.

## Complexity

From the header: prediction is $O(N)$ (only the robot block and robot-map
cross-covariances change) and one landmark update is $O(N^2)$, the cost of a textbook
implementation that exploits the sparsity of $H$. Augmentation copies the covariance,
$O(N^2)$; memory is $O(N^2)$.

This implementation reuses the dense, generic `kalman_update()`. Its Joseph-form product
$(I-KH)P(I-KH)^\top$ is evaluated with dense $(3+2N)\times(3+2N)$ matrices, so the
floating-point cost per observation grows as $O(N^3)$ rather than $O(N^2)$. Measured
runtime is reported below for a small map only.

## Configuration

There is no configuration struct. The constructor is:

```cpp
EkfSlam(const geometry::Pose2& initial_pose, const Mat3& initial_covariance,
        const localization::VelocityNoise& motion_noise, double range_std, double bearing_std);
```

| Parameter | Meaning |
|---|---|
| `initial_pose`, `initial_covariance` | Prior on the robot pose (validated as a covariance) |
| `motion_noise` | `VelocityNoise`: `alpha1 = 0.01`, `alpha2 = 0.001`, `alpha3 = 0.001`, `alpha4 = 0.01`, `min_var_v = 1e-6`, `min_var_omega = 1e-6` |
| `range_std`, `bearing_std` | Measurement noise standard deviations (> 0) |

## Example

```cpp
#include <vector>
#include "roboticscore/slam/ekf_slam.hpp"

using namespace roboticscore;

slam::EkfSlam ekf_slam(geometry::Pose2(0.0, 0.0, 0.0), Mat3::Identity() * 1e-6,
                       localization::VelocityNoise{}, /*range_std=*/0.1, /*bearing_std=*/0.02);

ekf_slam.predict(models::UnicycleInput{1.0, 0.15}, 0.1);
const std::vector<slam::RangeBearingObservation> zs = {{7, 8.1, 0.40}, {2, 10.3, 1.10}};
const std::size_t added = ekf_slam.update(zs);   // both new on first sight

const geometry::Pose2 pose = ekf_slam.pose();
if (const auto m = ekf_slam.landmark(7)) {
  const Mat2 cov = *ekf_slam.landmark_covariance(7);
}
```

## Benchmark

From [docs/benchmarks/README.md](../../benchmarks/README.md): landmark SLAM over 500 steps
around a circle with 8 landmarks.

| Method | ATE [m] | Landmark RMSE [m] | Time [ms/step] |
|---|---:|---:|---:|
| EKF-SLAM | 0.060 | 0.063 | 0.0252 |
| FastSLAM 1.0 (50) | 0.091 | 0.090 | 0.0858 |

With so few landmarks the quadratic (here cubic) growth in $N$ is not visible. No scaling
benchmark over map size is reported.

## Known limitations

- **Map size.** Cost and memory are quadratic in the number of landmarks (cubic per update
  in this dense implementation), so it is practical only up to a few hundred landmarks.
- **Inconsistency.** With large heading uncertainty, linearisation errors make the filter
  overconfident. Errors then exceed the reported covariance, which is the
  well-known EKF-SLAM inconsistency.
- **Known data association only.** A wrong ID corrupts the map irreversibly. There is no
  gating, no outlier rejection and no landmark deletion.
- **Unimodal.** Ambiguous situations cannot be represented.
- **Landmark type.** Only point landmarks with range-bearing sensing are supported.

## References

- R. Smith, M. Self, P. Cheeseman, "Estimating Uncertain Spatial Relationships in
  Robotics", in *Autonomous Robot Vehicles*, Springer, 1990.
- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005.
- T. Bailey, J. Nieto, J. Guivant, M. Stevens, E. Nebot, "Consistency of the EKF-SLAM
  Algorithm", Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems,
  2006.
