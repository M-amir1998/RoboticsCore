# FastSLAM 1.0

Header: [`include/roboticscore/slam/fastslam.hpp`](../../../include/roboticscore/slam/fastslam.hpp)

## Purpose

`slam::FastSlam` solves landmark SLAM with a Rao-Blackwellised particle filter. Each
particle holds one robot path hypothesis and, conditioned on that path, an independent
$2\times2$ EKF per landmark. It uses the same inputs as `EkfSlam`: odometry commands and
range-bearing observations with known landmark IDs. The header also provides the
trajectory metrics `absolute_trajectory_error()` and `relative_pose_error()`.

## Mathematical formulation

The SLAM posterior factors as

$$p(x_{1:t}, m \mid z_{1:t}, u_{1:t}) = p(x_{1:t} \mid z_{1:t}, u_{1:t}) \prod_{j} p(m_j \mid x_{1:t}, z_{1:t})$$

The first factor is represented by $M$ particles, the product by per-particle landmark EKFs.

**Proposal.** Each particle samples the motion model with noisy controls:
$\tilde{v} \sim \mathcal{N}(v, \sigma_v^2)$ and $\tilde{\omega} \sim \mathcal{N}(\omega, \sigma_\omega^2)$,
with variances from `VelocityNoise`. The new pose is $x_t = f(x_{t-1}, \tilde{u}, \Delta t)$.

**Landmark update** for particle $k$ and landmark $j$, with $H$ the $2\times2$ Jacobian
of range-bearing with respect to the landmark:

$$y = \big(r - \hat{r},\; \mathrm{wrap}(\phi - \hat{\phi})\big), \qquad S = H \Sigma H^\top + R, \qquad K = \Sigma H^\top S^{-1}$$

$$\mu \leftarrow \mu + K y, \qquad \Sigma \leftarrow (I - KH)\Sigma(I - KH)^\top + K R K^\top, \qquad \log w^{(k)} \mathrel{+}= \log \mathcal{N}(y; 0, S)$$

A new landmark is initialised from the inverse observation model:
$\mu = (x + r\cos(\theta+\phi), y + r\sin(\theta+\phi))$ and
$\Sigma = G_z R G_z^\top$. Initialisation leaves the weight unchanged.

**Metrics.**

- ATE is $\sqrt{\tfrac{1}{T}\sum_t \lVert A\,p^{\text{est}}_t - p^{\text{true}}_t \rVert^2}$.
  $A \in SE(2)$ is the least-squares alignment of the positions when `align = true`, and the
  identity otherwise.
- RPE is the RMS translation of $(\Delta^{\text{true}}_t)^{-1}\Delta^{\text{est}}_t$, where
  $\Delta_t = X_t^{-1} X_{t+\delta}$.

## Assumptions

- Data association is known and correct.
- Static point landmarks, Gaussian range-bearing noise, and the velocity motion model.
- The motion-model proposal is adequate. This holds when odometry noise is not much
  larger than what the measurements can correct.
- Enough particles to represent path uncertainty. Loops in particular need path diversity.

## Algorithm

1. `predict(u, dt)`: for each particle, draw a noisy $(v, \omega)$ and step the unicycle.
2. `update(observations)`:
   1. For each particle and observation, initialise the landmark (no weight change) or run
      the $2\times2$ EKF update and add $\log \mathcal{N}(y; 0, S)$ to the log weight.
   2. Normalise all log weights with log-sum-exp.
   3. If $\mathrm{ESS} < \text{threshold} \cdot M$, resample systematically (particles
      copied with their maps, weights reset to $1/M$). Return whether this happened.
3. Read the estimate with `best_particle()` or `mean_pose()` (circular mean for heading).

## Implementation notes

- **Log-domain weights.** Log weights are normalised by log-sum-exp after every update, so
  many observations per step do not underflow.
- **Map storage.** Each particle's map is a `std::vector<LandmarkEstimate>` indexed
  directly by landmark ID, with an `initialized` flag. Lookup is $O(1)$, but memory per
  particle scales with the largest ID, not with the number of landmarks. Prefer compact IDs.
- **Joseph form and RNG.** Landmark covariances use the Joseph form; updates are skipped
  when the predicted range is below 1e-9. The filter owns a seeded `math::Random`.
- **Best particle after resampling.** Right after resampling all weights are equal, so
  `best_particle()` returns an arbitrary (first) particle. Use `mean_pose()` for a smoother
  pose estimate.
- **Verified by tests** (`tests/unit/slam/test_slam.cpp`):
  - with 60 particles over a 400-step circular run with non-contiguous IDs, the best
    particle's landmark RMSE is below 0.5 m, the mean pose is within 0.6 m and the ESS stays
    at least 1;
  - ATE with alignment is zero for a rigidly offset trajectory but not without alignment;
  - RPE is zero for rigid offsets.

## Complexity

From the header:

- **Per step:** $O(M K)$ for $M$ particles and $K$ observations.
- **Resampling:** an extra $O(M N)$ for map copies, because maps are arrays. Tree-based maps
  (as in the original FastSLAM) reduce this to $O(M \log N)$.
- **Memory:** $O(M N)$.

## Configuration

There is no configuration struct. The constructor is:

```cpp
FastSlam(std::size_t particle_count, const geometry::Pose2& initial_pose,
         const localization::VelocityNoise& motion_noise, double range_std, double bearing_std,
         std::uint64_t seed, double resample_threshold = 0.5);
```

| Parameter | Meaning |
|---|---|
| `particle_count` | Number of particles $M$ (> 0) |
| `initial_pose` | All particles start here (no initial pose spread) |
| `motion_noise` | `VelocityNoise` (defaults `alpha1 = 0.01`, `alpha2 = 0.001`, `alpha3 = 0.001`, `alpha4 = 0.01`, floors `1e-6`) |
| `range_std`, `bearing_std` | Measurement noise (> 0) |
| `seed` | RNG seed |
| `resample_threshold` | Resample when ESS < threshold * M; in [0, 1], default 0.5 |

The metrics are `absolute_trajectory_error(estimate, truth, align = true)` and
`relative_pose_error(estimate, truth, delta = 1)`.

## Example

```cpp
#include <vector>
#include "roboticscore/slam/fastslam.hpp"

using namespace roboticscore;

slam::FastSlam fs(/*particles=*/50, geometry::Pose2(0.0, 0.0, 0.0),
                  localization::VelocityNoise{}, 0.1, 0.02, /*seed=*/11);

fs.predict(models::UnicycleInput{1.0, 0.15}, 0.1);
const std::vector<slam::RangeBearingObservation> zs = {{7, 8.1, 0.40}};
const bool resampled = fs.update(zs);

const geometry::Pose2 pose = fs.mean_pose();
const slam::FastSlamParticle& best = fs.best_particle();
if (7 < best.landmarks.size() && best.landmarks[7].initialized) {
  const Vec2 m7 = best.landmarks[7].mean;
}
// After a run: slam::absolute_trajectory_error(estimated_poses, true_poses);
```

## Benchmark

From [docs/benchmarks/README.md](../../benchmarks/README.md): landmark SLAM over 500 steps
around a circle with 8 landmarks.

| Method | ATE [m] | Landmark RMSE [m] | Time [ms/step] |
|---|---:|---:|---:|
| FastSLAM 1.0 (50) | 0.091 | 0.090 | 0.0858 |
| EKF-SLAM | 0.060 | 0.063 | 0.0252 |

## Known limitations

- **Known data association only.** The implementation has no per-particle data
  association or landmark rejection, although that is a strength of FastSLAM in general.
- **Proposal mismatch.** FastSLAM 1.0 samples from the motion model. When measurements are
  much more accurate than odometry, most samples receive negligible weight and the ESS
  collapses. FastSLAM 2.0 includes the measurement in the proposal; it is not implemented
  here.
- **Path degeneracy.** Repeated resampling makes all particles share a common ancestry
  for older parts of the path. Landmark estimates then become overconfident, and loop
  closure gets harder over long runs.
- **Memory and copy cost.** Array maps cost $O(N)$ per particle copy, and memory scales
  with the largest landmark ID.
- **No initial spread.** Every particle starts at `initial_pose` with no initial pose
  uncertainty.

## References

- M. Montemerlo, S. Thrun, D. Koller, B. Wegbreit, "FastSLAM: A Factored Solution to the
  Simultaneous Localization and Mapping Problem", Proc. AAAI National Conference on
  Artificial Intelligence, 2002.
- M. Montemerlo, S. Thrun, D. Koller, B. Wegbreit, "FastSLAM 2.0: An Improved Particle
  Filtering Algorithm for Simultaneous Localization and Mapping that Provably Converges",
  Proc. International Joint Conference on Artificial Intelligence (IJCAI), 2003.
- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005.
