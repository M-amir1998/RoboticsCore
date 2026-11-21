# Motion models

Headers: [`include/roboticscore/models/unicycle.hpp`](../../../include/roboticscore/models/unicycle.hpp), [`include/roboticscore/models/bicycle.hpp`](../../../include/roboticscore/models/bicycle.hpp), [`include/roboticscore/models/double_integrator.hpp`](../../../include/roboticscore/models/double_integrator.hpp)

## Purpose

Kinematic models shared by estimation, planning, control and simulation: the unicycle and
differential drive for mobile robots, the kinematic bicycle with Ackermann geometry and a
vehicle footprint for car-like robots, and the double integrator as the canonical linear test
system.

## Mathematical formulation

### Unicycle

State $(x, y, \theta)$, input $(v, \omega)$. For inputs held over $\Delta t$ the exact arc is
written in chord form, with $\phi = \theta + \omega\Delta t/2$:

$$
x' = x + v\Delta t\,\mathrm{sinc}(\omega\Delta t/2)\cos\phi, \quad
y' = y + v\Delta t\,\mathrm{sinc}(\omega\Delta t/2)\sin\phi, \quad
\theta' = \theta + \omega\Delta t .
$$

With $\alpha = \omega\Delta t/2$, $\sigma = \mathrm{sinc}\,\alpha$ and $\sigma' = d\,\mathrm{sinc}/d\alpha$, the
analytic Jacobians are $\partial(x', y')/\partial\theta = v\Delta t\,\sigma(-\sin\phi, \cos\phi)$,
$\partial(x', y', \theta')/\partial v = \Delta t\,\sigma(\cos\phi, \sin\phi, 0)$ and

$$
\frac{\partial(x', y', \theta')}{\partial\omega} = \Big(\tfrac{v\Delta t^2}{2}(\sigma'\cos\phi - \sigma\sin\phi),\ \tfrac{v\Delta t^2}{2}(\sigma'\sin\phi + \sigma\cos\phi),\ \Delta t\Big).
$$

### Differential drive

Wheel radius $r$, track width $b$, wheel speeds $\omega_L, \omega_R$:
$v = r(\omega_R + \omega_L)/2$, $\omega = r(\omega_R - \omega_L)/b$; inverse
$\omega_{L,R} = (v \mp \omega b/2)/r$.

### Kinematic bicycle

Rear-axle reference point, state $(x, y, \psi, v)$, input $(a, \delta)$, wheelbase $L$:

$$
\dot x = v\cos\psi, \quad \dot y = v\sin\psi, \quad \dot\psi = v\tan\delta / L, \quad \dot v = a .
$$

Curvature $\kappa = \tan\delta / L$, minimum turning radius $L / \tan\delta_{\max}$. The
affine discrete model used by MPC is $A = I + \Delta t\,A_c$, $B = \Delta t\,B_c$,
$c = \Delta t\,(f(\bar x, \bar u) - A_c\bar x - B_c\bar u)$ with the analytic Jacobians $A_c, B_c$
at $(\bar x, \bar u)$, so $A\bar x + B\bar u + c$ equals the Euler step at the linearisation point.

### Ackermann geometry

For a bicycle angle $\delta$ the rear-axle turning radius is $R = L/\tan\delta$ and the wheel
angles are $\delta_{\mathrm{left}} = \arctan\frac{L}{R - w/2}$, $\delta_{\mathrm{right}} = \arctan\frac{L}{R + w/2}$
(track width $w$). They satisfy $\cot\delta_{\mathrm{outer}} - \cot\delta_{\mathrm{inner}} = w/L$, i.e.
both wheel axes meet on the rear-axle line.

### Double integrator

State $(p, v) \in \mathbb{R}^{2d}$, input $a \in \mathbb{R}^d$, exact zero-order-hold discretisation and
continuous white-noise acceleration with spectral density $q$:

$$
A = \begin{bmatrix} I & \Delta t I \\ 0 & I \end{bmatrix}, \quad
B = \begin{bmatrix} \tfrac{\Delta t^2}{2} I \\ \Delta t I \end{bmatrix}, \quad
Q = q \begin{bmatrix} \tfrac{\Delta t^3}{3} I & \tfrac{\Delta t^2}{2} I \\ \tfrac{\Delta t^2}{2} I & \Delta t I \end{bmatrix}.
$$

## Assumptions

- Rigid bodies on a plane, no wheel or tyre slip, no load transfer.
- Inputs are piecewise constant over each step; actuators respond instantaneously.
- Bicycle: front-wheel steering with $\lvert\delta\rvert < \pi/2$; accurate at the low lateral
  accelerations of parking and urban driving, optimistic at high speed.

## Algorithm

| Function | Method |
|---|---|
| `Unicycle::step` | exact chord-form arc, $\theta$ wrapped to $[-\pi, \pi)$ |
| `Unicycle::jacobians` | analytic, valid for every $\omega$ including 0 |
| `DifferentialDrive::saturate` | scale $(v, \omega)$ by $\omega_{\max}/\max(\lvert\omega_L\rvert, \lvert\omega_R\rvert)$ if above the limit, preserving $\omega/v$ |
| `KinematicBicycle::step` | clamp $a, \delta$; one RK4 step; wrap yaw; clamp speed to $\pm v_{\max}$ |
| `KinematicBicycle::step_pose` | exact constant-speed, constant-steering arc via the unicycle |
| `KinematicBicycle::linearize` | analytic Jacobians, forward-Euler discretisation |
| `DoubleIntegrator::step` | $Ax + Bu$ (exact) |

## Implementation notes

- `sinc(a)` uses $1 - a^2/6 + a^4/120$ for $\lvert a\rvert < 10^{-4}$; its derivative uses
  $-a/3 + a^3/30$ for $\lvert a\rvert < 10^{-3}$. No special case for $\omega = 0$ is needed.
- `KinematicBicycle::derivative` uses inputs as given; only `step` clamps them.
- `VehicleFootprint::center` is the rear-axle pose times $(\ell/2 - o_r, 0)$ with length $\ell$ and
  rear overhang $o_r$; `bounding_radius` is $\tfrac12\sqrt{\ell^2 + w^2}$.
- `DoubleIntegrator::position_measurement()` returns $H = [I\ 0]$.
- Negative $\Delta t$ throws `std::invalid_argument` in the unicycle and bicycle steps.

## Complexity

All steps and Jacobians are $O(1)$ with fixed-size Eigen types, except the double integrator,
which uses dynamic matrices: $O(d^2)$ per step.

## Configuration

| Struct | Field | Default |
|---|---|---|
| `BicycleParams` | `wheelbase` / `max_steer` / `max_speed` / `max_accel` | 2.7 m / 0.6 rad / 15.0 m/s / 3.0 m/s² |
| `DifferentialDriveParams` | `wheel_radius` / `track_width` / `max_wheel_speed` | 0.05 m / 0.3 m / 20.0 rad/s |
| `AckermannGeometry` | `wheelbase` / `track_width` | 2.7 m / 1.6 m |
| `VehicleFootprint` | `length` / `width` / `rear_overhang` | 4.5 m / 1.9 m / 0.9 m |

`DoubleIntegrator(dimension, dt)` takes its parameters in the constructor.

## Example

```cpp
#include "roboticscore/models/bicycle.hpp"
#include "roboticscore/models/double_integrator.hpp"
#include "roboticscore/models/unicycle.hpp"

using namespace roboticscore;

const geometry::Pose2 p = models::Unicycle::step(geometry::Pose2(0, 0, 0), {1.0, 0.5}, 0.1);
const models::UnicycleJacobians j = models::Unicycle::jacobians(Vec3(0, 0, 0), {1.0, 0.0}, 0.1);

const models::DifferentialDrive dd({0.1, 0.5, 10.0});      // radius, track, wheel-speed limit
const models::UnicycleInput u = dd.saturate({3.0, 2.0});

const models::KinematicBicycle car(models::BicycleParams{});
models::BicycleState x(0.0, 0.0, 0.0, 2.0);
x = car.step(x, models::BicycleInput(0.5, 0.2), 0.05);
const models::BicycleLinearization lin = car.linearize(x, models::BicycleInput(0.0, 0.2), 0.1);

const models::DoubleIntegrator di(2, 0.1);
const MatX q = di.process_noise(0.5);
```

## Benchmark

There are no dedicated model benchmarks. The kinematic bicycle (wheelbase 2.7 m,
$\lvert\delta\rvert \le 0.6$ rad) is the plant of the path-tracking scenarios and the unicycle
the robot of the state-estimation scenarios in [`docs/benchmarks/README.md`](../../benchmarks/README.md).

## Known limitations

- The kinematic bicycle has no tyre slip, so it overestimates achievable lateral acceleration
  at speed; no dynamic bicycle model is provided.
- No actuator dynamics: steering angle and acceleration change instantaneously; the speed
  clamp is applied after the RK4 step rather than inside the dynamics.
- `linearize` is first order in $\Delta t$; the error grows with the prediction step.
- `AckermannGeometry` is geometry only; the bicycle model does not use individual wheel angles.
- The differential drive ignores wheel slip and wheel acceleration limits.

## References

- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge
  University Press, 2017.
- J. M. Snider, *Automatic Steering Methods for Autonomous Automobile Path Tracking*, Technical
  Report CMU-RI-TR-09-08, Robotics Institute, Carnegie Mellon University, 2009.
