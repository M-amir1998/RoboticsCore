# PID controller

Header: [`include/roboticscore/control/pid.hpp`](../../../include/roboticscore/control/pid.hpp) · source: [`src/control/pid.cpp`](../../../src/control/pid.cpp)

## Purpose

`PidController` is a discrete single-input single-output PID controller with output limits,
two anti-windup schemes and a low-pass filtered derivative. It is the longitudinal (speed)
loop of the Pure Pursuit, Stanley and LQR path trackers, and a general building block for
any scalar loop.

## Mathematical formulation

With setpoint $r_k$, measurement $y_k$, sample period $\Delta t$ and error $e_k = r_k - y_k$:

$$
u_k = K_p e_k + I_k + D_k, \qquad u^{\mathrm{sat}}_k = \mathrm{clamp}(u_k,\ u_{\min},\ u_{\max}).
$$

The integral state stores $K_i \int e\,dt$ (not $\int e\,dt$), updated with a rectangle rule:
$I_k = I_{k-1} + K_i e_k \Delta t$.

The derivative acts on the measurement, not on the error, so a setpoint step does not
produce a derivative kick. It is filtered by a first-order low-pass with time constant
$\tau$:

$$
D_k = a D_{k-1} - K_d (1 - a) \frac{y_k - y_{k-1}}{\Delta t}, \qquad a = \frac{\tau}{\tau + \Delta t}.
$$

$\tau = 0$ gives $a = 0$, i.e. the unfiltered backward difference.

Anti-windup variants (`AntiWindup`):

| Mode | Integral update when the output saturates |
|---|---|
| `kNone` | $I_k = I_{k-1} + K_i e_k \Delta t$ always |
| `kClamping` | conditional integration: if $u_k \ne u^{\mathrm{sat}}_k$ and $e_k (u_k - u^{\mathrm{sat}}_k) > 0$, keep $I_k = I_{k-1}$ and recompute the output |
| `kBackCalculation` | $I_k = I_{k-1} + K_i e_k \Delta t + K_t (u^{\mathrm{sat}}_k - u_k) \Delta t$, the discrete form of $\dot I = K_i e + K_t (u^{\mathrm{sat}} - u)$ |

## Assumptions

- The sample period $\Delta t > 0$ is supplied on every call; it may vary between calls.
- Gains are non-negative (direct-acting loop: a positive error increases the output).
- The measurement is reasonably smooth; noise is attenuated only by the first-order
  derivative filter.

## Algorithm

One call of `update(setpoint, measurement, dt)`:

1. $e = r - y$, $P = K_p e$.
2. If a previous measurement exists and $K_d > 0$, update the filtered derivative $D$.
   On the first call after construction or `reset()` the derivative stays at its previous
   value (zero).
3. Candidate integral $I' = I + K_i e \Delta t$, unsaturated output $u = P + I' + D$,
   saturated output $u^{\mathrm{sat}}$.
4. Apply the anti-windup rule from the table above and store the integral.
5. Record the terms in `last_terms()` and return $u^{\mathrm{sat}}$.

## Implementation notes

- With back-calculation the correction $K_t (u^{\mathrm{sat}} - u) \Delta t$ enters the stored
  integral, so it acts from the next sample on. The returned output of the current sample is
  the saturated value of the uncorrected sum.
- The clamping test $e (u - u^{\mathrm{sat}}) > 0$ freezes the integrator only when the error
  would push the output further into the active limit. Integration resumes as soon as the
  error changes sign.
- `PidTerms` (`proportional`, `integral`, `derivative`, `unsaturated`, `output`) exposes the
  breakdown of the last update for logging and tests.
- `reset()` clears the integral, the derivative state and the stored measurement.
- Invalid configuration (negative gains, `output_min > output_max`, negative $\tau$ or $K_t$)
  throws `std::invalid_argument` in the constructor; `dt <= 0` throws in `update`.

## Complexity

$O(1)$ time and memory per update, no allocation.

## Configuration

`PidConfig` (all fields public, aggregate-initialisable in this order):

| Field | Default | Meaning |
|---|---|---|
| `kp` | `1.0` | proportional gain $K_p$ |
| `ki` | `0.0` | integral gain $K_i$ |
| `kd` | `0.0` | derivative gain $K_d$ |
| `output_min` | `-infinity` | lower output limit |
| `output_max` | `+infinity` | upper output limit |
| `derivative_filter_tau` | `0.0` | derivative filter time constant $\tau$ [s] |
| `anti_windup` | `AntiWindup::kClamping` | anti-windup scheme |
| `back_calculation_gain` | `1.0` | tracking gain $K_t$ [1/s] |

The Pure Pursuit, Stanley and LQR trackers use
`PidConfig{1.5, 0.3, 0.0, -3.0, 3.0, 0.0, AntiWindup::kClamping, 1.0}` as their default speed
loop (MPC computes acceleration in its QP instead).

## Example

PI control of a first-order plant $\dot y = -y + u$ with a saturated actuator:

```cpp
#include "roboticscore/control/pid.hpp"

using roboticscore::control::AntiWindup;
using roboticscore::control::PidConfig;
using roboticscore::control::PidController;

PidConfig cfg{2.0, 3.0, 0.0, -1.0, 1.0};      // kp, ki, kd, output_min, output_max
cfg.anti_windup = AntiWindup::kBackCalculation;
cfg.back_calculation_gain = 2.0;
PidController pid(cfg);

double y = 0.0;
const double dt = 0.01;
for (int k = 0; k < 1000; ++k) {
  const double u = pid.update(/*setpoint=*/0.5, /*measurement=*/y, dt);
  y += dt * (-y + u);                          // explicit Euler plant step
}
const double integral = pid.last_terms().integral;
```

## Benchmark

There is no dedicated PID benchmark. The speed loop is part of the Pure Pursuit, Stanley and
LQR steps reported in [`docs/benchmarks/README.md`](../../benchmarks/README.md).

## Known limitations

- No setpoint weighting on the proportional term and no feedforward input; feedforward
  has to be added to the returned output by the caller.
- Gains are fixed at construction (there is no setter); construct a new controller to retune.
- The derivative uses a first-order backward difference and a first-order filter; with
  $\tau = 0$ measurement noise is amplified by $K_d / \Delta t$.
- Only non-negative gains are accepted; a reverse-acting loop must swap the roles of
  setpoint and measurement or negate the output.
- Integration uses the rectangle rule; for coarse sampling the integral term lags the
  continuous-time controller.

## References

- K. J. Åström and R. M. Murray, *Feedback Systems: An Introduction for Scientists and
  Engineers*, 2nd ed., Princeton University Press, 2021.
