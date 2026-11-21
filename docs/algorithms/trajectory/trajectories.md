# Trajectories, splines and reference paths

Headers: [`polynomial.hpp`](../../../include/roboticscore/trajectory/polynomial.hpp), [`spline.hpp`](../../../include/roboticscore/trajectory/spline.hpp), [`time_parameterization.hpp`](../../../include/roboticscore/trajectory/time_parameterization.hpp), [`reference_path.hpp`](../../../include/roboticscore/trajectory/reference_path.hpp), [`tracks.hpp`](../../../include/roboticscore/trajectory/tracks.hpp) (all in `include/roboticscore/trajectory/`)

## Purpose

Time-parameterised 1-D motions (polynomials, trapezoidal profile), planar curves (cubic
splines, B-splines, Bezier curves), speed profiles along a path, the `ReferencePath` consumed by
all path trackers, the benchmark tracks and trajectory quality metrics.

## Mathematical formulation

### Polynomials

$p(t) = \sum_k c_k t^k$ on $[0, T]$. Cubic with $p(0) = p_0, \dot p(0) = v_0, p(T) = p_1, \dot p(T) = v_1$:

$$
c_2 = \frac{3(p_1 - p_0) - (2v_0 + v_1)T}{T^2}, \qquad c_3 = \frac{2(p_0 - p_1) + (v_0 + v_1)T}{T^3}.
$$

Quintic: additionally $\ddot p(0) = a_0$, $\ddot p(T) = a_1$; $c_0..c_2$ follow directly and
$c_3..c_5$ from a $3\times 3$ linear system. Rest to rest it is the minimum-jerk profile
$p_0 + (p_1 - p_0)(10\tau^3 - 15\tau^4 + 6\tau^5)$, $\tau = t/T$.

### Trapezoidal profile

For distance $D$, limits $v_{\max}, a_{\max}$: if $v_{\max}^2 / a_{\max} > D$ the profile is
triangular with $v_{\mathrm{peak}} = \sqrt{D a_{\max}}$; otherwise $t_a = v_{\max}/a_{\max}$,
$t_c = (D - v_{\max} t_a)/v_{\max}$, total time $2t_a + t_c$.

### Cubic splines

Knot second derivatives $M_i$ solve (header notation, $h_i = x_{i+1} - x_i$)

$$
h_{i-1} M_{i-1} + 2 (h_{i-1} + h_i) M_i + h_i M_{i+1} = 6 \Big(\frac{y_{i+1} - y_i}{h_i} - \frac{y_i - y_{i-1}}{h_{i-1}}\Big)
$$

with natural ($M_0 = M_n = 0$) or clamped (end slopes prescribed) boundary rows.
`CubicSpline2D` fits $x(s), y(s)$ over cumulative chord length $s$; heading
$\mathrm{atan2}(y', x')$ and curvature $\kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^{3/2}$.

### B-splines and Bezier curves

`BSpline` of degree $p$ with $n$ control points uses the clamped uniform knot vector
($p+1$ zeros, interior knots $i/(n-p)$, $p+1$ ones) and De Boor's algorithm. Its derivative is
a degree $p-1$ B-spline with control points $Q_i = p\,(P_{i+1} - P_i)/(u_{i+p+1} - u_{i+1})$.
`BezierCurve` uses De Casteljau's algorithm; its hodograph has control points $n(P_{i+1} - P_i)$.

### Speed profile

$v_{\lim}(s) = \min(v_{\max}, \sqrt{a_{\mathrm{lat}}/\lvert\kappa(s)\rvert})$; a forward pass enforces
$v_{i+1}^2 \le v_i^2 + 2a_{\max}\Delta s$, a backward pass $v_i^2 \le v_{i+1}^2 + 2a_{\max}\Delta s$;
time stamps $\Delta t = 2\Delta s / (v_i + v_{i+1})$, accelerations $a_i = (v_{i+1}^2 - v_i^2)/(2\Delta s)$.

### Metrics

`evaluate_trajectory` differentiates time-stamped positions: segment speeds, then accelerations
and jerks at mid-times, and the discrete Menger curvature $\kappa = 2\lvert(b-a)\times(c-a)\rvert / (\lvert ab\rvert\,\lvert bc\rvert\,\lvert ca\rvert)$ of consecutive samples.

## Assumptions

- Polynomials and splines are evaluated by the caller in the intended range; polynomials and
  `CubicSpline` extrapolate outside it.
- The speed profile treats the vehicle as a point mass with decoupled tangential and lateral
  limits and symmetric acceleration/braking.
- `ReferencePath` samples are dense enough for linear interpolation; on reverse segments the
  stored heading is the vehicle yaw, not the direction of travel.

## Algorithm

| Component | Method |
|---|---|
| `Polynomial::derivative` | Horner evaluation of the differentiated coefficients |
| `CubicSpline` | Thomas algorithm for the tridiagonal system; segment by binary search |
| `ReferencePath::from_waypoints` | sample `CubicSpline2D` every `spacing` m, then re-measure $s$ along the polyline |
| `ReferencePath::at` | binary search on $s$, linear interpolation (heading on the circle), clamped to the path |
| `ReferencePath::project` | closest point over segments; with a hint only $\pm$`window` segments; sign of cross-track from the cross product with the tangent (positive left) |
| `apply_speed_profile` | `compute_speed_profile` on $\lvert\kappa\rvert$; keeps the sign of existing speeds |

Benchmark tracks (`make_track`, all start at the origin heading $+x$, constant speed):

| Name | Geometry |
|---|---|
| `straight` | 60 m line |
| `circle` | full counter-clockwise loop, radius 15 m |
| `s_curve` | 80 m, $\kappa(s) = 0.1 \sin(2\pi s/40)$ |
| `sharp_turns` | corners (0,0), (25,0), (25,20), (50,20), (50,0), (75,0) rounded with 4 m arcs |
| `variable_curvature` | 100 m, $\kappa(s) = 0.17\sin(\pi s/25)(0.25 + 0.75 s/100)$ |

Curvature-defined tracks integrate the heading with the midpoint rule.

## Implementation notes

- `CubicSpline::derivative(x, k)` supports $k = 0..3$; higher orders return 0.
- `BSpline::evaluate` clamps $u$ to $[0,1]$; `BezierCurve::evaluate` does not clamp $t$.
- `TrapezoidalProfile::position` clamps $t$ to $[0, T]$; velocity and acceleration are zero
  outside $(0, T)$. Negative distances are handled by a direction sign.
- `compute_speed_profile` guards a rest-to-rest segment with $\Delta t = 2\sqrt{\Delta s / a_{\max}}$.
- `evaluate_trajectory` needs at least four samples and strictly increasing time.

## Complexity

| Operation | Cost |
|---|---|
| polynomial, $k$-th derivative of degree $d$ | $O(dk)$ |
| cubic spline construction / evaluation | $O(n)$ / $O(\log n)$ |
| B-spline evaluation (degree $p$) | $O(n + p^2)$; `derivative` builds a new spline, $O(n)$ |
| Bezier evaluation / split (degree $n$) | $O(n^2)$ |
| speed profile, metrics | $O(n)$ |
| `ReferencePath::at` / `project` with hint / without | $O(\log n)$ / $O(w)$ / $O(n)$ |

## Configuration

These are function parameters rather than config structs: `compute_speed_profile(s, curvature,
v_max, a_max, a_lat_max, v_start = 0, v_end = 0)`, `ReferencePath::from_waypoints(waypoints,
spacing, speed = 1.0)`, `ReferencePath::project(p, hint = -1, window = 50)`,
`make_track(type, speed, spacing = 0.25)`, `BSpline(control_points, degree = 3)` and
`CubicSpline(x, y, start_slope = nullopt, end_slope = nullopt)`.

## Example

```cpp
#include "roboticscore/trajectory/polynomial.hpp"
#include "roboticscore/trajectory/reference_path.hpp"
#include "roboticscore/trajectory/spline.hpp"

using namespace roboticscore;
using namespace roboticscore::trajectory;

const Polynomial mj = quintic_trajectory(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0);  // p0 v0 a0 p1 v1 a1 T
const double jerk0 = mj.jerk(0.0);
const TrapezoidalProfile trap(0.0, 10.0, 2.0, 1.0);    // 7 s: accelerate, cruise, brake

const BSpline b({{0, 0}, {1, 2}, {3, 3}, {4, 0}, {6, 1}}, 3);
const Vec2 mid = b.evaluate(0.5);

ReferencePath path = ReferencePath::from_waypoints({{0, 0}, {10, 0}, {20, 5}}, 0.25);
const double duration = path.apply_speed_profile(5.0, 1.0, 2.0);  // v_max, a_max, a_lat_max
const PathProjection proj = path.project(Vec2(4.2, 1.5));
```

## Benchmark

There are no dedicated trajectory benchmarks. The five tracks, with a speed profile limited
to 6 m/s, 1.5 m/s² and 2 m/s² lateral, are the scenarios of the path-tracking tables in
[`docs/benchmarks/README.md`](../../benchmarks/README.md).

## Known limitations

- `CubicSpline2D` uses chord length, which only approximates arc length; heading and curvature
  are evaluated at the chord parameter. Natural ends force zero curvature at both ends.
- The Thomas algorithm does not pivot; it relies on the diagonal dominance of the spline system.
- The trapezoidal profile has unbounded jerk at phase changes; the speed profile ignores jerk,
  the friction circle and vehicle dynamics.
- `apply_speed_profile` keeps only the sign of the existing speeds and does not insert stops at
  direction changes (cusps).
- `ReferencePath` interpolates curvature linearly and the windowed projection can miss the
  closest point if the hint is far off.
- B-splines use uniform knots only and do not interpolate interior control points.

## References

- C. de Boor, *A Practical Guide to Splines*, Springer, 1978 (revised edition 2001).
- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge
  University Press, 2017.
