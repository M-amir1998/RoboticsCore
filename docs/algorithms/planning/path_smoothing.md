# Path smoothing: resampling, shortcutting, gradient smoothing

Header: [`include/roboticscore/planning/smoothing.hpp`](../../../include/roboticscore/planning/smoothing.hpp)

## Purpose

Post-processing of geometric polylines produced by grid or sampling planners. Raw RRT paths
are long and jagged, and grid paths hug obstacles. Three tools are provided: uniform
resampling, random shortcutting (shorter paths) and gradient-based smoothing (smoother paths
with more clearance). None of them introduces collisions into a collision-free input.

## Mathematical formulation

Gradient smoothing minimises, over the interior waypoints $x_1,\dots,x_{n-2}$ with the endpoints
fixed and $y_i$ the input path,

$$
J=w_\text{data}\sum_i\lVert x_i-y_i\rVert^2
+w_\text{smooth}\sum_i\lVert D_i\rVert^2
+w_\text{obst}\sum_i\max\big(0,\ m-c(x_i)\big)^2,
\qquad D_i=x_{i-1}-2x_i+x_{i+1},
$$

where $c(x)$ is the workspace clearance minus `robot_radius` and $m$ is `obstacle_margin`.
The gradient with respect to $x_i$ is

$$
\frac{\partial J}{\partial x_i}=2w_\text{data}(x_i-y_i)+w_\text{smooth}\big(2D_{i-1}-4D_i+2D_{i+1}\big)-2w_\text{obst}\big(m-c(x_i)\big)\nabla c(x_i),
$$

the last term only when $c(x_i)<m$, and $D_{i\pm1}$ terms only for interior neighbours. Each
waypoint moves by $\Delta=-\eta \partial J/\partial x_i$, clipped to
$\lVert\Delta\rVert\le$ `max_displacement`.

Shortcutting replaces the section between vertices $i<j$ ($j\ge i+2$) by the segment
$x_i x_j$ whenever it is collision free; by the triangle inequality the length never increases.

Roughness, used to measure smoothness, is $\sum_i\lVert D_i\rVert^2$ (`polyline_roughness`).

## Assumptions

- The input path is collision free for the same `robot_radius` (the guarantee is relative
  to the input).
- Disc robot in a static `Environment2D`; no heading or curvature constraints.
- For gradient smoothing the waypoints are dense enough that the second differences
  represent curvature; resample first.

## Algorithm

- `resample_polyline(path, spacing)`: split every segment into $\lceil\ell/\text{spacing}\rceil$
  equal pieces; all original vertices are kept.
- `shortcut_path(path, env, r, iterations, rng)`: repeat `iterations` times: draw two random
  vertex indices, and if they are at least two apart and the straight segment is free, delete
  the vertices between them.
- `gradient_smooth(path, env, params)`: for `iterations` sweeps, compute all $D_i$, then for
  each interior waypoint compute the gradient, take the clipped step, and accept the new
  position only if both adjacent segments are collision free; otherwise keep the old one.

## Implementation notes

- $\nabla c$ is computed by central differences of `Environment2D::clearance` ($h=10^{-4}$).
- Second differences are computed once per sweep; waypoints are then updated one by one, so
  each collision check uses the already updated previous neighbour.
- The per-move clip keeps the descent stable near obstacles, where the obstacle gradient is large.
- `gradient_smooth` returns a path with the same number of waypoints and identical endpoints.
  Paths with fewer than three points are returned unchanged.
- `shortcut_path` is deterministic for a given `math::Random` state.
- Verified by tests (`tests/unit/planning/test_local_planners.cpp`): resampling keeps vertices,
  length and the spacing bound; shortcutting an RRT path never increases its length and stays
  collision free; gradient smoothing of a resampled RRT path keeps the endpoints and the point
  count, lowers the roughness and keeps every segment collision free.

## Complexity

For $n$ waypoints and $M$ obstacle primitives:

- `resample_polyline`: linear in the output size.
- `shortcut_path`: $O(\text{iterations}\cdot(M+n))$ (one segment check and one erase per success).
- `gradient_smooth`: $O(\text{iterations}\cdot n\cdot M)$, with a few clearance queries and two
  segment checks per waypoint per sweep.

## Configuration

`GradientSmoothingParams`:

| Field | Default | Meaning |
|---|---|---|
| `weight_data` | `0.1` | $w_\text{data}$, attraction to the input path |
| `weight_smooth` | `1.0` | $w_\text{smooth}$, discrete curvature |
| `weight_obstacle` | `2.0` | $w_\text{obst}$ |
| `obstacle_margin` | `1.0` | $m$, clearance below which the obstacle term is active [m] |
| `robot_radius` | `0.0` | obstacle inflation for clearance and collision checks [m] |
| `step` | `0.05` | gradient-descent step $\eta$ |
| `max_displacement` | `0.02` | per-iteration move limit of one waypoint [m] |
| `iterations` | `300` | number of sweeps |

`resample_polyline` throws `std::invalid_argument` for non-positive spacing, and
`gradient_smooth` for negative weights, margin or radius, a non-positive step or
displacement, or negative iterations.

## Example

```cpp
#include "roboticscore/planning/sampling/sampling_planners.hpp"
#include "roboticscore/planning/smoothing.hpp"

using namespace roboticscore;
using namespace roboticscore::planning;

// env, start, goal and cfg (robot_radius = 0.3) as in the sampling-planner example.
const SamplingResult raw = rrt(env, start, goal, cfg);
if (raw.found) {
  math::Random rng(1);
  std::vector<Vec2> path = shortcut_path(raw.path, env, 0.3, 300, rng);
  path = resample_polyline(path, 0.25);

  GradientSmoothingParams sp;
  sp.robot_radius = 0.3;
  const std::vector<Vec2> smooth = gradient_smooth(path, env, sp);
  const double before = polyline_roughness(path);
  const double after = polyline_roughness(smooth);   // lower is smoother
  const double length = polyline_length(smooth);     // from sampling_planners.hpp
}
```

No animation is dedicated to smoothing.

## Benchmark

[docs/benchmarks/README.md](../../benchmarks/README.md) contains no measurements for path
smoothing. The minimum-clearance columns of its grid and sampling tables show the
unsmoothed paths running close to obstacles.

## Known limitations

- Gradient smoothing trades length for clearance: the obstacle term pushes waypoints away
  from obstacles and the data term holds them near the input, so the result can be longer than
  the input. Shortcutting does the opposite and leaves the path close to the inflated
  obstacles. A common sequence is shortcut, resample, smooth.
- A move that would cause a collision is rejected, not projected, so waypoints near obstacles
  can stop early. The fixed iteration count has no convergence test.
- Purely geometric: no curvature bound or heading, so the result is not guaranteed to be
  feasible for car-like robots (use [Hybrid A*](hybrid_astar.md) for that).
- Local method: it cannot change the homotopy class of the path.
- The clearance gradient is discontinuous where the nearest obstacle changes, which can make
  waypoints jitter between two obstacles.

## References

- S. M. LaValle, "Planning Algorithms", Cambridge University Press, 2006.
- D. Dolgov, S. Thrun, M. Montemerlo, J. Diebel, "Path planning for autonomous vehicles in unknown semi-structured environments", International Journal of Robotics Research, 2010 (gradient-based smoothing of the search result with obstacle and smoothness terms).
