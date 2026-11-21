# Occupancy grid mapping

Header: [`include/roboticscore/mapping/occupancy_grid.hpp`](../../../include/roboticscore/mapping/occupancy_grid.hpp), [`raycast.hpp`](../../../include/roboticscore/mapping/raycast.hpp)

## Purpose

`mapping::OccupancyGrid` builds a probabilistic 2D occupancy map from range scans taken at
known sensor poses. It stores log-odds per cell and updates them with an inverse sensor
model. `raycast.hpp` provides the two grid traversals used in the library:

- `bresenham_line()` for map updates;
- `traverse_grid()`, an exact traversal, for line-of-sight and collision checks.

## Mathematical formulation

Each cell stores the log-odds of occupancy:

$$l = \log \frac{p}{1 - p}, \qquad p = \frac{1}{1 + e^{-l}}$$

Under the cell-independence assumption, the binary Bayes filter becomes additive:

$$l_t = \mathrm{clamp}\big(l_{t-1} + l_{\text{meas}} - l_0,\; l_{\min},\; l_{\max}\big)$$

Here $l_0 = \mathrm{logit}(p_{\text{prior}})$. The measurement term is
$l_{\text{meas}} = \mathrm{logit}(p_{\text{hit}})$ for the cell where a beam ended with a
return, and $\mathrm{logit}(p_{\text{miss}})$ for cells the beam passed through. With the
defaults, $\mathrm{logit}(0.7) \approx 0.847$, $\mathrm{logit}(0.4) \approx -0.405$ and
$l_0 = 0$.

A cell's state is occupied if $p \ge 0.65$, free if $p \le 0.35$, and unknown otherwise.
These are the default thresholds of `state()`.

## Assumptions

- Sensor poses are known. This is mapping with known poses, not SLAM.
- The world is static, and cells are independent given the poses (the standard
  approximation).
- Measurements are independent across beams and scans.
- Beams are ideal rays. Beam width and range noise are not modelled except through
  $p_{\text{hit}}$ and $p_{\text{miss}}$.

## Algorithm

`integrate_scan(sensor_pose, angles, ranges, max_range)` processes each beam $i$ as follows:

1. Skip the beam if its range is non-finite or negative (an invalid return).
2. Set `hit = r < max_range` and length $\min(r, r_{\max})$. The end point is
   $o + \text{length}\,(\cos(\theta + a_i), \sin(\theta + a_i))$.
3. Call `integrate_ray(origin, end, hit)`:
   1. Rasterise the beam with Bresenham from `world_to_cell(origin)` to `world_to_cell(end)`.
   2. Add $l_{\text{miss}} - l_0$ to every cell except the last.
   3. Add $l_{\text{hit}} - l_0$ to the last cell if `hit`, otherwise $l_{\text{miss}} - l_0$.
   4. Ignore out-of-bounds cells, clamp each update, and mark the touched cells as observed.

## Implementation notes

- **Overflow-safe conversion.** `log_odds_to_probability()` uses two branches, so `exp()`
  never overflows. Tested at $l = \pm 800$.
- **Clamping.** Clamping to $[-4, 4]$ (about $p \in [0.018, 0.982]$) keeps the map
  responsive to change and avoids saturating at exactly 0 or 1. The prior must lie inside
  the clamp interval; the constructor checks this.
- **Ray rasterisation.** Bresenham visits one cell per major-axis step. It can skip cells
  that the continuous ray only clips at a corner. That is acceptable for probabilistic
  updates, but not for safety checks. `traverse_grid()` (Amanatides & Woo) reports every
  crossed cell, including both edge neighbours at exact corner crossings, after
  Liang-Barsky clipping to the grid.
- **Observed cells.** A separate `observed_` mask records which cells were ever updated.
  `compare_to_ground_truth()` scores only observed cells, classifying with $p > 0.5$, and
  reports coverage separately.
- **`occupied_mask(threshold)`.** Unknown (prior) cells are free when
  `threshold > p_prior`. Pass a threshold at or below `p_prior` to treat unknown space as
  occupied, for conservative planning.
- **Verified by tests** (`tests/unit/mapping/test_mapping.cpp`):
  - Bresenham lines are 8-connected and minimal in all octants and stay within half a cell
    of the ideal line;
  - `traverse_grid` covers every cell found by dense sampling and handles boundary, corner
    and zero-length cases;
  - log-odds conversion and clamping;
  - free and hit cells along a ray;
  - a four-pose LiDAR scan of a room with an obstacle reaches more than 85% coverage and
    more than 90% accuracy against the rasterised ground truth.

## Complexity

- **Per beam:** $O(\max(|\Delta x|, |\Delta y|))$ cells, so a scan costs $O(B \cdot r_{\max}/\text{res})$
  for $B$ beams.
- **`traverse_grid`:** $O(\text{cells crossed})$.
- **Storage:** $O(W H)$ doubles plus one byte per cell.
- **Queries:** `probabilities()`, `occupied_mask()` and `observed_cells()` are $O(W H)$.

## Configuration

`InverseSensorModel`:

| Field | Default | Meaning |
|---|---:|---|
| `p_hit` | 0.7 | $P(\text{occ} \mid \text{beam ended in cell})$ |
| `p_miss` | 0.4 | $P(\text{occ} \mid \text{beam passed through cell})$ |
| `p_prior` | 0.5 | Prior occupancy |
| `clamp_min` | -4.0 | Lower log-odds bound |
| `clamp_max` | 4.0 | Upper log-odds bound |

`geometry::GridSpec(width, height, resolution, origin)` places the grid. Cell $(i, j)$
covers $[o_x + i\,\text{res}, o_x + (i+1)\,\text{res}) \times [o_y + j\,\text{res}, \ldots)$.
Storage is row-major in $y$.

## Example

```cpp
#include <vector>
#include "roboticscore/mapping/occupancy_grid.hpp"
#include "roboticscore/mapping/raycast.hpp"

using namespace roboticscore;

const geometry::GridSpec spec(200, 200, 0.05, Vec2(-5.0, -5.0));  // 10 m x 10 m, 5 cm cells
mapping::OccupancyGrid grid(spec);                                  // default sensor model

std::vector<double> angles = {-0.5, 0.0, 0.5};
std::vector<double> ranges = {2.0, 3.1, 8.0};                       // 8.0 = no return
grid.integrate_scan(geometry::Pose2(0.0, 0.0, 0.0), angles, ranges, /*max_range=*/8.0);

const mapping::CellState s = grid.state(spec.world_to_cell(Vec2(3.1, 0.0)));
const std::vector<std::uint8_t> occ = grid.occupied_mask(0.65);

// Exact line-of-sight cells, e.g. for a collision check:
const auto cells = mapping::traverse_grid(spec, Vec2(0.0, 0.0), Vec2(2.0, 1.0));
```

## Benchmark

No occupancy-grid or ray-casting numbers appear in
[docs/benchmarks/README.md](../../benchmarks/README.md).

## Known limitations

- **Static world, known poses.** Moving objects leave traces until clamping lets the cells
  flip back. Pose errors smear walls, since there is no scan matching inside the grid.
- **Cell independence.** Cells are updated independently, so correlated evidence (one
  surface seen by many beams) makes the map overconfident.
- **Repeated updates.** A cell crossed by several beams of the same scan is updated once
  per beam.
- **Bresenham gaps.** Bresenham can miss corner-clipped cells, so thin obstacles hit at
  grazing angles may be partly carved as free.
- **Lost no-return beams.** Beams reported as non-finite are skipped entirely. To carve
  free space for a no-return beam, report a range of at least `max_range` instead of
  infinity.
- **Fixed extent.** Updates outside the grid are silently ignored; the map does not grow.

## References

- H. Moravec, A. Elfes, "High Resolution Maps from Wide Angle Sonar", Proc. IEEE
  International Conference on Robotics and Automation, 1985.
- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005.
- J. E. Bresenham, "Algorithm for Computer Control of a Digital Plotter", IBM Systems
  Journal, 1965.
- J. Amanatides, A. Woo, "A Fast Voxel Traversal Algorithm for Ray Tracing", Eurographics,
  1987.
