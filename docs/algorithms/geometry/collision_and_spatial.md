# Collision primitives, Environment2D, GridSpec and k-d tree

Headers: [`include/roboticscore/geometry/collision.hpp`](../../../include/roboticscore/geometry/collision.hpp), [`include/roboticscore/geometry/environment.hpp`](../../../include/roboticscore/geometry/environment.hpp), [`include/roboticscore/geometry/grid.hpp`](../../../include/roboticscore/geometry/grid.hpp), [`include/roboticscore/spatial/kd_tree.hpp`](../../../include/roboticscore/spatial/kd_tree.hpp)

## Purpose

The geometric layer shared by all planners. `collision.hpp` provides exact 2D distance and
intersection tests; `Environment2D` is the single workspace model used by the continuous,
local and kinodynamic planners and by simulated range sensors, which keeps benchmark
comparisons fair; `GridSpec` defines cell indexing for every grid; `KdTree2D` answers the
neighbour queries of the sampling-based planners.

## Mathematical formulation

**Conventions.** Shapes are closed sets: boundary points count as inside or intersecting.
Distances are Euclidean and zero for intersecting shapes unless stated otherwise.

- Orientation: $o(a,b,c)=(b_x-a_x)(c_y-a_y)-(b_y-a_y)(c_x-a_x)$, twice the signed triangle
  area, $>0$ for a counter-clockwise turn.
- Closest point on segment $ab$: $a+t(b-a)$ with
  $t=\mathrm{clamp}\big((p-a)\cdot(b-a)/\lVert b-a\rVert^2,\ 0,\ 1\big)$.
- Signed distance to a polygon $P$: $\pm\min_i d(p, e_i)$ over its edges, negative inside.
- Clearance of the workspace with bounds $[x_\text{min},x_\text{max}]\times[y_\text{min},y_\text{max}]$,
  circles $(c_j,r_j)$ and polygons $P_k$:

$$
\mathrm{clearance}(p)=\min\Big(p_x-x_\text{min},\ x_\text{max}-p_x,\ p_y-y_\text{min},\ y_\text{max}-p_y,\ \min_j\big(\lVert p-c_j\rVert-r_j\big),\ \min_k \mathrm{sd}(p,P_k)\Big),
$$

  negative inside an obstacle or outside the bounds.
- A disc robot of radius $r$ is handled by inflating queries (configuration-space obstacles
  as Minkowski sums with a disc): `point_free(p, r)` is $\mathrm{clearance}(p)>r$, and
  `segment_free(a, b, r)` tests the disc swept along $ab$ exactly.

**Grid.** Cell $(i,j)$ of a `GridSpec` with origin $o$ and resolution $s$ covers
$[o_x+is,\ o_x+(i+1)s)\times[o_y+js,\ o_y+(j+1)s)$; its centre is $o+s (i+\tfrac12,\ j+\tfrac12)$
and its linear index is $j\cdot\text{width}+i$ (row-major in $y$).

**k-d tree** (Bentley 1975). Each node splits on axis $x$ or $y$ alternating with depth; every
point of the left subtree has coordinate $\le$ the node's, every point of the right subtree
$\ge$. A subtree is skipped when the lower bound $(q_a-\text{node}_a)^2$ exceeds the current
best squared distance.

## Assumptions

- Polygons are simple (non-self-intersecting); this is assumed, not checked. The constructor
  requires at least three vertices and non-zero area and stores them counter-clockwise.
- The workspace is a static axis-aligned rectangle with circles and polygons.
- Point ids in the k-d tree are insertion indices.

## Algorithm

- `segments_intersect`: orientation tests with explicit collinear-overlap handling.
- `point_in_polygon`: crossing number after an AABB rejection; points within $10^{-12}$ of an
  edge count as inside.
- `segment_polygon_distance`: 0 if an endpoint is inside, else the minimum segment-edge distance.
- `polygons_intersect`: any pair of crossing edges, else containment of one first vertex.
- `polygon_circle_intersect`, `ray_cast` against circles, segments and polygons (origin inside
  a shape gives $t=0$).
- `Environment2D::segment_free`: both endpoints inside the bounds shrunk by $r$ (convex, so the
  whole segment is), each circle at distance $>r_j+r$ from the segment, each polygon at
  distance $>r$; polygons whose AABB does not overlap the segment's inflated AABB are skipped.
- `Environment2D::polygon_free`: all footprint vertices inside the bounds and no intersection
  with any obstacle.
- `KdTree2D`: balanced build by median split (`std::nth_element`) or incremental insertion
  at a leaf; iterative depth-first queries with the pruning bound above.

## Implementation notes

- Touching counts as collision: `point_free` requires clearance strictly greater than the
  inflation, and `segment_free` rejects distances $\le$ the inflated radius.
- `clearance` skips a polygon when the distance to its bounding box already exceeds the best
  value. `Environment2D::ray_cast` returns 0 if the origin is outside the bounds and `nullopt`
  if nothing is hit within `max_range`; boundary walls count as hits.
- `GridSpec::world_to_cell` uses `floor` and may return out-of-bounds cells;
  `world_to_cell_checked` returns `nullopt` instead. `std::hash<GridIndex>` is provided.
- k-d tree ties in distance are broken by the smaller id, so results do not depend on tree
  shape. `k_nearest` and `radius_search` return results sorted by (distance, id);
  `radius_search` is inclusive and throws for a negative radius. `rebuild()` rebalances.
- Verified by tests (`tests/unit/geometry/test_collision.cpp`, `tests/unit/spatial/test_kd_tree.cpp`):
  orientation normalisation, rotated rectangles, segment intersection cases (including
  collinear), concave point-in-polygon with boundary points, signed distances, polygon overlap,
  ray casting, clearance and free-space queries, a property test that every point sampled on a
  free segment has clearance above the inflation, grid round trips; k-d tree nearest,
  k-nearest and radius results equal to brute force (with duplicates and ties, bulk and
  incremental), depth at most 11 for 1,024 points after a balanced build and 1,024 for sorted
  incremental insertion.

## Complexity

- Primitive tests: $O(1)$; polygon tests $O(n)$ for $n$ vertices; `polygons_intersect` $O(nm)$.
- `Environment2D` queries: $O(C+V)$ for $C$ circles and $V$ polygon vertices in total; there is
  no spatial index over obstacles, only AABB pruning.
- `KdTree2D`: build $O(n\log n)$; insertion $O(\text{depth})$; nearest and k-nearest
  $O(\log n)$ expected on a balanced tree; radius $O(\log n+m)$ for $m$ results. Incremental
  insertion does not rebalance: random orders give expected depth $O(\log n)$, sorted orders
  degrade towards $O(n)$.

## Configuration

These are data types without config structs. Main parameters:

| Type | Constructor / field | Default | Notes |
|---|---|---|---|
| `Circle` | `center`, `radius` | origin, `0.0` | `Environment2D` requires radius $>0$ |
| `Aabb` | `min`, `max` | origin | `expanded(margin)`, `contains`, `overlaps` |
| `Polygon` | `Polygon(vertices)`, `rectangle(center, length, width, yaw = 0)`, `from_aabb(box)` | - | throws for $<3$ vertices or zero area |
| `Environment2D` | `Environment2D(bounds)` | - | throws for empty bounds |
| `GridSpec` | `GridSpec(width, height, resolution, origin = (0, 0))` | 1 x 1, 1.0 | throws for non-positive sizes |
| `KdTree2D` | `KdTree2D()`, `KdTree2D(points)` | empty | bulk build is balanced |

## Example

```cpp
#include "roboticscore/geometry/environment.hpp"
#include "roboticscore/geometry/grid.hpp"
#include "roboticscore/spatial/kd_tree.hpp"

using namespace roboticscore;
using namespace roboticscore::geometry;

Environment2D env({{0.0, 0.0}, {10.0, 10.0}});
env.add_obstacle(Circle{{5.0, 5.0}, 1.0});
env.add_obstacle(Polygon::rectangle({2.0, 8.0}, 2.0, 2.0));
const double c = env.clearance(Vec2(5.0, 7.0));                          // 1.0
const bool seg_ok = env.segment_free(Vec2(1.0, 1.0), Vec2(9.0, 1.0), 0.4); // true
const std::optional<double> hit = env.ray_cast(Vec2(2.0, 5.0), 0.0, 20.0);  // 2.0

const GridSpec spec(40, 40, 0.25);                 // 10 m x 10 m, origin (0, 0)
const GridIndex cell = spec.world_to_cell(Vec2(3.1, 4.7));
const Vec2 centre = spec.cell_center(cell);

spatial::KdTree2D tree;
tree.insert(Vec2(1.0, 1.0));
tree.insert(Vec2(4.0, 2.0));
const std::optional<spatial::Neighbor> nn = tree.nearest(Vec2(3.0, 2.0));  // id 1
const auto near = tree.radius_search(Vec2(3.0, 2.0), 2.5);                // ids 1, 0
```

## Benchmark

From [docs/benchmarks/README.md](../../benchmarks/README.md) (Google Benchmark, 5 repetitions,
medians):

| Benchmark | Real time [ns] | CPU time [ns] |
|---|---:|---:|
| BM_KdTreeNearest/100 | 319.5 | 304.8 |
| BM_KdTreeNearest/1000 | 520.2 | 481.3 |
| BM_KdTreeNearest/10000 | 698.3 | 662.7 |
| BM_KdTreeNearest/100000 | 952.3 | 941.7 |

The empirical complexity fitted by Google Benchmark for `BM_KdTreeNearest` is lgN. The
collision primitives and `Environment2D` have no dedicated entry; the scenario tables use them
as the common collision checker.

## Known limitations

- No broad-phase structure over obstacles: query cost grows linearly with the number of
  obstacles and polygon vertices.
- Polygon simplicity is not validated; self-intersecting input gives undefined results.
- Only discs (via inflation) and polygons (via `polygon_free`) are supported as robot shapes;
  swept-volume checks exist only for discs along straight segments.
- Boundary tolerance in `point_in_polygon` is absolute ($10^{-12}$), not scaled to coordinates.
- The k-d tree is 2D only and does not support deletion; incremental insertion can become
  unbalanced until `rebuild()` is called.

## References

- J. L. Bentley, "Multidimensional binary search trees used for associative searching", Communications of the ACM, 1975.
- S. M. LaValle, "Planning Algorithms", Cambridge University Press, 2006.
