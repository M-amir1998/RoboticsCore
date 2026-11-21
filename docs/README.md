# RoboticsCore documentation

- [Getting started](getting_started.md): build, test, install, demos, animations.
- [Architecture](architecture.md): modules, dependency graph, conventions, test suites.
- [Benchmarks](benchmarks/README.md): measured results and how to reproduce them.
- [Architecture decision records](adr/README.md).

## Algorithm pages

Each page follows the same template: purpose, mathematical formulation, assumptions,
algorithm, implementation notes, complexity, configuration, example, benchmark, known
limitations and references.

| Area | Pages |
|---|---|
| Geometry | [Collision primitives, environment, grids, k-d tree](algorithms/geometry/collision_and_spatial.md) |
| Motion models | [Unicycle, differential drive, bicycle, Ackermann, double integrator](algorithms/models/motion_models.md) |
| Estimation | [Kalman filter](algorithms/estimation/kalman_filter.md) · [EKF](algorithms/estimation/ekf.md) · [UKF](algorithms/estimation/ukf.md) · [Particle filter](algorithms/estimation/particle_filter.md) |
| Mapping | [Occupancy grid and ray casting](algorithms/mapping/occupancy_grid.md) · [ICP](algorithms/mapping/icp.md) |
| SLAM | [EKF-SLAM](algorithms/slam/ekf_slam.md) · [FastSLAM](algorithms/slam/fastslam.md) · [Pose-graph SLAM](algorithms/slam/pose_graph.md) |
| Global planning | [BFS, Dijkstra, A*, Theta*](algorithms/planning/grid_search.md) · [D* Lite](algorithms/planning/dstar_lite.md) · [PRM, RRT, RRT-Connect, RRT*, Informed RRT*](algorithms/planning/sampling_planners.md) · [Hybrid A* and Reeds-Shepp](algorithms/planning/hybrid_astar.md) |
| Local planning | [DWA, potential fields, velocity obstacles](algorithms/planning/local_planners.md) · [Path smoothing](algorithms/planning/path_smoothing.md) |
| Trajectories | [Polynomials, splines, speed profiles, reference paths](algorithms/trajectory/trajectories.md) |
| Optimisation | [Gauss-Newton, Levenberg-Marquardt, ADMM QP](algorithms/optimization/least_squares_and_qp.md) |
| Control | [PID](algorithms/control/pid.md) · [Pure Pursuit and Stanley](algorithms/control/geometric_trackers.md) · [LQR](algorithms/control/lqr.md) · [MPC](algorithms/control/mpc.md) |

## Mathematics notes

- [Lie groups SO(2), SE(2), SO(3), SE(3)](mathematics/lie_groups.md)
- [Numerical methods and conventions](mathematics/numerical_methods.md)

## API reference

The public headers carry Doxygen comments. Generate HTML with

```bash
doxygen docs/Doxyfile     # output in build/docs/html
```
