# Changelog

All notable changes are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[semantic versioning](https://semver.org/).

## [Unreleased]

### Added

- **Core**
  - Modular C++20 library with CMake presets, install rules and an exported
    `RoboticsCore` package.
  - Math utilities: angle wrapping, interpolation, numerical differentiation, RK4, statistics
    and chi-square functions, and a portable seeded random generator.
  - Lie groups SO(2), SE(2), SO(3) and SE(3) with Jacobians.
  - Collision primitives, a 2D environment, grid indexing and a k-d tree.
  - Motion models: unicycle, differential drive, kinematic bicycle and Ackermann geometry,
    and a double integrator.
- **Estimation**
  - Kalman filter (Joseph form), EKF, UKF and a particle filter with four resampling schemes.
  - NEES and NIS consistency checks, and chi-square gating.
  - Localisation models.
- **Mapping**: log-odds occupancy grid mapping, Bresenham and Amanatides-Woo ray casting, and
  point-to-point and point-to-line ICP.
- **SLAM**: EKF-SLAM, FastSLAM 1.0 and pose-graph SLAM with sparse Gauss-Newton and
  Levenberg-Marquardt, plus ATE and RPE metrics.
- **Planning**
  - Grid search: BFS, Dijkstra, A*, weighted A*, Theta* and D* Lite.
  - Sampling-based: PRM, RRT, RRT-Connect, RRT* and Informed RRT*.
  - Kinodynamic: Reeds-Shepp curves and Hybrid A*.
  - Path smoothing.
  - Local planners: DWA with moving obstacles, artificial potential fields, and velocity
    obstacles / RVO.
- **Trajectories and optimisation**
  - Cubic and quintic polynomials, trapezoidal profiles, cubic splines, B-splines and
    Bezier curves.
  - Speed profiles and reference paths.
  - Gauss-Newton, Levenberg-Marquardt and an ADMM QP solver.
- **Control**: PID with anti-windup, Pure Pursuit, Stanley, LQR (discrete Riccati equation)
  and linear time-varying MPC.
- **Simulation**: a deterministic 2D simulator with LiDAR, odometry and landmark sensors, and
  scenario generators.
- **Applications**: a demo executable with 12 scenarios, integrated pipelines, scenario
  benchmarks and Google Benchmark microbenchmarks.
- **Tests**: unit, numerical, integration and regression suites.
- **Python tooling**: Python package `roboticscore_viz` for reproducible GIF rendering and
  benchmark reports.
- **Documentation**: algorithm pages, mathematics notes, ADRs and a benchmark report.
- **Integration**
  - Jenkins pipeline.
  - Experimental ROS 2 adapter package.
