# Benchmarks

All numbers below were measured by `roboticscore_bench` and `roboticscore_microbenchmarks` on
the machine described here. Timings are wall-clock medians and depend on the hardware;
path lengths, success rates and errors are deterministic for the given seeds.

| Machine | |
|---|---|
| cpu | 13th Gen Intel(R) Core(TM) i5-1335U |
| logical cores | 12 |
| os | Windows 11 |
| compiler | MSVC 195035729 |
| build type | Release |
| library version | 0.1.0 |
| commit | ebfb3ec |

Reproduce with `python scripts/benchmark/run_benchmarks.py` (Release build required).
Fairness: every algorithm in a table uses the same map, start, goal, collision checker,
seeds and machine. Sampling-planner rows aggregate 20 seeds (medians over successful runs).

## Grid planners

8-connected, 0.25 m cells, obstacles inflated by 0.3 m, no corner cutting.

**open space**

| Planner | Success | Time [ms] | Path length [m] | Expanded nodes | Min clearance [m] |
|---|---:|---:|---:|---:|---:|
| BFS | yes | 0.159 | 22.63 | 5,184 | 1.88 |
| Dijkstra | yes | 0.427 | 22.63 | 5,940 | 1.88 |
| A* | yes | 0.051 | 22.63 | 65 | 1.88 |
| Weighted A* (w=2) | yes | 0.042 | 22.63 | 65 | 1.88 |
| Theta* | yes | 0.090 | 22.63 | 65 | 1.88 |

**cluttered**

| Planner | Success | Time [ms] | Path length [m] | Expanded nodes | Min clearance [m] |
|---|---:|---:|---:|---:|---:|
| BFS | yes | 0.229 | 35.40 | 6,476 | 0.34 |
| Dijkstra | yes | 0.551 | 34.30 | 6,490 | 0.29 |
| A* | yes | 0.377 | 34.30 | 1,520 | 0.29 |
| Weighted A* (w=2) | yes | 0.080 | 37.68 | 243 | 0.35 |
| Theta* | yes | 0.565 | 32.73 | 1,449 | 0.29 |

**narrow passage**

| Planner | Success | Time [ms] | Path length [m] | Expanded nodes | Min clearance [m] |
|---|---:|---:|---:|---:|---:|
| BFS | yes | 0.102 | 17.31 | 2,856 | 0.47 |
| Dijkstra | yes | 0.245 | 17.31 | 2,949 | 0.47 |
| A* | yes | 0.063 | 17.31 | 182 | 0.47 |
| Weighted A* (w=2) | yes | 0.058 | 18.14 | 112 | 0.47 |
| Theta* | yes | 0.119 | 16.19 | 191 | 0.37 |

**maze**

| Planner | Success | Time [ms] | Path length [m] | Expanded nodes | Min clearance [m] |
|---|---:|---:|---:|---:|---:|
| BFS | yes | 0.076 | 35.28 | 1,762 | 0.47 |
| Dijkstra | yes | 0.168 | 35.28 | 1,769 | 0.47 |
| A* | yes | 0.189 | 35.28 | 1,018 | 0.47 |
| Weighted A* (w=2) | yes | 0.104 | 35.28 | 418 | 0.47 |
| Theta* | yes | 0.432 | 33.98 | 1,032 | 0.41 |

**warehouse**

| Planner | Success | Time [ms] | Path length [m] | Expanded nodes | Min clearance [m] |
|---|---:|---:|---:|---:|---:|
| BFS | yes | 0.179 | 30.34 | 5,422 | 0.38 |
| Dijkstra | yes | 0.418 | 30.34 | 5,545 | 0.38 |
| A* | yes | 0.481 | 30.34 | 2,358 | 0.38 |
| Weighted A* (w=2) | yes | 0.059 | 33.59 | 296 | 0.38 |
| Theta* | yes | 0.883 | 28.85 | 2,403 | 0.35 |

**parking**

| Planner | Success | Time [ms] | Path length [m] | Expanded nodes | Min clearance [m] |
|---|---:|---:|---:|---:|---:|
| BFS | yes | 0.103 | 17.38 | 3,312 | 0.38 |
| Dijkstra | yes | 0.290 | 17.38 | 3,516 | 0.38 |
| A* | yes | 0.207 | 17.38 | 782 | 0.43 |
| Weighted A* (w=2) | yes | 0.040 | 17.38 | 68 | 0.43 |
| Theta* | yes | 0.321 | 16.07 | 672 | 0.62 |

**urban grid**

| Planner | Success | Time [ms] | Path length [m] | Expanded nodes | Min clearance [m] |
|---|---:|---:|---:|---:|---:|
| BFS | yes | 0.451 | 62.48 | 14,512 | 0.38 |
| Dijkstra | yes | 1.472 | 62.48 | 14,500 | 0.38 |
| A* | yes | 1.588 | 62.48 | 7,108 | 0.38 |
| Weighted A* (w=2) | yes | 0.088 | 62.48 | 424 | 0.38 |
| Theta* | yes | 3.074 | 58.34 | 6,537 | 0.45 |

BFS minimises the number of moves, not length; weighted A* trades optimality (bounded by
w) for fewer expansions; Theta* paths are any-angle and therefore shorter than grid paths.

## Sampling-based planners

Disc robot of radius 0.3 m, 3,000 iterations, 1 m steering step, PRM with 600 milestones.

**open space**

| Planner | Success rate | Time [ms] | Path length [m] | Tree / roadmap nodes | Collision checks | Min clearance [m] |
|---|---:|---:|---:|---:|---:|---:|
| RRT | 1.00 | 0.08 | 27.25 | 108 | 110 | 2.00 |
| RRT-Connect | 1.00 | 0.01 | 22.67 | 2 | 25 | 2.00 |
| RRT* | 1.00 | 10.81 | 23.05 | 2,860 | 10,924 | 2.00 |
| Informed RRT* | 1.00 | 25.39 | 22.63 | 3,001 | 15,387 | 2.00 |
| PRM | 1.00 | 1.83 | 23.40 | 602 | 4,076 | 2.00 |

**cluttered**

| Planner | Success rate | Time [ms] | Path length [m] | Tree / roadmap nodes | Collision checks | Min clearance [m] |
|---|---:|---:|---:|---:|---:|---:|
| RRT | 1.00 | 0.46 | 44.53 | 168 | 342 | 0.31 |
| RRT-Connect | 1.00 | 0.13 | 40.46 | 34 | 238 | 0.31 |
| RRT* | 1.00 | 9.34 | 36.15 | 1,936 | 6,742 | 0.31 |
| Informed RRT* | 1.00 | 10.48 | 35.13 | 2,006 | 7,239 | 0.31 |
| PRM | 1.00 | 3.43 | 34.51 | 602 | 4,399 | 0.33 |

**narrow passage**

| Planner | Success rate | Time [ms] | Path length [m] | Tree / roadmap nodes | Collision checks | Min clearance [m] |
|---|---:|---:|---:|---:|---:|---:|
| RRT | 1.00 | 0.18 | 20.28 | 162 | 292 | 0.34 |
| RRT-Connect | 1.00 | 0.01 | 17.11 | 10 | 31 | 0.34 |
| RRT* | 1.00 | 10.79 | 16.28 | 2,582 | 10,862 | 0.34 |
| Informed RRT* | 1.00 | 34.82 | 16.12 | 2,752 | 19,133 | 0.32 |
| PRM | 0.75 | 1.68 | 17.04 | 602 | 4,148 | 0.35 |

**warehouse**

| Planner | Success rate | Time [ms] | Path length [m] | Tree / roadmap nodes | Collision checks | Min clearance [m] |
|---|---:|---:|---:|---:|---:|---:|
| RRT | 1.00 | 0.23 | 37.08 | 168 | 360 | 0.32 |
| RRT-Connect | 1.00 | 0.07 | 35.04 | 25 | 168 | 0.32 |
| RRT* | 1.00 | 7.13 | 30.52 | 1,998 | 6,776 | 0.32 |
| Informed RRT* | 1.00 | 8.30 | 30.00 | 1,978 | 7,566 | 0.31 |
| PRM | 1.00 | 2.54 | 30.65 | 602 | 4,309 | 0.33 |

RRT and RRT-Connect stop at the first solution; RRT* and Informed RRT* use the whole
iteration budget to improve it. PRM's fixed roadmap can miss the narrow passage.

## Path-tracking controllers

Kinematic bicycle (wheelbase 2.7 m, |delta| <= 0.6 rad), 20 Hz control, speed profile
limited to 6 m/s, 1.5 m/s^2 and 2 m/s^2 lateral; vehicle starts at rest on the path.
Errors exclude the first 2 s. Steering effort = integral of delta^2; rate energy =
integral of (d delta/dt)^2.

**straight**

| Controller | Finished | Cross-track RMSE [m] | Max cross-track [m] | Heading RMSE [rad] | Steering effort | Steering-rate energy | Compute [us/step] |
|---|---:|---:|---:|---:|---:|---:|---:|
| pure_pursuit | yes | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 1.6 |
| stanley | yes | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 1.5 |
| lqr | yes | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 22.4 |
| mpc | yes | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 143.1 |

**circle**

| Controller | Finished | Cross-track RMSE [m] | Max cross-track [m] | Heading RMSE [rad] | Steering effort | Steering-rate energy | Compute [us/step] |
|---|---:|---:|---:|---:|---:|---:|---:|
| pure_pursuit | yes | 1.7e-04 | 4.1e-04 | 3.4e-05 | 0.558 | 1.9e-05 | 0.7 |
| stanley | yes | 0.005 | 0.031 | 0.180 | 0.518 | 1.4e-04 | 0.7 |
| lqr | yes | 1.7e-04 | 3.6e-04 | 2.1e-05 | 0.558 | 9.1e-05 | 11.2 |
| mpc | yes | 0.052 | 0.058 | 0.003 | 0.583 | 4.9e-04 | 128.8 |

**s curve**

| Controller | Finished | Cross-track RMSE [m] | Max cross-track [m] | Heading RMSE [rad] | Steering effort | Steering-rate energy | Compute [us/step] |
|---|---:|---:|---:|---:|---:|---:|---:|
| pure_pursuit | yes | 0.110 | 0.199 | 0.021 | 0.565 | 0.309 | 0.6 |
| stanley | yes | 0.013 | 0.022 | 0.186 | 0.493 | 0.288 | 0.6 |
| lqr | yes | 0.005 | 0.009 | 9.6e-04 | 0.569 | 0.341 | 9.8 |
| mpc | yes | 0.051 | 0.071 | 0.008 | 0.587 | 0.320 | 192.4 |

**sharp turns**

| Controller | Finished | Cross-track RMSE [m] | Max cross-track [m] | Heading RMSE [rad] | Steering effort | Steering-rate energy | Compute [us/step] |
|---|---:|---:|---:|---:|---:|---:|---:|
| pure_pursuit | yes | 0.153 | 0.348 | 0.068 | 2.201 | 1.604 | 0.6 |
| stanley | yes | 0.014 | 0.034 | 0.296 | 2.157 | 2.191 | 0.6 |
| lqr | yes | 0.027 | 0.065 | 0.009 | 2.833 | 14.330 | 9.9 |
| mpc | yes | 0.051 | 0.101 | 0.022 | 2.770 | 2.894 | 179.6 |

**variable curvature**

| Controller | Finished | Cross-track RMSE [m] | Max cross-track [m] | Heading RMSE [rad] | Steering effort | Steering-rate energy | Compute [us/step] |
|---|---:|---:|---:|---:|---:|---:|---:|
| pure_pursuit | yes | 0.098 | 0.199 | 0.018 | 0.992 | 0.306 | 0.6 |
| stanley | yes | 0.011 | 0.022 | 0.226 | 0.945 | 0.300 | 0.6 |
| lqr | yes | 0.004 | 0.009 | 7.1e-04 | 0.974 | 0.335 | 10.0 |
| mpc | yes | 0.053 | 0.079 | 0.007 | 1.010 | 0.318 | 164.6 |

## State estimation

Unicycle robot, range-bearing landmarks, 300 steps at 10 Hz, medians over 20 seeds. NEES/NIS are means over steps and runs; a
consistent filter gives NEES close to 3 (state dimension) and NIS close to 2 (measurement
dimension). The gated EKF rejects measurements with NIS above the 99% chi-square quantile.

**low noise**

| Filter | Position RMSE [m] | Heading RMSE [rad] | Mean NEES | Mean NIS | Diverged runs | Time [ms/step] |
|---|---:|---:|---:|---:|---:|---:|
| EKF | 0.015 | 0.0035 | 3.05 | 1.99 | 0 | 0.0079 |
| EKF + 99% gate | 0.016 | 0.0036 | 3.16 | 1.89 | 0 | 0.0081 |
| UKF | 0.015 | 0.0035 | 3.05 | 1.99 | 0 | 0.0268 |
| PF (500) | 0.015 | 0.0036 |  |  | 0 | 0.2143 |

**high odometry noise**

| Filter | Position RMSE [m] | Heading RMSE [rad] | Mean NEES | Mean NIS | Diverged runs | Time [ms/step] |
|---|---:|---:|---:|---:|---:|---:|
| EKF | 0.023 | 0.0047 | 3.38 | 2.05 | 0 | 0.0080 |
| EKF + 99% gate | 0.025 | 0.0050 | 3.67 | 1.93 | 0 | 0.0080 |
| UKF | 0.023 | 0.0047 | 3.38 | 2.05 | 0 | 0.0269 |
| PF (500) | 0.025 | 0.0049 |  |  | 0 | 0.2175 |

**high measurement noise**

| Filter | Position RMSE [m] | Heading RMSE [rad] | Mean NEES | Mean NIS | Diverged runs | Time [ms/step] |
|---|---:|---:|---:|---:|---:|---:|
| EKF | 0.060 | 0.0132 | 2.90 | 1.99 | 0 | 0.0077 |
| EKF + 99% gate | 0.061 | 0.0131 | 2.98 | 1.89 | 0 | 0.0082 |
| UKF | 0.060 | 0.0132 | 2.90 | 1.99 | 0 | 0.0272 |
| PF (500) | 0.062 | 0.0134 |  |  | 0 | 0.2112 |

**sparse landmarks**

| Filter | Position RMSE [m] | Heading RMSE [rad] | Mean NEES | Mean NIS | Diverged runs | Time [ms/step] |
|---|---:|---:|---:|---:|---:|---:|
| EKF | 0.038 | 0.0071 | 2.74 | 2.00 | 0 | 0.0026 |
| EKF + 99% gate | 0.038 | 0.0072 | 2.72 | 1.91 | 0 | 0.0024 |
| UKF | 0.038 | 0.0071 | 2.74 | 2.00 | 0 | 0.0087 |
| PF (500) | 0.049 | 0.0080 |  |  | 0 | 0.1684 |

**outlier measurement**

| Filter | Position RMSE [m] | Heading RMSE [rad] | Mean NEES | Mean NIS | Diverged runs | Time [ms/step] |
|---|---:|---:|---:|---:|---:|---:|
| EKF | 0.371 | 0.1206 | 2,739.77 | 1,881.78 | 0 | 0.0105 |
| EKF + 99% gate | 0.017 | 0.0037 | 3.32 | 1.91 | 0 | 0.0075 |
| UKF | 0.371 | 0.1206 | 2,739.73 | 1,881.78 | 0 | 0.0275 |
| PF (500) | 0.053 | 0.0121 |  |  | 0 | 0.2232 |

Without gating, the EKF and UKF are badly inconsistent under 5% outliers; the chi-square
gate restores consistency. The particle filter is robust without gating but costs about
20x more per step with 500 particles.

## SLAM

Landmark SLAM: 500 steps around a circle with 8 landmarks. Pose graph: two laps of a block
with drifting odometry and simulated place recognition.

| Method | ATE [m] | ATE odometry only [m] | Landmark RMSE [m] | Time [ms/step] | Total time [ms] |
|---|---:|---:|---:|---:|---:|
| EKF-SLAM | 0.060 |  | 0.063 | 0.0252 |  |
| FastSLAM 1.0 (50) | 0.091 |  | 0.090 | 0.0858 |  |
| Pose graph (Gauss-Newton) | 0.178 | 1.128 |  |  | 1.38 |

## Microbenchmarks

Google Benchmark, 5 repetitions, medians.

| Benchmark | Real time | CPU time | Unit |
|---|---:|---:|---:|
| BM_Se2ComposeExpLog | 40.3 | 38.4 | ns |
| BM_KdTreeNearest/100 | 319.5 | 304.8 | ns |
| BM_KdTreeNearest/1000 | 520.2 | 481.3 | ns |
| BM_KdTreeNearest/10000 | 698.3 | 662.7 | ns |
| BM_KdTreeNearest/100000 | 952.3 | 941.7 | ns |
| BM_AStarCluttered | 312.4 | 307.7 | us |
| BM_EkfPredictUpdate | 1,778.6 | 1,722.9 | ns |
| BM_UkfPredictUpdate | 8,597.2 | 8,021.8 | ns |
| BM_ReedsSheppShortest | 3,355.6 | 3,264.5 | ns |
| BM_DareDoubleIntegrator | 17,320.2 | 16,497.0 | ns |
| BM_IcpPointToLine | 1,072.5 | 1,001.0 | us |

Empirical complexity of k-d tree nearest-neighbour queries (fitted by Google Benchmark): BM_KdTreeNearest: lgN

