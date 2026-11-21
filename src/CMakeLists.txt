# Library targets. Each module is a static library exported as RoboticsCore::<module>;
# RoboticsCore::RoboticsCore links everything.

configure_file(${PROJECT_SOURCE_DIR}/include/roboticscore/core/version.hpp.in
               ${PROJECT_BINARY_DIR}/include/roboticscore/core/version.hpp @ONLY)

# core: error policy, math utilities, Lie groups, collision primitives, spatial
# indexing, robot motion models and small dense optimisers.
roboticscore_add_library(
  core
  SOURCES core/version.cpp
          geometry/collision.cpp
          geometry/environment.cpp
          geometry/so2_se2.cpp
          geometry/so3_se3.cpp
          math/angles.cpp
          math/integration.cpp
          math/interpolation.cpp
          math/linear_algebra.cpp
          math/random.cpp
          math/statistics.cpp
          models/bicycle.cpp
          models/double_integrator.cpp
          models/unicycle.cpp
          optimization/least_squares.cpp
          optimization/qp_solver.cpp
          spatial/kd_tree.cpp
  PUBLIC_DEPS roboticscore_eigen)

# estimation: generic Bayesian filters plus mobile-robot localisation models.
roboticscore_add_library(
  estimation
  SOURCES estimation/consistency.cpp
          estimation/extended_kalman_filter.cpp
          estimation/kalman_filter.cpp
          estimation/particle_filter.cpp
          estimation/resampling.cpp
          estimation/unscented_kalman_filter.cpp
          localization/robot_models.cpp
  PUBLIC_DEPS roboticscore_core)

# mapping: occupancy grids, grid ray traversal and scan registration.
roboticscore_add_library(
  mapping
  SOURCES mapping/occupancy_grid.cpp
          mapping/raycast.cpp
          registration/icp.cpp
  PUBLIC_DEPS roboticscore_core)

# planning: grid, sampling-based, kinodynamic and local planners.
roboticscore_add_library(
  planning
  SOURCES planning/grid/dstar_lite.cpp
          planning/grid/grid_map.cpp
          planning/grid/grid_search.cpp
          planning/kinodynamic/hybrid_astar.cpp
          planning/kinodynamic/reeds_shepp.cpp
          planning/local/dwa.cpp
          planning/local/potential_field.cpp
          planning/local/velocity_obstacles.cpp
          planning/sampling/sampling_planners.cpp
          planning/smoothing.cpp
  PUBLIC_DEPS roboticscore_core)

# trajectory: polynomials, splines, time parameterisation and reference paths.
roboticscore_add_library(
  trajectory
  SOURCES trajectory/polynomial.cpp
          trajectory/reference_path.cpp
          trajectory/spline.cpp
          trajectory/time_parameterization.cpp
          trajectory/tracks.cpp
  PUBLIC_DEPS roboticscore_core)

# control: PID and path-tracking controllers (Pure Pursuit, Stanley, LQR, MPC).
roboticscore_add_library(
  control
  SOURCES control/geometric_trackers.cpp
          control/lqr.cpp
          control/mpc.cpp
          control/path_tracking.cpp
          control/pid.cpp
  PUBLIC_DEPS roboticscore_core roboticscore_trajectory)

# sim: deterministic 2D simulator, sensors and benchmark scenario generators.
roboticscore_add_library(
  sim
  SOURCES sim/scenarios.cpp
          sim/simulation.cpp
          sim/world_sensors.cpp
  PUBLIC_DEPS roboticscore_core)

add_library(roboticscore_all INTERFACE)
set_target_properties(roboticscore_all PROPERTIES EXPORT_NAME RoboticsCore)
target_link_libraries(roboticscore_all INTERFACE roboticscore_control roboticscore_core roboticscore_estimation
                                                 roboticscore_mapping roboticscore_planning
                                                 roboticscore_sim roboticscore_trajectory)
add_library(RoboticsCore::RoboticsCore ALIAS roboticscore_all)
