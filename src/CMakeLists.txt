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
          planning/sampling/sampling_planners.cpp
  PUBLIC_DEPS roboticscore_core)

# sim: deterministic 2D simulator, sensors and benchmark scenario generators.
roboticscore_add_library(
  sim
  SOURCES sim/scenarios.cpp
          sim/simulation.cpp
          sim/world_sensors.cpp
  PUBLIC_DEPS roboticscore_core)

add_library(roboticscore_all INTERFACE)
set_target_properties(roboticscore_all PROPERTIES EXPORT_NAME RoboticsCore)
target_link_libraries(roboticscore_all INTERFACE roboticscore_core roboticscore_estimation
                                                 roboticscore_mapping roboticscore_planning
                                                 roboticscore_sim)
add_library(RoboticsCore::RoboticsCore ALIAS roboticscore_all)
