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

add_library(roboticscore_all INTERFACE)
set_target_properties(roboticscore_all PROPERTIES EXPORT_NAME RoboticsCore)
target_link_libraries(roboticscore_all INTERFACE roboticscore_core)
add_library(RoboticsCore::RoboticsCore ALIAS roboticscore_all)
