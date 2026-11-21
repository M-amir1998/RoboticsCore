# Demo and benchmark-scenario executables. They link the public library targets only and
# write machine-readable JSON traces that the Python tooling renders and tabulates.

# Integrated pipelines shared by the demos, the scenario benchmarks and the integration tests.
add_library(roboticscore_pipelines STATIC pipelines/hero.cpp pipelines/loop_slam.cpp
                                          pipelines/parking.cpp pipelines/tracking.cpp)
target_include_directories(roboticscore_pipelines PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/pipelines)
target_link_libraries(roboticscore_pipelines PUBLIC RoboticsCore::RoboticsCore)
roboticscore_set_warnings(roboticscore_pipelines)

roboticscore_add_executable(
  roboticscore_demo
  SOURCES demos/main.cpp demos/demos_control.cpp demos/demos_estimation.cpp
          demos/demos_integrated.cpp demos/demos_planning.cpp
  DEPS roboticscore_pipelines roboticscore_json)

roboticscore_add_executable(roboticscore_bench SOURCES benchmarks/bench_main.cpp
                            DEPS roboticscore_pipelines roboticscore_json)
