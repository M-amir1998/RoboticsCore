# Google Benchmark microbenchmarks. Always measure Release builds:
#   cmake --preset release && cmake --build --preset release
#   ./build/release/benchmarks/roboticscore_microbenchmarks --benchmark_repetitions=5

function(roboticscore_add_benchmark name)
  cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})
  roboticscore_add_executable(${name} SOURCES ${ARG_SOURCES}
                              DEPS RoboticsCore::RoboticsCore benchmark::benchmark_main)
endfunction()
