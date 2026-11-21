# Build options. Development targets default to ON only when RoboticsCore is the
# top-level project, so consumers using add_subdirectory/FetchContent get just the library.

option(ROBOTICSCORE_BUILD_TESTS "Build the GoogleTest suites" ${PROJECT_IS_TOP_LEVEL})
option(ROBOTICSCORE_BUILD_APPS "Build demo and benchmark-scenario executables" ${PROJECT_IS_TOP_LEVEL})
option(ROBOTICSCORE_BUILD_BENCHMARKS "Build Google Benchmark microbenchmarks" ${PROJECT_IS_TOP_LEVEL})
option(ROBOTICSCORE_WARNINGS_AS_ERRORS "Treat compiler warnings in project code as errors" OFF)
option(ROBOTICSCORE_ENABLE_COVERAGE "Instrument for gcov/llvm-cov coverage (GCC/Clang)" OFF)
option(ROBOTICSCORE_ENABLE_CLANG_TIDY "Run clang-tidy while compiling project targets" OFF)
option(ROBOTICSCORE_INSTALL "Generate install rules and CMake package config" ${PROJECT_IS_TOP_LEVEL})

set(ROBOTICSCORE_SANITIZERS "" CACHE STRING
    "Semicolon-separated sanitizers to enable (e.g. address;undefined). Empty disables.")
