cmake_minimum_required(VERSION 3.25)

project(
  RoboticsCore
  VERSION 0.1.0
  DESCRIPTION "Classical robotics algorithms in modern C++20"
  HOMEPAGE_URL "https://github.com/M-amir1998/RoboticsCore"
  LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(RoboticsCoreOptions)
include(CompilerWarnings)
include(Sanitizers)
include(Coverage)
include(Dependencies)
include(RoboticsCoreHelpers)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

roboticscore_configure_sanitizers()
roboticscore_configure_coverage()

add_subdirectory(src)

if(ROBOTICSCORE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(ROBOTICSCORE_BUILD_APPS)
  add_subdirectory(apps)
endif()

if(ROBOTICSCORE_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

include(RoboticsCoreInstall)

message(STATUS "RoboticsCore ${PROJECT_VERSION}")
message(STATUS "  Compiler:        ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Build type:      ${CMAKE_BUILD_TYPE}")
message(STATUS "  Tests:           ${ROBOTICSCORE_BUILD_TESTS}")
message(STATUS "  Apps / demos:    ${ROBOTICSCORE_BUILD_APPS}")
message(STATUS "  Benchmarks:      ${ROBOTICSCORE_BUILD_BENCHMARKS}")
message(STATUS "  Sanitizers:      ${ROBOTICSCORE_SANITIZERS}")
message(STATUS "  Warnings=errors: ${ROBOTICSCORE_WARNINGS_AS_ERRORS}")
