# Third-party dependencies.
#
# Library:    Eigen 3.4 (MPL-2.0), header-only. A system installation is preferred;
#             otherwise the release tarball is fetched and its headers are installed
#             alongside RoboticsCore so the exported package remains self-contained.
# Tests:      GoogleTest 1.15 (BSD-3-Clause).
# Benchmarks: Google Benchmark 1.9 (Apache-2.0).
# Apps:       nlohmann/json 3.11 (MIT), header-only, used by demo executables only.
#
# To work offline or share one download between several build trees, point
# FETCHCONTENT_SOURCE_DIR_<NAME> at an existing source checkout, e.g.
#   -DFETCHCONTENT_SOURCE_DIR_EIGEN=/path/to/eigen-3.4.0

include(FetchContent)

# ---------------------------------------------------------------------------
# Eigen
# ---------------------------------------------------------------------------
set(ROBOTICSCORE_EIGEN_BUNDLED OFF)
find_package(Eigen3 3.4 CONFIG QUIET)
if(TARGET Eigen3::Eigen)
  message(STATUS "Using system Eigen ${Eigen3_VERSION}")
  add_library(roboticscore_eigen INTERFACE)
  target_link_libraries(roboticscore_eigen INTERFACE Eigen3::Eigen)
else()
  message(STATUS "Eigen not found; fetching Eigen 3.4.0")
  FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    # Only the headers are needed; do not run Eigen's own CMake project.
    SOURCE_SUBDIR _roboticscore_headers_only)
  FetchContent_MakeAvailable(eigen)
  add_library(roboticscore_eigen INTERFACE)
  target_include_directories(
    roboticscore_eigen SYSTEM INTERFACE $<BUILD_INTERFACE:${eigen_SOURCE_DIR}>
                                        $<INSTALL_INTERFACE:include/eigen3>)
  set(ROBOTICSCORE_EIGEN_BUNDLED ON)
  set(ROBOTICSCORE_EIGEN_SOURCE_DIR "${eigen_SOURCE_DIR}")
endif()
set_target_properties(roboticscore_eigen PROPERTIES EXPORT_NAME eigen)
add_library(RoboticsCore::eigen ALIAS roboticscore_eigen)

# ---------------------------------------------------------------------------
# GoogleTest
# ---------------------------------------------------------------------------
if(ROBOTICSCORE_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
    FIND_PACKAGE_ARGS 1.14 NAMES GTest)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)
endif()

# ---------------------------------------------------------------------------
# Google Benchmark
# ---------------------------------------------------------------------------
if(ROBOTICSCORE_BUILD_BENCHMARKS)
  FetchContent_Declare(
    benchmark
    URL https://github.com/google/benchmark/archive/refs/tags/v1.9.1.tar.gz
    URL_HASH SHA256=32131c08ee31eeff2c8968d7e874f3cb648034377dfc32a4c377fa8796d84981
    FIND_PACKAGE_ARGS 1.8 NAMES benchmark)
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_INSTALL_DOCS OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_WERROR OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(benchmark)
endif()

# ---------------------------------------------------------------------------
# nlohmann/json (demo executables only; not part of the installed library)
# ---------------------------------------------------------------------------
if(ROBOTICSCORE_BUILD_APPS)
  FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
    SOURCE_SUBDIR _roboticscore_headers_only)
  FetchContent_MakeAvailable(nlohmann_json)
  add_library(roboticscore_json INTERFACE)
  target_include_directories(roboticscore_json SYSTEM
                             INTERFACE ${nlohmann_json_SOURCE_DIR}/single_include)
endif()
