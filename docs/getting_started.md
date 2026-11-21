# Getting started

## Prerequisites

| Tool | Version | Notes |
|---|---|---|
| C++ compiler | C++20 | Tested with MSVC 19.50 and clang 20 on Windows; GCC 12+ is expected to work but is not part of the verified matrix yet |
| CMake | 3.25+ | Presets version 6 |
| Ninja | any recent | Generator used by all presets |
| Python | 3.11 or 3.12 | Only for visualisation, benchmark reports and doc tooling |

Eigen, GoogleTest, Google Benchmark and nlohmann/json are downloaded automatically, pinned
to verified hashes ([ADR 0003](adr/0003-dependencies.md)). An installed Eigen 3.4 is used
if CMake can find one.

On Windows, open a developer environment first:

```bat
call scripts\setup\vs_env.cmd
```

## Build and test

```bash
cmake --preset release
cmake --build --preset release
ctest --preset release                    # all suites
ctest --preset release -L unit            # one suite: unit, numerical, integration, regression
```

Other presets: `debug`, `clang-debug`, `clang-release`, `gcc-debug`, `gcc-release`,
`sanitize` (AddressSanitizer + UndefinedBehaviorSanitizer), `coverage`, `ci-debug` and
`ci-release` (warnings as errors).

| CMake option | Default | Effect |
|---|---|---|
| `ROBOTICSCORE_BUILD_TESTS` | ON at top level | GoogleTest suites |
| `ROBOTICSCORE_BUILD_APPS` | ON at top level | `roboticscore_demo`, `roboticscore_bench` |
| `ROBOTICSCORE_BUILD_BENCHMARKS` | ON at top level | Google Benchmark microbenchmarks |
| `ROBOTICSCORE_INSTALL` | ON at top level | Install rules and CMake package |
| `ROBOTICSCORE_WARNINGS_AS_ERRORS` | OFF | `-Werror` / `/WX` |
| `ROBOTICSCORE_ENABLE_CLANG_TIDY` | OFF | Run clang-tidy during compilation |
| `ROBOTICSCORE_ENABLE_COVERAGE` | OFF | gcov / llvm-cov instrumentation (GCC, Clang) |
| `ROBOTICSCORE_SANITIZERS` | empty | For example `address;undefined` |

When RoboticsCore is consumed with `add_subdirectory` or `FetchContent`, only the library
is built by default.

## Use the library from another CMake project

```bash
cmake --install build/release --prefix /opt/roboticscore
cmake -S examples/external_project -B build/external -DCMAKE_PREFIX_PATH=/opt/roboticscore
cmake --build build/external
```

```cmake
find_package(RoboticsCore 0.1 CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE RoboticsCore::planning RoboticsCore::control)
# or RoboticsCore::RoboticsCore for every module
```

[`examples/external_project/main.cpp`](../examples/external_project/main.cpp) plans a path with
A*, converts it into a reference path and tracks it with Pure Pursuit on a kinematic bicycle.

## First program

```cpp
#include "roboticscore/planning/grid/grid_search.hpp"

using namespace roboticscore;

int main() {
  planning::GridMap map(geometry::GridSpec(40, 30, 0.5));  // 40 x 30 cells of 0.5 m
  for (int y = 0; y < 22; ++y) map.set_blocked({20, y}, true);
  const planning::GridPath plan = planning::astar(map, {2, 2}, {37, 3});
  return plan.found() ? 0 : 1;  // plan.cells, plan.cost, plan.expanded
}
```

No exception means the call ran; `plan.status` says whether a path was found
([ADR 0002](adr/0002-error-handling.md)).

## Demos and animations

`roboticscore_demo` runs a named scenario and writes a JSON trace:

```bash
./build/release/apps/roboticscore_demo --list
./build/release/apps/roboticscore_demo astar --seed 3 --config configs/planning/astar.json --out astar.json
```

The Python package renders traces into the GIFs in [`assets/`](assets/):

```bash
python -m venv .venv && . .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
python scripts/render/render_astar.py          # one animation
python scripts/render/render_all.py            # all of them; updates docs/assets/manifest.json
```

The renderer finds the demo executable in `$ROBOTICSCORE_DEMO`, on `PATH` or in the usual
build directories. [`assets/manifest.json`](assets/manifest.json) records the demo, seed,
configuration and commit behind every GIF.

## Benchmarks

```bash
python scripts/benchmark/run_benchmarks.py --build-dir build/release
```

This runs the scenario benchmarks and microbenchmarks and regenerates
[`benchmarks/README.md`](benchmarks/README.md). Only Release builds give meaningful timings.

## Where next

- [Architecture](architecture.md): modules, dependencies, conventions.
- [Algorithm pages](README.md): formulation, assumptions, complexity and limits for every
  algorithm.
- [Mathematics notes](mathematics/lie_groups.md): Lie groups and numerical methods.
