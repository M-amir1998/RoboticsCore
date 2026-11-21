# Project status

Version 0.1.0 (unreleased). The per-algorithm status matrix is in [README.md](README.md#what-is-implemented)
and is generated from [`configs/algorithms.yaml`](configs/algorithms.yaml).

## Verified

| Check | Environment | Result |
|---|---|---|
| Build, MSVC `/W4 /WX` | Windows 11, MSVC 19.50, Ninja, Release and Debug | Clean |
| Build, clang `-Wall -Wextra -Werror` | Windows 11, clang 20.1 | Clean |
| Test suites (unit, numerical, integration, regression) | Windows 11, MSVC Release | 256 tests pass |
| AddressSanitizer + UndefinedBehaviorSanitizer | Windows 11, clang 20.1 | All tests pass |
| clang-tidy (repository `.clang-tidy`) | clang-tidy 20.1 | No findings |
| clang-format, ruff check, ruff format | | Clean |
| Python tests (`pytest`) | Python 3.12 | Pass |
| Install + `find_package` from an external project | Windows 11, MSVC | Builds and runs |
| Linux build and test suites | ROS 2 Jazzy image, GCC 13.3, `-Werror` | 256 tests pass (`scripts/ros2/verify_in_docker.sh`) |
| ROS 2 adapter: colcon build, unit and closed-loop node tests | ROS 2 Jazzy image | Builds; 7 tests pass (conversions, closed-loop plan and track) |
| Benchmarks | Intel Core i5-1335U, Windows 11, MSVC Release | [docs/benchmarks](docs/benchmarks/README.md) |
| Animations | 12 GIFs, about 2.4 MB in total | [docs/assets/manifest.json](docs/assets/manifest.json) |

## Not verified

- The Jenkins pipeline ([`Jenkinsfile`](Jenkinsfile)) has not been run on a Jenkins
  controller. Every stage calls commands that were run locally.
- The Doxygen output: [`docs/Doxyfile`](docs/Doxyfile) has not been run, because Doxygen was
  not available on the development machine.
- Coverage (`coverage` preset): it is configured but has not been measured.
- Sanitizer runs with GCC on Linux, and clang-tidy on Linux.
- The ROS 2 adapter on distributions other than Jazzy, with Nav2, or on real robots.
- The C++ snippets in the algorithm pages: they follow the tested APIs but are not compiled
  as part of the build.

## Known limitations

These are listed in the [README](README.md#limitations) and on each algorithm page.

## Next steps

See the [roadmap](README.md#roadmap).
