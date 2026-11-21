# ADR 0003: Minimal, pinned third-party dependencies

Status: Accepted

## Context

A classical-robotics library should be easy to build on any machine with a C++20 compiler,
including offline machines and CI agents. Every dependency adds build time, licensing obligations and
supply-chain risk.

## Decision

| Dependency | Version | License | Used by |
|---|---|---|---|
| Eigen | 3.4.0 | MPL-2.0 | Library: header-only, and the only dependency of the installed library |
| GoogleTest | 1.15.2 | BSD-3-Clause | Tests |
| Google Benchmark | 1.9.1 | Apache-2.0 | Microbenchmarks |
| nlohmann/json | 3.11.3 | MIT | Demo and benchmark executables, for trace and result output |

- An installed Eigen 3.4 is preferred. If none is found, `FetchContent` downloads the release
  archive. Its headers are then installed alongside RoboticsCore, so the exported package stays
  self-contained.
- Every download is pinned to a release and checked against a SHA-256 hash in
  [`cmake/Dependencies.cmake`](../../cmake/Dependencies.cmake).
- For offline builds, `FETCHCONTENT_SOURCE_DIR_<NAME>` can point to local copies.
- Test, benchmark and app dependencies are fetched only when the matching
  `ROBOTICSCORE_BUILD_*` option is on.
- Some algorithms are often taken from external libraries: QP solving, sparse least squares,
  k-d trees and Reeds-Shepp curves. RoboticsCore implements them itself and documents them as
  internal implementations. For example, the QP solver follows the OSQP iteration but is not
  the OSQP library.
- The Python visualisation uses NumPy, Matplotlib and Pillow. It is not needed to build or use
  the C++ library.

## Consequences

- Builds are reproducible and auditable. Upgrading a dependency is an explicit change to its
  pinned URL and hash.
- The internal solvers are smaller and less optimised than dedicated libraries. Their accuracy
  and limits are documented on the corresponding algorithm pages.
