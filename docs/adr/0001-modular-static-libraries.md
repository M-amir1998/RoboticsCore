# ADR 0001: Modular static libraries with one umbrella target

Status: Accepted

## Context

The library covers math, estimation, mapping, planning, trajectories, control, SLAM and a
simulator. Many users need only one area, for example a planner inside an existing stack. They
should not pay for unrelated code or dependencies. Users who want everything should need only
one `find_package` call and one target.

## Decision

- Each area is a static library `roboticscore_<module>`, exported as `RoboticsCore::<module>`.
  The modules are `core`, `estimation`, `mapping`, `planning`, `trajectory`, `control`, `slam`
  and `sim`.
- `core` holds the shared foundation:
  - the error policy and math utilities;
  - Lie groups and collision primitives;
  - spatial indexing and motion models;
  - the small dense optimisers.
- Module dependencies form a directed acyclic graph:
  - `estimation`, `mapping`, `planning`, `trajectory` and `sim` depend only on `core`.
  - `control` also depends on `trajectory`.
  - `slam` also depends on `estimation` and `mapping`.
- `RoboticsCore::RoboticsCore` is an interface target that links all modules.
- Headers live under `include/roboticscore/<area>/`, with one namespace per area (for example
  `roboticscore::planning`). Implementation files in `src/` mirror the header layout.
- Demos, pipelines and benchmark drivers live in `apps/`. They are never part of the installed
  library.

## Consequences

- Linking is fine-grained and incremental builds are faster. CMake enforces the dependency
  graph, so it does not rely on convention.
- Static libraries need no symbol-export boilerplate on Windows. A shared-library build is
  not offered at present.
- A new module needs a new `roboticscore_add_library` call and an entry in the umbrella
  target.
