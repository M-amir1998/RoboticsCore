# ADR 0007: ROS 2 integration as a separate adapter package

Status: Accepted

## Context

Many users run ROS 2. The algorithms themselves do not need ROS. A ROS dependency would make
the core library harder to build, test and reuse elsewhere.

## Decision

- The core library has no ROS dependency.
- ROS 2 support lives in [`ros2/roboticscore_ros`](../../ros2/roboticscore_ros), an
  `ament_cmake` package. It finds the installed `RoboticsCore` CMake package and wraps
  selected algorithms as nodes.
- Message conversions are kept in a single header.
- The adapter is optional and has its own build instructions. The root CMake project does not
  build it.

## Consequences

- The core library builds on any C++20 toolchain without ROS.
- The adapter must follow ROS 2 API changes on its own. Its README states how far it has been
  verified.
