#!/usr/bin/env bash
# Builds and tests RoboticsCore with GCC on Linux and then the ROS 2 adapter package, inside
# the official ROS 2 Jazzy image. Run from the repository root:
#
#   docker run --rm -v "$PWD":/src:ro ros:jazzy-ros-base bash /src/scripts/ros2/verify_in_docker.sh
#
# The repository is mounted read-only; all build output stays inside the container.
# Set ROBOTICSCORE_DEPS=/src/build/_deps-src to reuse local dependency sources offline.
# ROS setup scripts reference unset variables, so source them before enabling -u.

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -euo pipefail
JOBS="${JOBS:-4}"
deps=()
if [[ -n "${ROBOTICSCORE_DEPS:-}" ]]; then
  deps=(-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$ROBOTICSCORE_DEPS/googletest"
        -DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON="$ROBOTICSCORE_DEPS/nlohmann_json"
        -DFETCHCONTENT_SOURCE_DIR_EIGEN="$ROBOTICSCORE_DEPS/eigen")
fi

echo "== RoboticsCore: configure, build and test with $(g++ --version | head -1)"
cmake -S /src -B /tmp/core -DCMAKE_BUILD_TYPE=Release -DROBOTICSCORE_BUILD_BENCHMARKS=OFF \
  -DROBOTICSCORE_WARNINGS_AS_ERRORS=ON "${deps[@]}"
cmake --build /tmp/core -j "$JOBS"
ctest --test-dir /tmp/core -j "$JOBS" --output-on-failure
cmake --install /tmp/core --prefix /opt/roboticscore

echo "== roboticscore_ros: colcon build and test"
mkdir -p /tmp/ws/src
cp -r /src/ros2/roboticscore_ros /tmp/ws/src/
cd /tmp/ws
colcon build --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/roboticscore
colcon test --event-handlers console_direct+
colcon test-result --verbose
echo "== all checks passed"
