# roboticscore_ros

**Status: Experimental.** This is an optional ROS 2 adapter for RoboticsCore. It is a separate
`ament_cmake` package that consumes the installed core library (see
[ADR 0007](../../docs/adr/0007-ros2-adapter.md)).

## Nodes

| Executable | Algorithm | Subscribes | Publishes |
|---|---|---|---|
| `grid_planner` | A*, Dijkstra or Theta* on an inflated occupancy grid | `map` (nav_msgs/OccupancyGrid), `initialpose` (geometry_msgs/PoseWithCovarianceStamped), `goal_pose` (geometry_msgs/PoseStamped) | `plan` (nav_msgs/Path, transient local) |
| `path_tracker` | Pure Pursuit on a kinematic bicycle model, with a speed profile limited by speed, acceleration and lateral acceleration | `plan` (nav_msgs/Path), `odom` (nav_msgs/Odometry) | `cmd_vel` (geometry_msgs/Twist) |

The parameters and their defaults are documented in
[`grid_planner_node.hpp`](include/roboticscore_ros/grid_planner_node.hpp) and
[`path_tracker_node.hpp`](include/roboticscore_ros/path_tracker_node.hpp). An example
parameter file is in [`config/params.yaml`](config/params.yaml).

`initialpose` and `goal_pose` are the topics that the RViz *2D Pose Estimate* and
*2D Goal Pose* tools publish to.

The tracker converts the steering angle δ into a twist (v, v·tan δ / L). This means it can also
drive differential-drive bases: set a small virtual wheelbase `L` for them.

## Build

```bash
# 1. Install the core library.
cmake -S . -B build/release -DCMAKE_BUILD_TYPE=Release
cmake --build build/release
cmake --install build/release --prefix /opt/roboticscore

# 2. Build the adapter in a colcon workspace.
mkdir -p ~/ws/src && cp -r ros2/roboticscore_ros ~/ws/src/
cd ~/ws && source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_PREFIX_PATH=/opt/roboticscore
source install/setup.bash
ros2 launch roboticscore_ros planner_tracker.launch.py
```

## Verification

[`scripts/ros2/verify_in_docker.sh`](../../scripts/ros2/verify_in_docker.sh) runs these steps
inside the `ros:jazzy-ros-base` image:

1. Builds the core library with GCC and runs its test suite.
2. Installs the core library.
3. Builds this package with colcon.
4. Runs its tests:
   - `test_conversions`: map layout and thresholds, quaternion and yaw conversion, twist
     conversion.
   - `test_nodes`: a closed-loop test. A map with a wall is published, the planner routes
     around the wall, and the tracker drives a simulated unicycle to the goal. The test also
     checks that the planner reports no path when the goal is unreachable.

The script has been run against ROS 2 Jazzy only. Other distributions, real robots and
Nav2 integration have not been tested.

## Limitations

- There are no TF lookups: start, goal and odometry must be expressed in the map frame.
- Rotated map origins are rejected.
- The planner plans once per new map, start or goal. It does not replan continuously.
- There is no obstacle avoidance beyond the static map. Combine it with a local planner for
  dynamic environments.
