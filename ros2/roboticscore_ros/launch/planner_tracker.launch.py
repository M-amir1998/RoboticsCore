"""Start the grid planner and the path tracker with the parameters in config/params.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("roboticscore_ros"), "config", "params.yaml")
    return LaunchDescription(
        [
            Node(package="roboticscore_ros", executable="grid_planner", name="grid_planner",
                 parameters=[params], output="screen"),
            Node(package="roboticscore_ros", executable="path_tracker", name="path_tracker",
                 parameters=[params], output="screen"),
        ]
    )
