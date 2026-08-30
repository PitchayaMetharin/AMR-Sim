"""Launch global planning and collision-checked path smoothing."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = os.path.join(
        get_package_share_directory("amr_navigation"), "config", "planner.yaml")
    return LaunchDescription([
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace="/amr",
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            namespace="/amr",
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_planning",
            namespace="/amr",
            output="screen",
            parameters=[parameters],
        ),
    ])
