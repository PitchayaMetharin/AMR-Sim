"""Launch the Phase 14 Nav2 regulated pure pursuit controller and local costmap."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = os.path.join(
        get_package_share_directory("amr_mpc_controller"),
        "config",
        "controller.yaml",
    )
    return LaunchDescription([
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            namespace="/amr",
            output="screen",
            parameters=[parameters],
            remappings=[("cmd_vel", "/amr/mpc/cmd_vel")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_controller",
            namespace="/amr",
            output="screen",
            parameters=[parameters],
        ),
    ])
