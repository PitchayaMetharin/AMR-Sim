"""Launch the fail-closed frontier explorer behind the mission action boundary."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("amr_exploration"),
        "config",
        "frontier_explorer.yaml",
    )
    return LaunchDescription([
        Node(
            package="amr_exploration",
            executable="frontier_explorer.py",
            name="frontier_explorer",
            namespace="/amr",
            parameters=[config],
            output="screen",
        ),
    ])
