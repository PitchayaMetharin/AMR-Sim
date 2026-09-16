"""Canonical AWS warehouse preset for the world-agnostic exploration launch."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Declare the preset toggles and include portable exploration once."""

    package_share = Path(get_package_share_directory("amr_simulation"))
    portable_launch = package_share / "launch" / "portable_exploration.launch.py"
    world = package_share / "worlds" / "aws_warehouse.sdf"
    rviz_config = package_share / "rviz" / "aws_warehouse_exploration.rviz"

    portable = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(portable_launch)),
        launch_arguments={
            "world": str(world),
            "initial_x": "0.0",
            "initial_y": "0.0",
            "initial_z": "0.12",
            "initial_yaw": "0.0",
            "resource_paths": "",
            "headless": LaunchConfiguration("headless"),
            "rviz": LaunchConfiguration("rviz"),
            "auto_start_exploration": LaunchConfiguration("auto_start_exploration"),
            # This is intentionally internal: portable exploration keeps its
            # sensors.rviz default and does not grow another public argument.
            "rviz_config": str(rviz_config),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "auto_start_exploration",
            default_value="true",
            choices=["true", "false"],
        ),
        portable,
    ])
