"""Canonical AWS warehouse static-map localization preset."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Launch the AWS world with a saved map and AMCL instead of SLAM."""

    package_share = Path(get_package_share_directory("amr_simulation"))
    portable_launch = package_share / "launch" / "portable_exploration.launch.py"
    world = package_share / "worlds" / "aws_warehouse.sdf"
    rviz_config = package_share / "rviz" / "aws_warehouse_exploration.rviz"

    portable = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(portable_launch)),
        launch_arguments={
            "world": str(world),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_z": LaunchConfiguration("initial_z"),
            "initial_yaw": LaunchConfiguration("initial_yaw"),
            "resource_paths": "",
            "headless": LaunchConfiguration("headless"),
            "software_rendering": LaunchConfiguration("software_rendering"),
            "rviz": LaunchConfiguration("rviz"),
            "auto_start_exploration": "false",
            "simulation_diagnostics": "false",
            "localization_mode": "amcl",
            "map_yaml": LaunchConfiguration("map_yaml"),
            "rviz_config": str(rviz_config),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_yaml",
            default_value="",
            description=(
                "Absolute path to the validated saved occupancy-map YAML; "
                "the launch fails before starting if it is missing."),
        ),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_z", default_value="0.12"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument(
            "software_rendering", default_value="auto", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        portable,
    ])
