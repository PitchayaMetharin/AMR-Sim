"""Start the complete Phase 14 factory graph with the existing local stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    factory = get_package_share_directory("amr_factory")
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(factory, "launch", "factory_localization.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "software_rendering": LaunchConfiguration("software_rendering"),
            "require_hardware_rendering": LaunchConfiguration(
                "require_hardware_rendering"),
            "factory_attachment": LaunchConfiguration("factory_attachment"),
            "control_mode": LaunchConfiguration("control_mode"),
            "map_yaml": LaunchConfiguration("map_yaml"),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_yaw": LaunchConfiguration("initial_yaw"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("software_rendering", default_value="false"),
        DeclareLaunchArgument("require_hardware_rendering", default_value="true"),
        DeclareLaunchArgument("factory_attachment", default_value="false"),
        # Production behavior remains autonomous by default; manual mode
        # excludes the Nav2 controller and is intended for prototype_teleop.
        DeclareLaunchArgument("control_mode", default_value="autonomous"),
        DeclareLaunchArgument(
            "map_yaml", default_value=os.path.join(factory, "maps", "factory.yaml")),
        DeclareLaunchArgument("initial_x", default_value="-4.5"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        localization,
        Node(
            package="amr_manipulation",
            executable="manipulation_supervisor_node",
            name="manipulation_supervisor_node",
            namespace="/amr",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="amr_factory",
            executable="factory_supervisor_node",
            name="factory_supervisor_node",
            namespace="/amr",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])
