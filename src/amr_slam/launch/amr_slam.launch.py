"""Launch online simulation-only SLAM mapping."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    parameters = [os.path.join(
        get_package_share_directory("amr_slam"), "config", "mapper.yaml")]
    overlay = LaunchConfiguration("params_overlay").perform(context)
    if overlay:
        parameters.append(overlay)
    return [Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="/amr",
        output="screen",
        parameters=parameters,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_overlay", default_value=""),
        OpaqueFunction(function=_launch_nodes),
    ])
