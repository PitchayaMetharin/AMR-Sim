"""Launch global planning and collision-checked path smoothing."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    parameters = os.path.join(
        get_package_share_directory("amr_navigation"), "config", "planner.yaml")
    lattice_file = os.path.join(
        get_package_share_directory("nav2_smac_planner"), "sample_primitives",
        "5cm_resolution", "0.5m_turning_radius", "diff", "output.json")
    overlay = LaunchConfiguration("params_overlay").perform(context)
    planner_parameters = [parameters]
    if overlay:
        planner_parameters.append(overlay)
    planner_parameters.append({"GridBased.lattice_filepath": lattice_file,
                               "ExactGoalLattice.lattice_filepath": lattice_file})
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_planning",
        namespace="/amr",
        output="screen",
        parameters=[parameters],
    )
    return [
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace="/amr",
            output="screen",
            parameters=planner_parameters,
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            namespace="/amr",
            output="screen",
            parameters=[parameters],
        ),
        TimerAction(period=1.0, actions=[lifecycle_manager]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_overlay", default_value=""),
        OpaqueFunction(function=_launch_nodes),
    ])
