"""Launch global planning and collision-checked path smoothing."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    parameters = os.path.join(
        get_package_share_directory("amr_navigation"), "config", "planner.yaml")
    lattice_file = os.path.join(
        get_package_share_directory("nav2_smac_planner"), "sample_primitives",
        "5cm_resolution", "0.5m_turning_radius", "diff", "output.json")
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_planning",
        namespace="/amr",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription([
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace="/amr",
            output="screen",
            parameters=[parameters, {"GridBased.lattice_filepath": lattice_file}],
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
    ])
