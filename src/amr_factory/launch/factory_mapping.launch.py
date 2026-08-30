"""Launch the factory world in online mapping mode.

This is deliberately a separate entry point from the production localization
launch.  The shared factory graph suppresses map_server and AMCL when
``mapping_mode`` is true; SLAM Toolbox is then the only ``map -> odom`` owner.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def validate_mapping_session(context):
    """Require a narrow, explicit session path before starting the graph."""
    raw_path = LaunchConfiguration("session_dir").perform(context).strip()
    if not raw_path:
        raise RuntimeError(
            "session_dir is required for factory mapping; choose a run-specific directory"
        )
    session_dir = Path(raw_path).expanduser().resolve()
    if session_dir == Path("/"):
        raise RuntimeError("session_dir must not be the filesystem root")
    canonical_maps = {
        Path(get_package_share_directory("amr_factory"), "maps").resolve(),
        Path(__file__).resolve().parents[1] / "maps",
    }
    if any(session_dir == path or path in session_dir.parents for path in canonical_maps):
        raise RuntimeError("session_dir must not be the canonical factory maps directory")


def validate_mapping_options(context):
    """Reject unknown commissioning control modes before starting processes."""
    control_mode = LaunchConfiguration("control_mode").perform(context).strip().lower()
    if control_mode not in {"manual", "autonomous"}:
        raise RuntimeError("control_mode must be manual or autonomous")


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
            "mapping_mode": "true",
            "map_yaml": LaunchConfiguration("map_yaml"),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_yaw": LaunchConfiguration("initial_yaw"),
        }.items(),
    )
    slam_parameters = os.path.join(
        get_package_share_directory("amr_slam"), "config", "mapper.yaml")
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="/amr",
        parameters=[slam_parameters],
        output="screen",
    )
    explorer = Node(
        package="amr_exploration",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        namespace="/amr",
        parameters=[os.path.join(
            get_package_share_directory("amr_exploration"),
            "config", "frontier_explorer.yaml")],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("control_mode"), "' == 'autonomous'",
        ])),
        output="screen",
    )
    # Preserve the existing stowed-proof interlock in both mapping modes; this
    # node does not publish base velocity or bypass the control boundary.
    manipulation = Node(
        package="amr_manipulation",
        executable="manipulation_supervisor_node",
        name="manipulation_supervisor_node",
        namespace="/amr",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("software_rendering", default_value="false"),
        DeclareLaunchArgument("require_hardware_rendering", default_value="true"),
        DeclareLaunchArgument("factory_attachment", default_value="false"),
        DeclareLaunchArgument("control_mode", default_value="manual"),
        DeclareLaunchArgument("session_dir"),
        # Kept explicit for the shared launch contract; mapping mode never
        # starts map_server and therefore never reads this file.
        DeclareLaunchArgument(
            "map_yaml", default_value=os.path.join(factory, "maps", "factory.yaml")),
        DeclareLaunchArgument("initial_x", default_value="-4.5"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        OpaqueFunction(function=validate_mapping_options),
        OpaqueFunction(function=validate_mapping_session),
        SetEnvironmentVariable(
            name="AMR_FACTORY_MAPPING_SESSION_DIR",
            value=LaunchConfiguration("session_dir")),
        localization,
        slam,
        manipulation,
        explorer,
    ])
