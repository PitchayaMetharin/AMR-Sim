"""Launch the standalone Phase 14 KUKA trajectory smoke test."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_gazebo(context):
    server_only = LaunchConfiguration("headless").perform(context).lower() == "true"
    arguments = "-r " + ("-s " if server_only else "") + "-v 2 empty.sdf"
    resource_paths = [
        os.path.dirname(get_package_share_directory("kuka_agilus_support")),
        os.path.dirname(get_package_share_directory("amr_description")),
    ]
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH")
    if existing_resource_path:
        resource_paths.append(existing_resource_path)
    return [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.pathsep.join(resource_paths),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )),
            launch_arguments={"gz_args": arguments}.items(),
        ),
    ]


def generate_launch_description():
    description = get_package_share_directory("amr_description")
    robot_xml = xacro.process_file(
        os.path.join(description, "urdf", "phase14_kr6_r900_2.urdf.xacro"),
        mappings={"controller_config": os.path.join(
            description, "config", "phase14_arm_controllers.yaml")},
    ).toxml()
    robot_description = {"robot_description": robot_xml, "use_sim_time": True}

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "phase14_kuka", "-param", "robot_description"],
        parameters=[robot_description],
        output="screen",
    )
    joint_states = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        OpaqueFunction(function=launch_gazebo),
        state_publisher,
        spawn,
        bridge,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=[joint_states],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_states,
            on_exit=[arm_controller],
        )),
    ])
