"""Start project-owned MoveIt for the Phase 14 composite robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description = get_package_share_directory("amr_description")
    manipulation = get_package_share_directory("amr_manipulation")
    moveit_config = (
        MoveItConfigsBuilder(
            "phase14_mobile_manipulator", package_name="amr_manipulation")
        .robot_description(
            file_path=os.path.join(
                description, "urdf", "phase14_mobile_manipulator.urdf.xacro"),
            mappings={
                "controller_config": os.path.join(
                    description,
                    "config",
                    "phase14_mobile_manipulator_controllers.yaml",
                ),
                "loaded_product": "false",
                "factory_attachment": "true",
                "joint_state_topic": "/world/factory_world/model/amr/joint_state",
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(
                description, "config", "phase14_mobile_manipulator.srdf"))
        .robot_description_kinematics(
            file_path=os.path.join(manipulation, "config", "kinematics.yaml"))
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
            remappings=[
                ("joint_states", "/amr/base/joint_states"),
            ],
        )
    ])
