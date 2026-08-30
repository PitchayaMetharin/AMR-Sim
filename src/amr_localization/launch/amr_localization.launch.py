"""Launch encoder odometry and the sole local-state EKF."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    wheel_odometry = LifecycleNode(
        package="amr_localization",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        namespace="/amr",
        parameters=[{
            "use_sim_time": True,
            # Keep the encoder model identical to the Gazebo DiffDrive plugin.
            "wheel_radius": 0.1128,
            "wheel_separation": 0.566,
        }],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(wheel_odometry),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=wheel_odometry,
        goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(wheel_odometry),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace="/amr",
        output="screen",
        parameters=[os.path.join(
            get_package_share_directory("amr_localization"),
            "config",
            "ekf.yaml",
        )],
        remappings=[("odometry/filtered", "/amr/localization/odometry")],
    )
    return LaunchDescription([wheel_odometry, configure, activate, ekf])
