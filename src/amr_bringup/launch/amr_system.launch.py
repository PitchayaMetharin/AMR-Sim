"""ROS-only simulation launch ownership."""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    health = LifecycleNode(
        package="amr_health",
        executable="health_supervisor_node",
        name="health_supervisor_node",
        namespace="/amr",
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        health,
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(health),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=health,
            goal_state="inactive",
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(health),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )),
    ])
