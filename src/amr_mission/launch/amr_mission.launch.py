"""Launch and activate the fail-closed mission boundary."""
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    mission = LifecycleNode(
        package="amr_mission",
        executable="mission_supervisor_node",
        name="mission_supervisor_node",
        namespace="/amr",
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription([
        mission,
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(mission),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=mission,
            goal_state="inactive",
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(mission),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )),
    ])
