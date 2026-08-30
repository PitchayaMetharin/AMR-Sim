"""Launch the independent simulation-only LiDAR perception pipelines."""
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def managed_node(executable):
    node = LifecycleNode(
        package="amr_perception",
        executable=executable,
        name=executable,
        namespace="/amr",
        parameters=[{"use_sim_time": True}],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))
    return [node, configure, activate]


def generate_launch_description():
    actions = []
    for executable in (
            "front_lidar_perception_node", "rear_lidar_perception_node"):
        actions.extend(managed_node(executable))
    return LaunchDescription(actions)
