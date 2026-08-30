"""Launch and activate the Phase 11 fail-closed control boundary."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def managed_node(executable, parameters):
    node = LifecycleNode(
        package="amr_control",
        executable=executable,
        name=executable,
        namespace="/amr",
        output="screen",
        parameters=parameters if isinstance(parameters, list) else [parameters],
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
    return node, activate, configure


def generate_launch_description():
    parameters = os.path.join(
        get_package_share_directory("amr_control"), "config", "control.yaml")
    actions = []
    for executable in ("command_arbitration_node",):
        actions.extend(managed_node(executable, [parameters, {
            # Keep the generic control launch base-only by default.  Factory
            # orchestration explicitly enables this interlock below.
            "require_manipulator_stowed": ParameterValue(
                LaunchConfiguration("require_manipulator_stowed"), value_type=bool),
        }]))
    return LaunchDescription([
        DeclareLaunchArgument("require_manipulator_stowed", default_value="false"),
        *actions,
    ])
