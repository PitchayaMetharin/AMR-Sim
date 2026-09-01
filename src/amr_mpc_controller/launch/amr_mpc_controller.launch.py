"""Launch the Phase 14 Nav2 regulated pure pursuit controller and local costmap."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    parameters = os.path.join(
        get_package_share_directory("amr_mpc_controller"),
        "config",
        "controller.yaml",
    )
    # Humble's lifecycle manager starts autostart from a zero-delay wall
    # timer.  The controller server constructs its nested local costmap on
    # startup, so launching both processes together can send the first
    # change_state request before the controller has finished constructing its
    # lifecycle services.  Keep the controller process first and give that
    # construction a bounded one-second barrier before starting its manager.
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace="/amr",
        output="screen",
        parameters=[parameters],
        remappings=[("cmd_vel", "/amr/mpc/cmd_vel")],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_controller",
        namespace="/amr",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription([
        controller_server,
        TimerAction(period=1.0, actions=[lifecycle_manager]),
    ])
