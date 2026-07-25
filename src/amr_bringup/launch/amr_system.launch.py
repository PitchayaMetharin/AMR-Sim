"""Phase 4 launch ownership only; runtime nodes are added by their owner phases."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
    ])
