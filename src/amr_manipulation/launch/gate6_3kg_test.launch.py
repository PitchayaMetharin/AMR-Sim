"""Prepare product 102 from the current AMR pose and run Gate 6."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="amr_manipulation",
            executable="gate6_product_test",
            name="gate6_product_test_3kg",
            parameters=[{"product_id": 102, "use_sim_time": True}],
            output="screen",
        ),
    ])
