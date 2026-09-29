# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Launch the AFTR status LED node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for the status LED package."""
    return LaunchDescription(
        [
            Node(
                package="aftr_status_led",
                executable="status_led_node",
                name="mdbot_status_led",
                output="screen",
            )
        ]
    )
