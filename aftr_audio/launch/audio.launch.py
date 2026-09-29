# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Launch the AFTR audio feedback node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for the AFTR audio package."""
    return LaunchDescription(
        [
            Node(
                package="aftr_audio",
                executable="audio_node",
                name="mdbot_audio",
                output="screen",
            )
        ]
    )
