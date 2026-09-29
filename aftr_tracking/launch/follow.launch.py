# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Launch the MDBOT person follower."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the follow launch description."""
    robust_tracking = LaunchConfiguration("robust_tracking")
    robust_person_follower = Node(
        package="aftr_tracking",
        executable="person_follower_robust",
        name="person_follower",
        output="screen",
        condition=IfCondition(robust_tracking),
    )
    stable_person_follower = Node(
        package="aftr_tracking",
        executable="person_follower",
        name="person_follower",
        output="screen",
        condition=UnlessCondition(robust_tracking),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robust_tracking",
                default_value="false",
                description="Use conservative target identity locking.",
            ),
            robust_person_follower,
            stable_person_follower,
        ]
    )
