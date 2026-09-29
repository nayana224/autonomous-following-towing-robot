# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch fall detection with its configured camera and model parameters."""
    package_share = Path(
        get_package_share_directory(
            "aftr_fall_detection"
        )
    )

    parameter_file = (
        package_share
        / "config"
        / "fall_detection.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="aftr_fall_detection",
                executable="fall_detection_node",
                name="fall_detection",
                output="screen",
                emulate_tty=True,
                parameters=[str(parameter_file)],
            ),
        ]
    )
