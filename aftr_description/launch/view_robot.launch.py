# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Launch the robot description and visualization tools."""
    pkg_description = get_package_share_directory("aftr_description")

    urdf_file = os.path.join(pkg_description, "urdf", "robot.urdf.xacro")
    doc = xacro.process_file(urdf_file, mappings={"use_sim": "false"})
    robot_description = {"robot_description": doc.toxml()}

    rviz_config_file = os.path.join(pkg_description, "rviz", "view_robot.rviz")

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
            ),
        ]
    )
