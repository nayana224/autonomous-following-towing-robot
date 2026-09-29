# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Launch the base, controller, and LiDAR stack in its established order."""
    pkg_description = get_package_share_directory("aftr_description")
    pkg_bringup = get_package_share_directory("aftr_bringup")

    xacro_file = os.path.join(pkg_description, "urdf", "robot.urdf.xacro")
    doc = xacro.process_file(xacro_file, mappings={"use_sim": "false"})
    robot_description = {"robot_description": doc.toxml()}

    controller_params_file = os.path.join(pkg_bringup, "config", "md_controllers.yaml")
    lidar_params_file = os.path.join(pkg_bringup, "config", "sllidar_params.yaml")
    filter_params_file = os.path.join(pkg_bringup, "config", "laser_filter_params.yaml")

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/diff_drive_controller/odom", "/odom"),
        ],
        output="screen",
    )

    node_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )

    node_diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    # Start the diff-drive spawner only after the joint-state broadcaster is ready.
    delay_diff_drive_spawner_after_joint_state_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_joint_state_broadcaster_spawner,
            on_exit=[node_diff_drive_controller_spawner],
        )
    )

    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[lidar_params_file],
        remappings=[("/scan", "/scan_unfiltered")],
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_filter",
        parameters=[filter_params_file],
        remappings=[
            ("/scan", "/scan_unfiltered"),
            ("/scan_filtered", "/scan"),
        ],
    )

    # Start the LiDAR driver first so the filter can resolve its scan input and TF.
    delayed_laser_filter_node = TimerAction(
        period=1.0,
        actions=[laser_filter_node],
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            node_controller_manager,
            node_joint_state_broadcaster_spawner,
            delay_diff_drive_spawner_after_joint_state_spawner,
            lidar_node,
            delayed_laser_filter_node,
        ]
    )
