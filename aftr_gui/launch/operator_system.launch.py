# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Launch the complete AFTR operator system with fall safety enabled."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the operator GUI, safety manager, camera, and fall detector."""
    enable_fall_camera = LaunchConfiguration("enable_fall_camera")
    enable_fall_detection = LaunchConfiguration("enable_fall_detection")
    show_fall_image = LaunchConfiguration("show_fall_image")
    robust_tracking = LaunchConfiguration("robust_tracking")
    auto_start_base = LaunchConfiguration("auto_start_base")

    fall_detection_share = get_package_share_directory("aftr_fall_detection")
    fall_detection_params = os.path.join(
        fall_detection_share,
        "config",
        "fall_detection.yaml",
    )

    mode_manager = Node(
        package="aftr_mode_manager",
        executable="mode_manager",
        name="mode_manager",
        output="screen",
        parameters=[
            {
                "auto_start_base": ParameterValue(auto_start_base, value_type=bool),
                "robust_tracking": robust_tracking,
                "fall_detection_required": ParameterValue(
                    enable_fall_detection,
                    value_type=bool,
                ),
            }
        ],
    )

    realsense_camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        condition=IfCondition(enable_fall_camera),
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            {
                "enable_color": True,
                "enable_depth": False,
                "enable_gyro": False,
                "enable_accel": False,
                "rgb_camera.color_profile": "640x480x15",
                "wait_for_device_timeout": -1.0,
                "reconnect_timeout": 3.0,
            }
        ],
    )

    fall_detection_node = Node(
        package="aftr_fall_detection",
        executable="fall_detection_node",
        name="fall_detection",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(enable_fall_detection),
        parameters=[
            fall_detection_params,
            {
                "show_image": show_fall_image,
            },
        ],
    )

    delayed_fall_detection = TimerAction(
        period=2.0,
        actions=[fall_detection_node],
    )

    operator_gui = Node(
        package="aftr_gui",
        executable="operator_gui",
        name="operator_gui",
        output="screen",
    )

    aftr_audio = Node(
        package="aftr_audio",
        executable="audio_node",
        name="mdbot_audio",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    aftr_status_led = Node(
        package="aftr_status_led",
        executable="status_led_node",
        name="mdbot_status_led",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    close_launch_when_gui_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=operator_gui,
            on_exit=[EmitEvent(event=Shutdown(reason="operator GUI closed"))],
        )
    )
    close_launch_when_manager_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=mode_manager,
            on_exit=[EmitEvent(event=Shutdown(reason="mode manager exited"))],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_fall_camera",
                default_value="true",
                description="Start the RealSense color camera used for fall detection.",
            ),
            DeclareLaunchArgument(
                "enable_fall_detection",
                default_value="true",
                description="Start the CUDA fall-detection node.",
            ),
            DeclareLaunchArgument(
                "show_fall_image",
                default_value="false",
                description="Show the OpenCV fall-detection debug window.",
            ),
            DeclareLaunchArgument(
                "auto_start_base",
                default_value="true",
                description="Automatically start base hardware after the operator GUI opens.",
            ),
            DeclareLaunchArgument(
                "robust_tracking",
                default_value="false",
                description="Use conservative worker target identity locking.",
            ),
            mode_manager,
            realsense_camera,
            delayed_fall_detection,
            operator_gui,
            aftr_audio,
            aftr_status_led,
            close_launch_when_gui_exits,
            close_launch_when_manager_exits,
        ]
    )
