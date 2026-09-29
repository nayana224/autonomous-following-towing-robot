# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch SLAM with the configured scan and map parameters."""
    slam_toolbox_dir = FindPackageShare("slam_toolbox")
    aftr_slam_dir = FindPackageShare("aftr_slam")

    slam_params_file = PathJoinSubstitution(
        [aftr_slam_dir, "config", "mapper_params_online_async.yaml"]
    )

    start_async_slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_toolbox_dir, "launch", "online_async_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "false",  # Use wall-clock time on real hardware.
            "slam_params_file": slam_params_file,
        }.items(),
    )

    return LaunchDescription([start_async_slam_toolbox_node])
