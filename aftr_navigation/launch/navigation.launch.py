# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Launch separate Nav2 processes for lifecycle monitoring."""
    pkg_nav2 = get_package_share_directory("nav2_bringup")
    pkg_aftr_nav = get_package_share_directory("aftr_navigation")

    map_file = os.path.expanduser("~/map/mdbot_map.yaml")
    params_file = os.path.join(pkg_aftr_nav, "config", "nav2_params.yaml")

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": "False",  # Use wall-clock time on real hardware.
            "params_file": params_file,
            "autostart": "True",
            # Keep Nav2 processes separate because the mode manager monitors their lifecycle states.
            # Separate processes also avoid load_node/InvalidHandle warnings on shutdown.
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription([nav2_bringup_launch])
