# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""ROS service callback bridge for bringup-related commands.

This module is intentionally thin. It exposes ROS service callbacks that apply
the shared exclusive-command guard, then delegate the real work to
``BringupRuntimeMixin`` and ``Nav2RuntimeMixin``.
"""

from aftr_mode_manager.runtime.bringup_runtime import BringupRuntimeMixin
from aftr_mode_manager.runtime.nav2_runtime import Nav2RuntimeMixin


class BringupManagerMixin(
    BringupRuntimeMixin,
    Nav2RuntimeMixin,
):
    """Expose bringup operations and Nav2 helpers behind callback methods."""

    def start_base_callback(self, request, response):
        """Handle the public request to start base bringup."""
        return self.run_exclusive_command(
            response,
            "start_base",
            self.start_base,
        )

    def check_base_ready_callback(self, request, response):
        """Handle the public request to validate base readiness."""
        return self.run_exclusive_command(
            response,
            "check_base_ready",
            self.check_base_ready,
        )

    def stop_base_callback(self, request, response):
        """Handle the public request to stop base bringup."""
        return self.run_exclusive_command(
            response,
            "stop_base",
            self.stop_base,
        )

    def start_slam_callback(self, request, response):
        """Handle the public request to start SLAM."""
        return self.run_exclusive_command(
            response,
            "start_slam",
            self.start_slam,
        )

    def check_slam_ready_callback(self, request, response):
        """Handle the public request to validate SLAM readiness."""
        return self.run_exclusive_command(
            response,
            "check_slam_ready",
            self.check_slam_ready,
        )

    def stop_slam_callback(self, request, response):
        """Handle the public request to stop SLAM."""
        return self.run_exclusive_command(
            response,
            "stop_slam",
            self.stop_slam,
        )

    def start_path_manager_callback(self, request, response):
        """Handle the public request to start ``path_manager``."""
        return self.run_exclusive_command(
            response,
            "start_path_manager",
            self.start_path_manager,
        )

    def check_path_ready_callback(self, request, response):
        """Handle the public request to validate ``path_manager`` readiness."""
        return self.run_exclusive_command(
            response,
            "check_path_ready",
            self.check_path_ready,
        )

    def stop_path_manager_callback(self, request, response):
        """Handle the public request to stop ``path_manager``."""
        return self.run_exclusive_command(
            response,
            "stop_path_manager",
            self.stop_path_manager,
        )

    def start_follower_node_callback(self, request, response):
        """Handle the public request to start the person follower."""
        return self.run_exclusive_command(
            response,
            "start_follower_node",
            self.start_follower_node,
        )

    def check_follower_ready_callback(self, request, response):
        """Handle the public request to validate follower readiness."""
        return self.run_exclusive_command(
            response,
            "check_follower_ready",
            self.check_follower_ready,
        )

    def stop_follower_node_callback(self, request, response):
        """Handle the public request to stop the person follower."""
        return self.run_exclusive_command(
            response,
            "stop_follower_node",
            self.stop_follower_node,
        )

    def start_nav2_callback(self, request, response):
        """Handle the public request to start Nav2."""
        return self.run_exclusive_command(
            response,
            "start_nav2",
            self.start_nav2,
        )

    def check_nav2_ready_callback(self, request, response):
        """Handle the public request to validate Nav2 readiness."""
        return self.run_exclusive_command(
            response,
            "check_nav2_ready",
            self.check_nav2_ready,
        )

    def stop_nav2_callback(self, request, response):
        """Handle the public request to stop Nav2."""
        return self.run_exclusive_command(
            response,
            "stop_nav2",
            self.stop_nav2,
        )

    def start_rviz_callback(self, request, response):
        """Handle the public request to start RViz."""
        return self.run_exclusive_command(
            response,
            "start_rviz",
            self.start_rviz,
        )

    def stop_rviz_callback(self, request, response):
        """Handle the public request to stop the managed RViz process."""
        return self.run_exclusive_command(
            response,
            "stop_rviz",
            self.stop_rviz,
        )
