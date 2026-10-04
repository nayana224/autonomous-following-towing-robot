# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Bringup command callbacks and startup runtime guards.

This module owns bringup-facing command callbacks, stale-runtime startup
checks, and the bridge to ``BringupRuntimeMixin`` and ``Nav2RuntimeMixin``.
"""

import rclpy

from aftr_mode_manager.runtime.bringup_runtime import BringupRuntimeMixin
from aftr_mode_manager.runtime.nav2_runtime import Nav2RuntimeMixin
from aftr_mode_manager.runtime.process_supervisor import (
    cleanup_registered_process_groups,
)


class BringupManagerMixin(
    BringupRuntimeMixin,
    Nav2RuntimeMixin,
):
    """Own bringup callbacks and startup guards for managed runtime processes."""

    MANAGED_RUNTIME_NODE_NAMES = {
        "/controller_manager",
        "/diff_drive_controller",
        "/joint_state_broadcaster",
        "/laser_filter",
        "/robot_state_publisher",
        "/sllidar_node",
        "/slam_toolbox",
        "/path_manager",
        "/person_follower",
        "/amcl",
        "/map_server",
        "/controller_server",
        "/planner_server",
        "/smoother_server",
        "/behavior_server",
        "/bt_navigator",
        "/waypoint_follower",
        "/velocity_smoother",
        "/lifecycle_manager_localization",
        "/lifecycle_manager_navigation",
        "/local_costmap/local_costmap",
        "/global_costmap/global_costmap",
    }

    def visible_node_names(self):
        """Return fully qualified ROS node names visible in the current graph."""
        names = []
        for node_name, namespace in self.get_node_names_and_namespaces():
            if namespace == "/":
                names.append(f"/{node_name}")
            else:
                names.append(f"{namespace.rstrip('/')}/{node_name}")
        return names

    def count_visible_node(self, fully_qualified_name):
        """Count how many graph-visible nodes match one fully qualified name."""
        return self.visible_node_names().count(fully_qualified_name)

    def audit_stale_runtime_nodes_once(self):
        """Serialize stale-runtime checks in the reentrant ROS executor."""
        if not self.stale_runtime_audit_lock.acquire(blocking=False):
            return
        try:
            self._audit_stale_runtime_nodes_once()
        finally:
            self.stale_runtime_audit_lock.release()

    def _audit_stale_runtime_nodes_once(self):
        """Confirm, clean, and recheck descendants from an older manager."""
        if (
            self.cleanup_stale_runtime_processes
            and not self.stale_runtime_cleanup_attempted
        ):
            self.stale_runtime_cleanup_attempted = True
            cleanup_ok, cleanup_messages = cleanup_registered_process_groups(
                self.managed_process_registry_dir,
                timeout_sec=self.stale_runtime_cleanup_timeout_sec,
            )
            stopped_process = False
            for cleanup_message in cleanup_messages:
                stopped_process = stopped_process or cleanup_message.startswith(
                    "stopped stale managed process"
                )
                if cleanup_message.startswith("stopped stale managed process"):
                    self.get_logger().warning(cleanup_message)
                elif cleanup_ok:
                    self.get_logger().info(cleanup_message)
                else:
                    self.get_logger().error(cleanup_message)
            if stopped_process:
                # Wait for DDS discovery to remove nodes from the stopped group.
                self.stale_runtime_audit_count = 0
                return

        visible = set(self.visible_node_names())
        stale_nodes = sorted(visible & self.MANAGED_RUNTIME_NODE_NAMES)
        if not stale_nodes:
            self.finish_stale_runtime_audit()
            return

        self.stale_runtime_audit_count += 1
        attempts = max(1, int(self.stale_runtime_audit_attempts))
        if self.stale_runtime_audit_count < attempts:
            self.get_logger().warning(
                "stale managed-runtime candidates are still visible; "
                f"confirming again ({self.stale_runtime_audit_count}/{attempts}): "
                + ", ".join(stale_nodes)
            )
            return

        message = (
            "refusing startup because nodes from an older operator runtime are "
            "still visible: " + ", ".join(stale_nodes)
        )
        self.status.last_error = message
        self.status.busy = False
        self.get_logger().fatal(message)
        self.publish_status()
        # Exiting mode_manager causes operator_system.launch.py to shut down all
        # of the new launch's sibling processes as well.
        rclpy.shutdown()

    def finish_stale_runtime_audit(self):
        """Continue startup after stale nodes disappear or cleanup succeeds."""
        if self.stale_runtime_audit_timer is not None:
            self.stale_runtime_audit_timer.cancel()
            self.destroy_timer(self.stale_runtime_audit_timer)
            self.stale_runtime_audit_timer = None
        self.get_logger().info(
            "stale managed-runtime audit passed "
            f"after {self.stale_runtime_audit_count} confirmation(s)"
        )
        if self.auto_start_base:
            self.schedule_auto_start_base(delay_sec=0.2)

    def schedule_auto_start_base(self, delay_sec):
        """Schedule base startup once after the stale-runtime gate passes."""
        if self.auto_start_base_timer is not None:
            return
        self.auto_start_base_timer = self.create_timer(
            max(0.1, float(delay_sec)),
            self.auto_start_base_once,
            callback_group=self.callback_group,
        )

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
