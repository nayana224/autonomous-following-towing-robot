# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Nav2 bringup and readiness helpers for the mode manager.

This module owns the launch/start/stop flow for the Nav2 stack and provides
the readiness checks needed by localization and autonomous replay steps.
"""

import time

from lifecycle_msgs.srv import GetState


class Nav2RuntimeMixin:
    """Provide start, stop, and readiness checks for Nav2 runtime."""

    def log_nav2_stop_request(self, reason):
        """Record who requested Nav2 shutdown without changing stop behavior."""
        self.get_logger().warning(
            f"Nav2 stop requested: reason={reason}, mode={self._mode_name()}, "
            f"active_command={self.active_command_name or 'none'}, "
            f"safety_reason={getattr(self.status, 'safety_reason', '') or 'none'}; "
            + self.nav2_process.describe()
        )

    def start_nav2(self, response):
        """Start Nav2 and wait until the full Nav2 stack is ready."""
        self.status.busy = True
        self.status.last_command = "start_nav2"

        if not self.ensure_base_running(response):
            return response

        if self.nav2_process.is_running():
            if self.status.nav2_ready and not self.nav2_missing_dependencies():
                self.status.busy = False
                response.success = True
                response.message = "Nav2 is already running and ready"
                self.publish_status()
                return response

            self.status.busy = False
            response.success = True
            response.message = "Nav2 is already running"
            self.publish_status()
            return self.check_nav2_ready(response)

        if not self.start_nav2_process_only(response):
            return response
        return self.check_nav2_ready(response)

    def start_nav2_process_only(self, response):
        """Start only the Nav2 launch process without full readiness checks."""
        try:
            self.nav2_process.start()
        except OSError as exc:
            self.fail_response(response, f"failed to start Nav2: {exc}")
            return False

        self.status.busy = False
        self.status.nav2_running = True
        response.success = True
        response.message = "Nav2 started"
        self.get_logger().info(
            f"{response.message}; {self.nav2_process.describe()}"
        )
        self.publish_status()
        return True

    def check_nav2_localization_ready(self, response):
        """Wait only for the localization subset needed before initial pose use.

        This check is intentionally narrower than ``check_nav2_ready`` because
        localization startup may need to publish the saved initial pose before
        the whole navigation stack becomes fully usable.
        """
        self.status.busy = True
        self.status.last_command = "check_nav2_localization_ready"

        if not self.nav2_process.is_running():
            return self.fail_response(
                response,
                "Nav2 is not running; " + self.nav2_process.describe(),
            )

        topics_ready, missing_topics = self.nav2_readiness.wait_until_ready(
            self.nav2_ready_timeout_sec,
        )
        nodes_ready, missing_nodes = self.wait_for_nav2_nodes(
            self.nav2_ready_timeout_sec,
            [
                "/amcl",
                "/map_server",
                "/lifecycle_manager_localization",
            ],
        )
        lifecycle_ready, inactive_services = self.wait_for_nav2_lifecycle_active(
            self.nav2_lifecycle_ready_timeout_sec,
            self.nav2_localization_lifecycle_service_names,
        )
        if not topics_ready or not nodes_ready or not lifecycle_ready:
            message = self.nav2_not_ready_message(
                missing_topics,
                missing_nodes,
                inactive_services,
            )
            self.status.nav2_running = self.nav2_process.is_running()
            self.status.nav2_ready = False
            self.status.amcl_pose_ready = False
            self.status.busy = False
            response.success = False
            response.message = message
            self.get_logger().warn(message)
            self.get_logger().warning(
                "Nav2 localization readiness snapshot: "
                + self.nav2_process.describe()
            )
            self.publish_status()
            return response

        self.status.busy = False
        self.status.nav2_ready = True
        response.success = True
        response.message = "Nav2 localization is ready"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def wait_for_nav2_nodes(self, timeout_sec, required_nodes):
        """Wait for selected Nav2 nodes to appear in the ROS graph.

        Args:
            timeout_sec: Maximum wait duration in seconds.
            required_nodes: Iterable of fully qualified node names.

        Returns:
            Tuple ``(is_ready, missing_nodes)``.
        """
        deadline = time.monotonic() + float(timeout_sec)
        required_nodes = tuple(required_nodes)

        while time.monotonic() < deadline:
            visible_nodes = self.nav2_node_readiness.visible_nodes()
            missing_nodes = [
                node_name
                for node_name in required_nodes
                if node_name not in visible_nodes
            ]
            if not missing_nodes:
                return True, []
            time.sleep(0.2)

        visible_nodes = self.nav2_node_readiness.visible_nodes()
        return False, [
            node_name
            for node_name in required_nodes
            if node_name not in visible_nodes
        ]

    def check_nav2_ready(self, response):
        """Wait for the full Nav2 runtime to become ready."""
        self.status.busy = True
        self.status.last_command = "check_nav2_ready"

        if not self.nav2_process.is_running():
            return self.fail_response(
                response,
                "Nav2 is not running; " + self.nav2_process.describe(),
            )

        topics_ready, missing_topics = self.nav2_readiness.wait_until_ready(
            self.nav2_ready_timeout_sec,
        )
        nodes_ready, missing_nodes = self.nav2_node_readiness.wait_until_ready(
            self.nav2_ready_timeout_sec,
        )
        lifecycle_ready, inactive_services = self.wait_for_nav2_lifecycle_active(
            self.nav2_lifecycle_ready_timeout_sec,
            self.nav2_lifecycle_service_names,
        )
        if not topics_ready or not nodes_ready or not lifecycle_ready:
            message = self.nav2_not_ready_message(
                missing_topics,
                missing_nodes,
                inactive_services,
            )
            self.status.nav2_running = self.nav2_process.is_running()
            self.status.nav2_ready = False
            self.status.amcl_pose_ready = False
            self.status.busy = False
            response.success = False
            response.message = message
            self.get_logger().warn(message)
            self.get_logger().warning(
                "Nav2 full readiness snapshot: " + self.nav2_process.describe()
            )
            self.publish_status()
            return response

        self.status.busy = False
        self.status.nav2_ready = True
        response.success = True
        response.message = "Nav2 is ready"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def inactive_nav2_lifecycle_services_now(self):
        """Return lifecycle services that are not active right now.

        This helper performs a single non-blocking style snapshot instead of
        waiting for the full Nav2 startup timeout. It is used for lightweight
        status promotion while the operator remains on the localization page.
        """
        inactive_services = []
        for service_name in self.nav2_lifecycle_service_names:
            client = self.nav2_lifecycle_clients[service_name]
            state_label = self.get_lifecycle_state_label(
                service_name,
                client,
                timeout_sec=0.3,
            )
            if state_label != "active":
                inactive_services.append(f"{service_name}={state_label}")
        return inactive_services

    def nav2_ready_now(self):
        """Return whether Nav2 appears fully ready from a single snapshot."""
        if not self.nav2_process.is_running():
            return False
        if self.nav2_readiness.missing_topics():
            return False
        if self.nav2_node_readiness.missing_nodes():
            return False
        if self.amcl_pose_readiness.missing_topics():
            return False
        if not self.amcl_pose_message_readiness.has_message():
            return False
        if not self.localization_transform_ready(timeout_sec=0.05):
            return False
        return not self.inactive_nav2_lifecycle_services_now()

    def autonomous_readiness_issues_now(self):
        """Return fast, live readiness failures that block driving startup."""
        issues = []
        if not self.nav2_process.is_running():
            issues.append("Nav2 process is not running")
            issues.append(self.nav2_process.describe())
            return issues

        issues.extend(
            f"missing topic {name}" for name in self.nav2_readiness.missing_topics()
        )
        issues.extend(
            f"missing node {name}" for name in self.nav2_node_readiness.missing_nodes()
        )
        issues.extend(
            f"missing topic {name}"
            for name in self.amcl_pose_readiness.missing_topics()
        )
        if not self.amcl_pose_message_readiness.has_message():
            issues.append("/amcl_pose has not been received for this localization")
        if not self.localization_transform_ready(timeout_sec=0.05):
            issues.append("transform map -> base_footprint is unavailable")
        return issues

    def wait_for_stable_autonomous_readiness(self, response):
        """Require live localization inputs to remain ready before driving."""
        deadline = time.monotonic() + float(self.nav2_stable_ready_timeout_sec)
        stable_since = None
        last_issues = []

        while time.monotonic() < deadline:
            last_issues = self.autonomous_readiness_issues_now()
            if last_issues:
                stable_since = None
            else:
                if stable_since is None:
                    stable_since = time.monotonic()
                if (
                    time.monotonic() - stable_since
                    >= float(self.nav2_stable_ready_duration_sec)
                ):
                    inactive = self.inactive_nav2_lifecycle_services_now()
                    if not inactive:
                        self.get_logger().info(
                            "Nav2 autonomous readiness remained stable for "
                            f"{self.nav2_stable_ready_duration_sec:.1f}s"
                        )
                        return True
                    last_issues = [
                        "inactive lifecycle nodes: " + ", ".join(inactive)
                    ]
                    stable_since = None
            time.sleep(0.2)

        detail = "; ".join(last_issues) if last_issues else (
            "readiness did not remain continuously stable"
        )
        self.status.nav2_ready = False
        self.status.amcl_pose_ready = False
        self.get_logger().warning(
            "autonomous entry blocked by Nav2 readiness: "
            f"{detail}; {self.nav2_process.describe()}"
        )
        self.fail_response(
            response,
            "Nav2 is not stably ready for autonomous driving; " + detail,
        )
        return False

    def wait_for_nav2_lifecycle_active(self, timeout_sec, service_names):
        """Wait until selected Nav2 lifecycle services report ``active``.

        Args:
            timeout_sec: Maximum wait duration in seconds.
            service_names: Iterable of lifecycle ``get_state`` service names.

        Returns:
            Tuple ``(is_ready, inactive_services)``.
        """
        deadline = time.monotonic() + float(timeout_sec)
        inactive_services = list(service_names)
        last_log_time = 0.0

        while time.monotonic() < deadline:
            inactive_services = []
            for service_name in service_names:
                client = self.nav2_lifecycle_clients[service_name]
                state_label = self.get_lifecycle_state_label(
                    service_name,
                    client,
                    timeout_sec=1.0,
                )
                if state_label != "active":
                    inactive_services.append(f"{service_name}={state_label}")

            if not inactive_services:
                return True, []

            now = time.monotonic()
            if now - last_log_time > 5.0:
                self.get_logger().info(
                    "waiting for Nav2 lifecycle active: "
                    + ", ".join(inactive_services)
                )
                last_log_time = now

            time.sleep(0.2)

        return False, inactive_services

    def get_lifecycle_state_label(self, service_name, client, timeout_sec):
        """Return one lifecycle node state label or an availability error label."""
        if not client.wait_for_service(timeout_sec=0.2):
            return "service_unavailable"

        future = client.call_async(GetState.Request())
        if not self.wait_for_future(future, timeout_sec):
            return "timeout"

        result = future.result()
        if result is None:
            return "no_response"

        label = result.current_state.label
        if label:
            return label

        return f"id_{result.current_state.id}"

    def nav2_missing_dependencies(self):
        """Return missing topics and nodes for full Nav2 readiness checks."""
        missing = []
        missing.extend(
            f"topic {topic_name}"
            for topic_name in self.nav2_readiness.missing_topics()
        )
        missing.extend(
            f"node {node_name}"
            for node_name in self.nav2_node_readiness.missing_nodes()
        )
        return missing

    def nav2_missing_localization(self):
        """Return missing dependencies needed before publishing initial pose."""
        visible_nodes = self.nav2_node_readiness.visible_nodes()
        required_nodes = {
            "/amcl",
            "/map_server",
            "/lifecycle_manager_localization",
        }
        missing = []
        missing.extend(
            f"topic {topic_name}"
            for topic_name in self.nav2_readiness.missing_topics()
        )
        missing.extend(
            f"node {node_name}"
            for node_name in required_nodes
            if node_name not in visible_nodes
        )
        return missing

    def nav2_not_ready_message(
        self,
        missing_topics,
        missing_nodes,
        inactive_services,
    ):
        """Build one readable Nav2 readiness failure message."""
        parts = []
        if missing_topics:
            parts.append("missing topics: " + ", ".join(missing_topics))
        if missing_nodes:
            parts.append("missing nodes: " + ", ".join(missing_nodes))
        if inactive_services:
            parts.append("inactive lifecycle nodes: " + ", ".join(inactive_services))
        if not parts:
            parts.append("graph was not ready before timeout")
        return "Nav2 is not ready; " + "; ".join(parts)

    def stop_nav2(self, response):
        """Stop Nav2 without shutting down base bringup."""
        self.status.busy = True
        self.status.last_command = "stop_nav2"
        self.log_nav2_stop_request("stop_nav2 command")

        if not self.nav2_process.stop(self.shutdown_timeout_sec):
            return self.fail_response(response, "Nav2 shutdown timed out")

        self.status.busy = False
        self.status.nav2_running = False
        self.status.nav2_ready = False
        self.status.amcl_pose_ready = False
        response.success = True
        response.message = "Nav2 stopped"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def start_nav2_for_localization(self, response):
        """Start Nav2 and wait only for localization-related readiness."""
        if self.nav2_process.is_running():
            self.status.nav2_running = True
            self.get_logger().info(
                "reusing existing Nav2 process for localization readiness check; "
                + self.nav2_process.describe()
            )
            if self.status.nav2_ready and not self.nav2_missing_localization():
                return True
            ready_response = self.check_nav2_localization_ready(response)
            return ready_response.success

        if not self.start_nav2_process_only(response):
            return False
        ready_response = self.check_nav2_localization_ready(response)
        return ready_response.success
