# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Runtime bringup helpers for non-Nav2 managed processes.

This module contains the start/check/stop logic for base bringup, SLAM,
``path_manager``, person following, and RViz. It focuses on process lifecycle
handling and readiness confirmation rather than on high-level workflow rules.
"""

import os
import subprocess
import time

from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Trigger

from aftr_mode_manager.mode_state import RobotMode


class BringupRuntimeMixin:
    """Provide process bringup, readiness, and cleanup helpers."""

    def _start_runtime_process(
        self,
        response,
        *,
        command_name,
        process,
        running_attr,
        ready_attr,
        process_label,
        check_method,
    ):
        """Start one managed process, then run its readiness check.

        Args:
            response: Trigger response returned to the caller.
            command_name: Command name recorded in workflow status.
            process: Managed process instance to start.
            running_attr: Status attribute set when the process is running.
            ready_attr: Status attribute set when readiness succeeds.
            process_label: Human-readable process label used in logs.
            check_method: Bound readiness-check method for this process.

        Returns:
            Updated response after startup or readiness validation.
        """
        self.status.busy = True
        self.status.last_command = command_name

        if process.is_running():
            if getattr(self.status, ready_attr):
                self.status.busy = False
                response.success = True
                response.message = f"{process_label} is already running and ready"
                self.publish_status()
                return response

            self.status.busy = False
            response.success = True
            response.message = f"{process_label} is already running"
            self.publish_status()
            return check_method(response)

        try:
            process.start()
        except OSError as exc:
            return self.fail_response(
                response,
                f"failed to start {process_label}: {exc}",
            )

        self.status.busy = False
        setattr(self.status, running_attr, True)
        setattr(self.status, ready_attr, False)
        response.success = True
        response.message = f"{process_label} started"
        self.get_logger().info(response.message)
        self.publish_status()
        return check_method(response)

    def _check_runtime_readiness(
        self,
        response,
        *,
        command_name,
        process,
        running_attr,
        ready_attr,
        process_label,
        readiness,
        timeout_sec,
        not_running_message,
    ):
        """Wait until a managed process exposes its required readiness topics."""
        self.status.busy = True
        self.status.last_command = command_name

        if not process.is_running():
            return self.fail_response(response, not_running_message)

        is_ready, missing_topics = readiness.wait_until_ready(timeout_sec)
        if not is_ready:
            process.stop(self.shutdown_timeout_sec)
            setattr(self.status, running_attr, False)
            setattr(self.status, ready_attr, False)
            message = f"{process_label} is not ready; missing topics: "
            return self.fail_response(response, message + ", ".join(missing_topics))

        self.status.busy = False
        setattr(self.status, running_attr, True)
        setattr(self.status, ready_attr, True)
        response.success = True
        response.message = f"{process_label} is ready"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def _stop_runtime_process(
        self,
        response,
        *,
        command_name,
        process,
        running_attr,
        ready_attr,
        process_label,
        timeout_message,
    ):
        """Stop one managed process and clear its cached readiness state."""
        self.status.busy = True
        self.status.last_command = command_name

        if not process.stop(self.shutdown_timeout_sec):
            return self.fail_response(response, timeout_message)

        self.status.busy = False
        setattr(self.status, running_attr, False)
        setattr(self.status, ready_attr, False)
        response.success = True
        response.message = f"{process_label} stopped"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def _ensure_runtime_process(
        self,
        response,
        *,
        process,
        running_attr,
        ready_attr,
        readiness,
        check_method,
        start_method,
    ):
        """Ensure a managed process is running and still appears ready."""
        if process.is_running():
            setattr(self.status, running_attr, True)
            if getattr(self.status, ready_attr) and not readiness.missing_topics():
                return True
            ready_response = check_method(response)
            return ready_response.success

        start_response = start_method(response)
        return start_response.success

    def _visible_path_manager_count(self, *, warn_if_multiple=False):
        """Return how many ``/path_manager`` nodes are visible in the graph."""
        visible_count = self.count_visible_node("/path_manager")
        if warn_if_multiple and visible_count > 1:
            self.get_logger().warn(
                f"detected {visible_count} visible /path_manager nodes"
            )
        return visible_count

    def _has_unmanaged_visible_path_manager(self):
        """Return whether a visible ``/path_manager`` exists outside this manager.

        The mode manager must only trust the path manager process that it started
        itself. If a visible ``/path_manager`` node exists while the owned
        ``ManagedProcess`` is not running, it is most likely a stale process from
        an older launch or another shell session.
        """
        return (
            self._visible_path_manager_count(warn_if_multiple=True) > 0
            and not self.path_process.is_running()
        )

    def _finish_recording_mode_guard(self, response):
        """Validate whether recording can transition into alignment flow."""
        if self.status.mode == RobotMode.RECORDING_FOLLOW:
            return True

        if self.status.mode == RobotMode.ALIGNMENT:
            self.status.busy = False
            response.success = True
            response.message = "already in ALIGNMENT"
            self.get_logger().info(response.message)
            self.publish_status()
            return False

        self.reject_response(
            response,
            "finish_recording_for_alignment requires RECORDING_FOLLOW mode",
        )
        return False

    def _stop_follower_for_alignment(self, response):
        """Stop the follower before saving recorded path data."""
        if not self.follower_node.stop(self.shutdown_timeout_sec):
            self.fail_response(response, "follower node shutdown timed out")
            return False

        self.status.follower_running = False
        self.status.follower_ready = False
        return True

    def _save_recording_outputs_for_alignment(self, response):
        """Stop path recording, save the map, and save the alignment pose."""
        self.set_operator_message(
            "경로 저장을 마무리하고 있습니다.",
            publish=True,
        )
        if not self.call_trigger_service(
            self.path_stop_record_client,
            "/path_manager/stop_record",
            timeout_sec=10.0,
        ):
            self.fail_response(response, "failed to stop path recording")
            return False

        self.set_operator_message(
            "지도를 저장하고 있습니다.",
            publish=True,
        )
        if not self.save_slam_map():
            self.fail_response(response, "failed to save SLAM map")
            return False

        self.set_operator_message(
            "현재 위치를 저장하고 있습니다.",
            publish=True,
        )
        if not self.call_trigger_service(
            self.path_save_pose_client,
            "/path_manager/save_pose",
            timeout_sec=5.0,
        ):
            self.fail_response(response, "failed to save alignment pose")
            return False

        return True

    def _prepare_alignment_state(self):
        """Reset alignment-related runtime flags after recording finishes."""
        self.alignment_control_active = False
        self.next_drive_direction = "reverse"

    def _switch_to_alignment_and_start_localizing(self, response):
        """Enter alignment flow and start localization runtime."""
        alignment_response = Trigger.Response()
        self.request_mode(
            alignment_response,
            RobotMode.ALIGNMENT,
            "finish_recording_for_alignment",
        )
        if not alignment_response.success:
            self.copy_response(response, alignment_response)
            return False

        self.set_operator_message(
            "위치 추정을 시작하고 있습니다.",
            publish=True,
        )
        localization_response = Trigger.Response()
        self.start_localizing(localization_response)
        if not localization_response.success:
            self.copy_response(response, localization_response)
            return False

        self.request_mode(
            response,
            RobotMode.ALIGNMENT,
            "finish_recording_for_alignment",
        )
        if response.success:
            self.set_operator_message(
                "정렬 준비가 완료되었습니다.",
                publish=True,
            )
            self.schedule_alignment_confirmation_request_for(
                expected_last_commands=("finish_recording_for_alignment",),
                delay_sec=0.0,
            )
        return response.success

    def start_base(self, response):
        """Start base bringup and recover when a running base is unhealthy."""
        self.status.busy = True
        self.status.last_command = "start_base"

        if self.base_process.is_running():
            if self.status.base_ready and self.base_is_still_ready():
                self.status.busy = False
                response.success = True
                response.message = "base bringup is already running and ready"
                self.publish_status()
                return response

            self.status.busy = False
            response.success = True
            response.message = "base bringup is already running; checking readiness"
            self.publish_status()
            return self.check_base_ready(response)

        try:
            self.base_process.start()
        except OSError as exc:
            return self.fail_response(response, f"failed to start base bringup: {exc}")

        self.base_started_at = time.monotonic()
        self.status.busy = False
        self.status.base_running = True
        response.success = True
        response.message = "base bringup started"
        self.get_logger().info(response.message)
        self.publish_status()
        return self.check_base_ready(response)

    def check_base_ready(self, response):
        """Wait for base topics, scan messages, and controllers to become ready."""
        self.status.busy = True
        self.status.last_command = "check_base_ready"

        attempts = max(0, int(self.base_restart_attempts))
        last_missing_topics = []
        for attempt_index in range(attempts + 1):
            if not self.base_process.is_running():
                if not self.start_base_process_only():
                    return self.fail_response(response, "failed to start base bringup")

            self.wait_for_base_minimum_uptime()

            is_ready, missing_topics = self.base_readiness.wait_until_ready(
                self.base_ready_timeout_sec,
            )
            scan_has_messages = False
            controllers_ready = False
            if is_ready:
                scan_has_messages = self.scan_message_readiness.wait_for_new_message(
                    self.scan_message_timeout_sec,
                )
                if not scan_has_messages:
                    missing_topics = [*missing_topics, "/scan messages"]
                controllers_ready, missing_controllers = self.controllers_are_active(
                    self.controller_ready_timeout_sec,
                )
                if not controllers_ready:
                    missing_topics = [*missing_topics, *missing_controllers]

            if is_ready and scan_has_messages and controllers_ready:
                self.status.busy = False
                self.status.base_running = True
                self.status.base_ready = True
                response.success = True
                response.message = "base bringup is ready"
                if self.status.mode == RobotMode.IDLE:
                    self.notify_system_ready()
                self.get_logger().info(response.message)
                self.publish_status()
                return response

            last_missing_topics = missing_topics
            if attempt_index >= attempts:
                break

            self.get_logger().warn(
                "base bringup is not ready; restarting base "
                f"(missing topics: {', '.join(missing_topics)})"
            )
            if not self.restart_base_process_only():
                return self.fail_response(response, "base bringup restart timed out")

        message = "base bringup is not ready; missing topics: "
        self.stop_base_dependents()
        self.base_process.stop(self.shutdown_timeout_sec)
        self.status.base_running = False
        self.status.base_ready = False
        return self.fail_response(response, message + ", ".join(last_missing_topics))

    def controllers_are_active(self, timeout_sec):
        """Check whether required ros2_control controllers are active.

        Args:
            timeout_sec: Maximum wait duration for the controller service.

        Returns:
            Tuple ``(is_ready, missing_items)``.
        """
        required_controllers = {
            "joint_state_broadcaster",
            "diff_drive_controller",
        }
        if not self.controller_list_client.wait_for_service(timeout_sec=timeout_sec):
            return False, ["/controller_manager/list_controllers service"]

        future = self.controller_list_client.call_async(ListControllers.Request())
        if not self.wait_for_future(future, timeout_sec):
            return False, ["/controller_manager/list_controllers response"]

        result = future.result()
        if result is None:
            return False, ["/controller_manager/list_controllers result"]

        active_controllers = {
            controller.name
            for controller in result.controller
            if controller.state == "active"
        }
        missing = sorted(required_controllers - active_controllers)
        if missing:
            return False, [f"controller inactive: {name}" for name in missing]
        return True, []

    def base_is_still_ready(self):
        """Return whether cached base readiness still looks healthy."""
        if self.base_readiness.missing_topics():
            return False
        if not self.scan_message_readiness.has_recent_message(
            max(float(self.scan_message_timeout_sec), 2.0),
        ):
            return False
        controllers_ready, _missing_controllers = self.controllers_are_active(
            min(float(self.controller_ready_timeout_sec), 2.0),
        )
        return controllers_ready

    def stop_base(self, response):
        """Stop base bringup and its dependent upper-layer processes."""
        self.status.busy = True
        self.status.last_command = "stop_base"

        self.stop_base_dependents()
        if not self.base_process.stop(self.shutdown_timeout_sec):
            return self.fail_response(response, "base bringup shutdown timed out")

        self.status.busy = False
        self.status.base_running = False
        self.status.base_ready = False
        self.system_ready_audio_announced = False
        response.success = True
        response.message = "base bringup stopped"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def start_base_process_only(self):
        """Start only the base launch process without readiness validation."""
        try:
            self.base_process.start()
        except OSError as exc:
            self.get_logger().error(f"failed to start base bringup: {exc}")
            return False

        self.base_started_at = time.monotonic()
        self.status.base_running = True
        self.status.base_ready = False
        self.publish_status()
        return True

    def wait_for_base_minimum_uptime(self):
        """Wait for the minimum base uptime before readiness polling starts."""
        if self.base_started_at is None:
            return

        min_delay = max(0.0, float(self.base_min_ready_delay_sec))
        elapsed = time.monotonic() - self.base_started_at
        remaining = min_delay - elapsed
        if remaining > 0.0:
            time.sleep(remaining)

    def restart_base_process_only(self):
        """Restart base bringup after stopping processes that depend on it."""
        self.stop_base_dependents()

        if not self.base_process.stop(self.shutdown_timeout_sec):
            return False

        self.status.base_running = False
        self.status.base_ready = False
        self.publish_status()
        time.sleep(float(self.base_restart_delay_sec))
        return self.start_base_process_only()

    def stop_base_dependents(self):
        """Stop upper-layer processes before restarting base hardware bringup."""
        self.follower_node.stop(self.shutdown_timeout_sec)
        self.log_nav2_stop_request("base restart dependency cleanup")
        self.nav2_process.stop(self.shutdown_timeout_sec)
        self.slam_process.stop(self.shutdown_timeout_sec)
        self.status.follower_running = False
        self.status.follower_ready = False
        self.status.nav2_running = False
        self.status.nav2_ready = False
        self.status.amcl_pose_ready = False
        self.status.slam_running = False
        self.status.slam_ready = False

    def start_slam(self, response):
        """Start SLAM after ensuring base bringup is available."""
        if not self.ensure_base_running(response):
            return response

        return self._start_runtime_process(
            response,
            command_name="start_slam",
            process=self.slam_process,
            running_attr="slam_running",
            ready_attr="slam_ready",
            process_label="SLAM",
            check_method=self.check_slam_ready,
        )

    def check_slam_ready(self, response):
        """Wait for SLAM readiness before allowing mapping-dependent modes."""
        return self._check_runtime_readiness(
            response,
            command_name="check_slam_ready",
            process=self.slam_process,
            running_attr="slam_running",
            ready_attr="slam_ready",
            process_label="SLAM",
            readiness=self.slam_readiness,
            timeout_sec=self.slam_ready_timeout_sec,
            not_running_message="SLAM is not running",
        )

    def stop_slam(self, response):
        """Stop SLAM without shutting down the base bringup."""
        return self._stop_runtime_process(
            response,
            command_name="stop_slam",
            process=self.slam_process,
            running_attr="slam_running",
            ready_attr="slam_ready",
            process_label="SLAM",
            timeout_message="SLAM shutdown timed out",
        )

    def start_path_manager(self, response):
        """Start ``path_manager`` and wait for its status topic."""
        self.status.busy = True
        self.status.last_command = "start_path_manager"

        if not self.ensure_base_running(response):
            return response

        if self._has_unmanaged_visible_path_manager():
            return self.fail_response(
                response,
                "unmanaged /path_manager is already visible; stop the stale "
                "path_manager process before starting a new one",
            )

        return self._start_runtime_process(
            response,
            command_name="start_path_manager",
            process=self.path_process,
            running_attr="path_running",
            ready_attr="path_ready",
            process_label="path manager",
            check_method=self.check_path_ready,
        )

    def check_path_ready(self, response):
        """Wait for ``path_manager`` readiness before mapping follow mode."""
        self.status.busy = True
        self.status.last_command = "check_path_ready"

        visible_path_managers = self._visible_path_manager_count(warn_if_multiple=True)

        if self._has_unmanaged_visible_path_manager():
            self.status.path_running = True
            self.status.path_ready = False
            return self.fail_response(
                response,
                "visible /path_manager is not owned by this mode_manager; "
                "restart or stop the stale path_manager process first",
            )

        if not self.path_process.is_running() and visible_path_managers == 0:
            return self.fail_response(response, "path manager is not running")

        is_ready, missing_topics = self.path_readiness.wait_until_ready(
            self.path_ready_timeout_sec,
        )
        if not is_ready:
            message = "path manager is not ready; missing topics: "
            self.path_process.stop(self.shutdown_timeout_sec)
            self.status.path_running = False
            self.status.path_ready = False
            return self.fail_response(response, message + ", ".join(missing_topics))

        self.status.busy = False
        self.status.path_running = True
        self.status.path_ready = True
        response.success = True
        response.message = "path manager is ready"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def stop_path_manager(self, response):
        """Stop the managed ``path_manager`` process."""
        self.status.busy = True
        self.status.last_command = "stop_path_manager"

        if self.path_process.is_running() and not self.path_process.stop(
            self.shutdown_timeout_sec
        ):
            return self.fail_response(response, "path manager shutdown timed out")

        visible_path_managers = self._visible_path_manager_count()
        if visible_path_managers > 0:
            self.get_logger().warn(
                f"{visible_path_managers} unmanaged /path_manager node(s) still visible; "
                "they were not started by this mode_manager"
            )

        self.status.busy = False
        self.status.path_running = visible_path_managers > 0
        self.status.path_ready = False
        response.success = True
        response.message = (
            "path manager stopped"
            if visible_path_managers == 0
            else "owned path manager stopped; unmanaged path manager still visible"
        )
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def finish_recording_for_alignment(self, response):
        """Finish recording-follow and transition into alignment preparation."""
        self.status.busy = True
        self.status.last_command = "finish_recording_for_alignment"
        self.set_operator_message(
            "작업자 추종을 종료하고 있습니다.",
            publish=True,
        )

        if not self._finish_recording_mode_guard(response):
            return response

        if not self._stop_follower_for_alignment(response):
            return response

        if not self._save_recording_outputs_for_alignment(response):
            return response
        self.notify_recording_follow_completed()

        self._prepare_alignment_state()

        self._switch_to_alignment_and_start_localizing(response)
        return response

    def start_follower_node(self, response):
        """Start the person follower after ensuring base bringup is ready."""
        if not self.ensure_base_running(response):
            return response

        return self._start_runtime_process(
            response,
            command_name="start_follower_node",
            process=self.follower_node,
            running_attr="follower_running",
            ready_attr="follower_ready",
            process_label="follower node",
            check_method=self.check_follower_ready,
        )

    def check_follower_ready(self, response):
        """Wait for follower readiness before allowing follow modes."""
        return self._check_runtime_readiness(
            response,
            command_name="check_follower_ready",
            process=self.follower_node,
            running_attr="follower_running",
            ready_attr="follower_ready",
            process_label="follower node",
            readiness=self.follower_readiness,
            timeout_sec=self.follower_ready_timeout_sec,
            not_running_message="follower node is not running",
        )

    def stop_follower_node(self, response):
        """Stop follower node without shutting down base bringup."""
        return self._stop_runtime_process(
            response,
            command_name="stop_follower_node",
            process=self.follower_node,
            running_attr="follower_running",
            ready_attr="follower_ready",
            process_label="follower node",
            timeout_message="follower node shutdown timed out",
        )

    def start_rviz(self, response):
        """Start RViz using the navigation display configuration."""
        self.status.busy = True
        self.status.last_command = "start_rviz"

        if self.rviz_process.is_running():
            self.status.busy = False
            response.success = True
            response.message = "RViz is already running"
            self.get_logger().info(response.message)
            self.publish_status()
            return response

        if not os.path.exists(self.rviz_config_file):
            return self.fail_response(
                response,
                f"RViz config file does not exist: {self.rviz_config_file}",
            )

        try:
            self.rviz_process.start()
        except OSError as exc:
            return self.fail_response(response, f"failed to start RViz: {exc}")

        self.status.busy = False
        response.success = True
        response.message = f"RViz started with {self.rviz_config_file}"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def stop_rviz(self, response):
        """Stop RViz without changing robot mode."""
        self.status.busy = True
        self.status.last_command = "stop_rviz"

        if not self.rviz_process.stop(self.shutdown_timeout_sec):
            return self.fail_response(response, "RViz shutdown timed out")

        self.status.busy = False
        response.success = True
        response.message = "RViz stopped"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def ensure_base_running(self, response):
        """Start base bringup before a mode that requires robot hardware."""
        if self.base_process.is_running():
            self.status.base_running = True
            if self.status.base_ready and self.base_is_still_ready():
                return True
            ready_response = self.check_base_ready(response)
            return ready_response.success

        start_response = self.start_base(response)
        return start_response.success

    def ensure_slam_running(self, response):
        """Start SLAM before a mode that requires live mapping."""
        return self._ensure_runtime_process(
            response,
            process=self.slam_process,
            running_attr="slam_running",
            ready_attr="slam_ready",
            readiness=self.slam_readiness,
            check_method=self.check_slam_ready,
            start_method=self.start_slam,
        )

    def ensure_path_manager_running(self, response):
        """Start path manager before mapping follow mode."""
        if self._has_unmanaged_visible_path_manager():
            return self.fail_response(
                response,
                "stale /path_manager is visible; cannot safely reuse it",
            )

        visible_path_managers = self._visible_path_manager_count(warn_if_multiple=True)
        if self.path_process.is_running() or visible_path_managers > 0:
            self.status.path_running = True
            if self.status.path_ready and not self.path_readiness.missing_topics():
                return True
            ready_response = self.check_path_ready(response)
            return ready_response.success

        start_response = self.start_path_manager(response)
        return start_response.success

    def ensure_follower_node_running(self, response):
        """Start follower node before a follow mode."""
        return self._ensure_runtime_process(
            response,
            process=self.follower_node,
            running_attr="follower_running",
            ready_attr="follower_ready",
            readiness=self.follower_readiness,
            check_method=self.check_follower_ready,
            start_method=self.start_follower_node,
        )

    def save_slam_map(self):
        """Save the current /map topic to the configured map file prefix."""
        if self.slam_readiness.missing_topics():
            is_ready, missing_topics = self.slam_readiness.wait_until_ready(
                self.slam_ready_timeout_sec,
            )
            if not is_ready:
                self.get_logger().error(
                    "cannot save map; missing topics: " + ", ".join(missing_topics)
                )
                return False

        map_prefix = self.normalized_map_save_file()
        map_dir = os.path.dirname(map_prefix)
        if map_dir:
            os.makedirs(map_dir, exist_ok=True)

        command = [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            map_prefix,
        ]

        self.get_logger().info(f"saving SLAM map to {map_prefix}")
        try:
            env = os.environ.copy()
            env["ROS_LOG_DIR"] = "/tmp"
            result = subprocess.run(
                command,
                capture_output=True,
                check=False,
                text=True,
                timeout=float(self.map_save_timeout_sec),
                env=env,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error("map saver timed out")
            return False
        except OSError as exc:
            self.get_logger().error(f"failed to run map saver: {exc}")
            return False

        if result.stdout:
            self.get_logger().info(result.stdout.strip())
        if result.stderr:
            self.get_logger().warn(result.stderr.strip())

        if result.returncode != 0:
            self.get_logger().error(
                f"map saver failed with exit code {result.returncode}"
            )
            return False

        self.get_logger().info(f"SLAM map saved to {map_prefix}")
        return True

    def normalized_map_save_file(self):
        """Return a map prefix without .yaml/.pgm extension."""
        map_file = str(self.map_save_file)
        for suffix in (".yaml", ".pgm"):
            if map_file.endswith(suffix):
                return map_file[: -len(suffix)]
        return map_file

    def shutdown_processes(self):
        """Best-effort cleanup used when the node itself is shutting down."""
        failures = []
        if not self.rviz_process.stop(self.shutdown_timeout_sec):
            failures.append(self.rviz_process.describe())
        if not self.follower_node.stop(self.shutdown_timeout_sec):
            failures.append(self.follower_node.describe())
        self.log_nav2_stop_request("mode_manager process shutdown")
        for process in (
            self.nav2_process,
            self.path_process,
            self.slam_process,
            self.base_process,
        ):
            if not process.stop(self.shutdown_timeout_sec):
                failures.append(process.describe())

        if failures:
            self.get_logger().error(
                "managed process groups survived full shutdown escalation: "
                + " | ".join(failures)
            )
