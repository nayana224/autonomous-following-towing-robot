# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""High-level workflow rules for operator-facing robot behavior.

This module keeps workflow and mode-transition logic separate from process
supervision. It owns command locking, mode validation, path replay direction
rules, and the high-level orchestration used by the GUI-facing command
services.
"""

import time

from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from std_srvs.srv import Trigger

from aftr_mode_manager.mode_state import RobotMode
from aftr_mode_manager.runtime.process_supervisor import (
    cleanup_registered_process_groups,
)
from aftr_mode_manager.workflow.path_events import is_blocked_path_event
from aftr_mode_manager.workflow.path_events import is_terminal_path_failure_event
from aftr_mode_manager.workflow.path_events import parse_path_status
from aftr_mode_manager.workflow.state import WorkflowStateMixin


class WorkflowManagerMixin(WorkflowStateMixin):
    """Provide workflow events, transitions, and command orchestration."""

    AUTONOMOUS_MODES = {
        RobotMode.AUTONOMOUS_READY,
        RobotMode.AUTONOMOUS_DRIVING,
    }

    ACTIVE_NAV2_MODES = {
        RobotMode.ALIGNMENT,
        RobotMode.LOCALIZING,
        RobotMode.AUTONOMOUS_READY,
        RobotMode.AUTONOMOUS_DRIVING,
    }

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

    def follow_state_callback(self, msg):
        """Cache person follower state for GUI-facing status generation."""
        previous_state = self.latest_follow_state
        self.latest_follow_state = msg.data
        self.previous_follow_state = previous_state
        self._update_follow_audio_timestamps(previous_state, self.latest_follow_state)
        self.publish_follow_audio_transition(previous_state, self.latest_follow_state)
        self.publish_led_follow_state()

    def reset_follow_runtime_state(self):
        """Clear cached follower state before a new follow session begins."""
        self.latest_follow_state = ""
        self.previous_follow_state = ""
        self.follow_search_started_at = None
        self.follow_detected_announced = False

    def _update_follow_audio_timestamps(self, previous_state, current_state):
        """Track follow-state timing used by operator audio debounce rules."""
        previous = str(previous_state).strip().upper()
        current = str(current_state).strip().upper()
        if current == previous:
            return
        if current == "SEARCH":
            self.follow_search_started_at = time.monotonic()
            return
        if current == "FOLLOW":
            self.follow_search_started_at = None

    def odom_callback(self, msg):
        """Cache linear odometry speed for operator-facing ETA estimates."""
        self.latest_linear_speed = abs(float(msg.twist.twist.linear.x))

    def wait_for_path_recording_active(self, timeout_sec):
        """Wait until cached path-manager status reports ``recording=true``.

        Args:
            timeout_sec: Maximum wait duration in seconds.

        Returns:
            ``True`` if path recording becomes active before timeout.
        """
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if bool(self.latest_path_status.get("recording", False)):
                return True
            time.sleep(0.05)
        return bool(self.latest_path_status.get("recording", False))

    def wait_for_path_manager_service(self, client, service_name, timeout_sec):
        """Wait until a path-manager Trigger service becomes available.

        Args:
            client: ROS Trigger client instance to probe.
            service_name: Fully qualified service name for logging.
            timeout_sec: Maximum wait duration in seconds.

        Returns:
            ``True`` if the service becomes available before the timeout.
        """
        if client.wait_for_service(timeout_sec=timeout_sec):
            return True

        self.last_service_error = (
            f"timed out waiting for service: {service_name} "
            f"({timeout_sec:.1f}s)"
        )
        self.get_logger().error(self.last_service_error)
        return False

    def path_status_callback(self, msg):
        """Update cached path status and react to path replay completion.

        When autonomous replay completes, this callback moves the workflow back
        to ``ALIGNMENT`` and flips the next requested replay direction.
        """
        self.latest_path_status = self.parse_path_status(msg.data)
        event = str(self.latest_path_status.get("last_follow_event", ""))
        if not event:
            return

        previous_event = self.last_path_follow_event
        self.last_path_follow_event = event
        replay_starting = (
            self.status.mode == RobotMode.AUTONOMOUS_READY
            and bool(self.pending_drive_direction)
        )
        replay_active = self.status.mode == RobotMode.AUTONOMOUS_DRIVING

        if self.status.mode == RobotMode.AUTONOMOUS_DRIVING:
            if self.is_blocked_path_event(event):
                self.notify_autonomous_blocked()
                self.notify_autonomous_blocked_led()
            else:
                self.clear_autonomous_blocked()
                self.clear_autonomous_blocked_led()

        if (
            event == "completed"
            and previous_event != "completed"
            and (replay_active or replay_starting)
        ):
            self.notify_autonomous_arrived()
            self.notify_autonomous_arrived_led()
            self.status.mode = RobotMode.ALIGNMENT
            self.status.last_command = "path_completed_alignment"
            self.status.transition_count += 1
            self.alignment_control_active = False
            self.next_drive_direction = (
                "forward"
                if (
                    self.active_drive_direction or self.pending_drive_direction
                ) == "reverse"
                else "reverse"
            )
            self.active_drive_direction = ""
            self.pending_drive_direction = ""
            self.get_logger().info(
                "FollowPath completed; mode set to ALIGNMENT "
                f"(next_drive_direction={self.next_drive_direction})"
            )
            self.schedule_alignment_confirmation_request()
            self.publish_status()
            return

        following_path = bool(
            self.latest_path_status.get("following_path", False)
        )
        if (
            (replay_active or replay_starting)
            and not following_path
            and event != previous_event
            and self.is_terminal_path_failure_event(event)
        ):
            self.active_drive_direction = ""
            self.pending_drive_direction = ""
            self.notify_autonomous_failed()
            self.notify_autonomous_failed_led()
            self.set_error(
                "saved-path replay ended without completion: " + event
            )

    @staticmethod
    def parse_path_status(text):
        """Parse a path-manager status payload."""
        return parse_path_status(text)

    @staticmethod
    def is_blocked_path_event(event):
        """Return whether a path event indicates obstacle blocking."""
        return is_blocked_path_event(event)

    @staticmethod
    def is_terminal_path_failure_event(event):
        """Return whether a path event is a terminal replay failure."""
        return is_terminal_path_failure_event(event)

    def _reject_if_error_state(self, response):
        """Reject a command when the current workflow mode is ``ERROR``."""
        if self.status.mode != RobotMode.ERROR:
            return False
        self.reject_response(
            response,
            "mode manager is in ERROR; call clear_error first",
        )
        return True

    def _save_pose_via_path_manager(self, response, failure_message):
        """Save the current pose through ``path_manager``.

        Args:
            response: Trigger response updated when the call fails.
            failure_message: Fallback error text when the service gives none.

        Returns:
            ``True`` when pose save succeeds.
        """
        if not self.ensure_path_manager_running(response):
            return False
        if self.call_trigger_service_with_retry(
            self.path_save_pose_client,
            "/path_manager/save_pose",
            timeout_sec=5.0,
            attempts=2,
        ):
            return True
        message = self.last_service_error or failure_message
        self.reject_response(response, message)
        return False

    def _publish_initial_pose_via_path_manager(self, response, failure_message):
        """Publish the stored initial pose through ``path_manager``.

        Args:
            response: Trigger response updated when the call fails.
            failure_message: Fallback error text when the service gives none.

        Returns:
            ``True`` when initial-pose publication succeeds.
        """
        if self.call_trigger_service_with_retry(
            self.path_publish_initial_pose_client,
            "/path_manager/publish_initial_pose",
            timeout_sec=5.0,
            attempts=getattr(self, "initial_pose_service_attempts", 3),
            success_probe=getattr(self, "_initial_pose_runtime_ready", None),
        ):
            return True
        message = self.last_service_error or failure_message
        self.reject_response(response, message)
        return False

    def _require_amcl_pose_ready(self, response, failure_prefix):
        """Require ``/amcl_pose`` visibility before autonomous steps continue.

        Args:
            response: Trigger response updated when readiness fails.
            failure_prefix: Prefix prepended to the missing-topic error text.

        Returns:
            ``True`` when the AMCL pose topic is ready.
        """
        is_ready, missing_topics = self.amcl_pose_readiness.wait_until_ready(
            self.amcl_pose_ready_timeout_sec,
        )
        self.status.amcl_pose_ready = is_ready
        if is_ready:
            return True
        self.fail_response(
            response,
            failure_prefix + ", ".join(missing_topics),
        )
        return False

    def _stop_process_or_fail(
        self,
        response,
        process,
        timeout_message,
        running_flag=None,
        ready_flag=None,
    ):
        """Stop one managed process and clear cached runtime flags.

        Args:
            response: Trigger response updated when stop fails.
            process: Managed process instance to stop.
            timeout_message: Error text when shutdown times out.
            running_flag: Optional status attribute cleared after stop.
            ready_flag: Optional status attribute cleared after stop.

        Returns:
            ``True`` when the process stops successfully.
        """
        if not process.stop(self.shutdown_timeout_sec):
            self.fail_response(response, timeout_message)
            return False
        if running_flag is not None:
            setattr(self.status, running_flag, False)
        if ready_flag is not None:
            setattr(self.status, ready_flag, False)
        return True

    def _cancel_active_path_follow(self, timeout_sec=3.0):
        """Cancel the current FollowPath goal if one may still be active."""
        self.call_trigger_service(
            self.path_cancel_follow_path_client,
            "/path_manager/cancel_follow_path",
            timeout_sec=timeout_sec,
        )

    def _set_process_state(self, running_attr=None, ready_attr=None):
        """Clear cached runtime flags after a shutdown or recovery step."""
        if running_attr is not None:
            setattr(self.status, running_attr, False)
        if ready_attr is not None:
            setattr(self.status, ready_attr, False)

    def start_follow_callback(self, request, response):
        """Request FOLLOW mode."""
        return self.run_exclusive_command(
            response,
            "start_follow",
            self.start_follow,
        )

    def start_follow(self, response):
        """Start the minimal runtime required for person-follow mode.

        Args:
            response: Trigger response returned to the caller.

        Returns:
            Updated response after base/follower checks and mode transition.
        """
        self.get_logger().info("start_follow: checking base")
        if not self.ensure_base_running(response):
            return response
        self.get_logger().info("start_follow: checking person follower")
        if not self.ensure_follower_node_running(response):
            return response
        self.reset_follow_runtime_state()
        previous_mode = self.status.mode
        mode_response = self.request_mode(response, RobotMode.FOLLOW, "start_follow")
        if mode_response.success and previous_mode != RobotMode.FOLLOW:
            self.notify_follow_start(recording=False)
        return mode_response

    def start_recording_follow_callback(self, request, response):
        """Request RECORDING_FOLLOW mode."""
        return self.run_exclusive_command(
            response,
            "start_recording_follow",
            self.start_recording_follow,
        )

    def start_recording_follow(self, response):
        """Start the runtime required for path-recording follow mode.

        This command ensures base, follower, SLAM, and path manager are ready,
        then asks ``path_manager`` to begin recording.
        """
        self.get_logger().info("start_recording_follow: checking base")
        if not self.ensure_base_running(response):
            return response
        self.get_logger().info("start_recording_follow: checking person follower")
        if not self.ensure_follower_node_running(response):
            return response
        self.reset_follow_runtime_state()
        self.get_logger().info("start_recording_follow: checking SLAM")
        if not self.ensure_slam_running(response):
            return response
        self.get_logger().info("start_recording_follow: checking path manager")
        if not self.ensure_path_manager_running(response):
            return response
        if not self.wait_for_path_manager_service(
            self.path_start_record_client,
            "/path_manager/start_record",
            timeout_sec=5.0,
        ):
            return self.fail_response(response, "failed to start path recording")
        if not self.call_trigger_service(
            self.path_start_record_client,
            "/path_manager/start_record",
            timeout_sec=2.0,
        ):
            if not self.wait_for_path_recording_active(timeout_sec=2.0):
                return self.fail_response(response, "failed to start path recording")
            self.get_logger().warn(
                "/path_manager/start_record response was not received, "
                "but path_manager status reports recording=true"
            )
        previous_mode = self.status.mode
        mode_response = self.request_mode(
            response,
            RobotMode.RECORDING_FOLLOW,
            "start_recording_follow",
        )
        if mode_response.success and previous_mode != RobotMode.RECORDING_FOLLOW:
            self.notify_follow_start(recording=True)
        return mode_response

    def finish_recording_for_alignment_callback(self, request, response):
        """Finish recording-follow and keep SLAM/path manager for alignment."""
        return self.run_exclusive_command(
            response,
            "finish_recording_for_alignment",
            self.finish_recording_for_alignment,
        )

    def start_alignment_callback(self, request, response):
        """Request ALIGNMENT mode."""
        return self.run_exclusive_command(
            response,
            "start_alignment",
            self.start_alignment,
        )

    def skip_alignment_callback(self, request, response):
        """Skip manual alignment and continue autonomous preparation."""
        return self.run_exclusive_command(
            response,
            "skip_alignment",
            self.skip_alignment,
        )

    def start_alignment(self, response):
        """Enter manual alignment mode and enable alignment control."""
        self.cancel_alignment_confirmation_request()
        if self.status.mode == RobotMode.AUTONOMOUS_DRIVING:
            self.call_trigger_service(
                self.path_cancel_follow_path_client,
                "/path_manager/cancel_follow_path",
                timeout_sec=3.0,
            )

        self.alignment_control_active = True
        mode_response = self.request_mode(
            response,
            RobotMode.ALIGNMENT,
            "start_alignment",
        )
        if mode_response.success:
            self.notify_alignment_request()
        return mode_response

    def skip_alignment(self, response):
        """Use the current pose and continue without manual alignment."""
        if self.status.mode != RobotMode.ALIGNMENT:
            return self.reject_response(
                response,
                "skip_alignment requires ALIGNMENT mode",
            )
        if self.alignment_control_active:
            return self.reject_response(
                response,
                "skip_alignment is only allowed before manual alignment starts",
            )

        self.cancel_alignment_confirmation_request()

        if not self._save_pose_via_path_manager(
            response,
            "failed to save current pose before autonomous preparation",
        ):
            return response

        if not self._publish_initial_pose_via_path_manager(
            response,
            "failed to publish current pose before autonomous preparation",
        ):
            return response

        self.alignment_control_active = False
        if self.nav2_ready_now():
            return self._enter_autonomous_ready(
                response,
                command_name="skip_alignment",
            )

        return self.request_mode(
            response,
            RobotMode.LOCALIZING,
            "skip_alignment",
        )

    def finish_alignment_callback(self, request, response):
        """Save the aligned pose and make autonomous driving available."""
        return self.run_exclusive_command(
            response,
            "finish_alignment",
            self.finish_alignment,
        )

    def save_pose_callback(self, request, response):
        """Save the current robot pose through path_manager."""
        return self.run_exclusive_command(
            response,
            "save_pose",
            self.save_pose,
        )

    def save_pose(self, response):
        """Save the current robot pose without changing workflow mode."""
        if not self._save_pose_via_path_manager(
            response,
            "failed to save current pose",
        ):
            return response

        response.success = True
        response.message = "current pose saved"
        self.status.last_command = "save_pose"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def start_localizing_callback(self, request, response):
        """Start Nav2 localization and publish the saved initial pose."""
        return self.run_exclusive_command(
            response,
            "start_localizing",
            self.start_localizing,
        )

    def start_localizing(self, response):
        """Start localization runtime and publish the saved initial pose.

        This command ensures base and path manager are available, stops SLAM if
        it is still active, starts Nav2 localization, then republishes the last
        saved pose for AMCL initialization.
        """
        if not self.ensure_base_running(response):
            return response
        if not self.ensure_path_manager_running(response):
            return response
        self._cancel_active_path_follow(timeout_sec=2.0)

        if self.slam_process.is_running() and not self._stop_process_or_fail(
            response,
            self.slam_process,
            "SLAM shutdown timed out",
            running_flag="slam_running",
            ready_flag="slam_ready",
        ):
            return response

        if not self.start_nav2_for_localization(response):
            return response

        time.sleep(float(self.nav2_initial_pose_delay_sec))

        if not self.call_trigger_service(
            self.path_publish_initial_pose_client,
            "/path_manager/publish_initial_pose",
            timeout_sec=5.0,
        ):
            return self.fail_response(response, "failed to publish initial pose")

        is_ready, missing_topics = self.amcl_pose_readiness.wait_until_ready(
            self.amcl_pose_ready_timeout_sec,
        )
        self.status.amcl_pose_ready = is_ready
        if is_ready:
            self.get_logger().info("AMCL pose is visible after initial pose")
        else:
            self.get_logger().warn(
                "AMCL pose is not visible yet after initial pose; "
                f"missing topics: {', '.join(missing_topics)}"
            )
        return self.request_mode(
            response,
            RobotMode.LOCALIZING,
            "start_localizing",
        )

    def set_autonomous_ready_callback(self, request, response):
        """Request AUTONOMOUS_READY mode."""
        return self.run_exclusive_command(
            response,
            "set_autonomous_ready",
            self.set_autonomous_ready,
        )

    def set_autonomous_ready(self, response):
        """Move from ``LOCALIZING`` to ``AUTONOMOUS_READY``.

        The command requires AMCL pose visibility and successful Nav2 readiness
        validation before the mode transition is accepted.
        """
        if self.status.mode != RobotMode.LOCALIZING:
            self.reject_response(
                response,
                "set_autonomous_ready requires LOCALIZING mode",
            )
            return response

        if not self.start_nav2_for_localization(response):
            return response

        if not self._require_amcl_pose_ready(
            response,
            "AMCL pose is not ready; missing topics: ",
        ):
            return response

        ready_response = self.check_nav2_ready(response)
        if not ready_response.success:
            return response
        if not self.wait_for_stable_autonomous_readiness(response):
            return response

        mode_response = self.request_mode(
            response,
            RobotMode.AUTONOMOUS_READY,
            "set_autonomous_ready",
        )
        if mode_response.success:
            self.notify_autonomous_ready()
        return mode_response

    def prepare_autonomous_callback(self, request, response):
        """Prepare localization after operator alignment with one GUI command."""
        return self.run_exclusive_command(
            response,
            "prepare_autonomous",
            self.prepare_autonomous,
        )

    def path_forward_callback(self, request, response):
        """Drive the saved path in the original recording direction."""
        return self.run_exclusive_command(
            response,
            "path_forward",
            self.path_forward,
        )

    def path_forward(self, response):
        """Drive the saved path in the original recording direction."""
        return self.start_saved_path_driving(
            response,
            reverse=False,
            command_name="path_forward",
        )

    def path_reverse_callback(self, request, response):
        """Drive the saved path in reverse, usually for returning after recording."""
        return self.run_exclusive_command(
            response,
            "path_reverse",
            self.path_reverse,
        )

    def path_reverse(self, response):
        """Drive the saved path in reverse, usually for returning after recording."""
        return self.start_saved_path_driving(
            response,
            reverse=True,
            command_name="path_reverse",
        )

    def path_forward_auto_callback(self, request, response):
        """Prepare autonomous mode if needed, then drive the saved path forward."""
        return self.run_exclusive_command(
            response,
            "path_forward_auto",
            self.path_forward_auto,
        )

    def path_forward_auto(self, response):
        """Prepare autonomous mode if needed, then drive the saved path forward."""
        return self.prepare_and_start_saved_path_driving(
            response,
            reverse=False,
            command_name="path_forward_auto",
        )

    def path_reverse_auto_callback(self, request, response):
        """Prepare autonomous mode if needed, then drive the saved path reverse."""
        return self.run_exclusive_command(
            response,
            "path_reverse_auto",
            self.path_reverse_auto,
        )

    def path_reverse_auto(self, response):
        """Prepare autonomous mode if needed, then drive the saved path reverse."""
        return self.prepare_and_start_saved_path_driving(
            response,
            reverse=True,
            command_name="path_reverse_auto",
        )

    def prepare_autonomous(self, response):
        """Prepare the workflow for autonomous path replay.

        Depending on the current mode, this command may save the current pose,
        start localization, and finish by entering ``AUTONOMOUS_READY``.
        """
        if self._reject_if_error_state(response):
            return response

        if self.status.mode == RobotMode.IDLE:
            return self.reject_response(
                response,
                "prepare_autonomous is not allowed from IDLE; "
                "start recording-follow and finish alignment first",
            )

        if self.status.mode in self.AUTONOMOUS_MODES:
            response.success = True
            response.message = "autonomous mode is already prepared"
            self.status.last_command = "prepare_autonomous"
            self.get_logger().info(response.message)
            self.publish_status()
            return response

        if self.status.mode == RobotMode.ALIGNMENT:
            save_response = Trigger.Response()
            if not self._save_pose_via_path_manager(
                save_response,
                "failed to save current pose",
            ):
                return self.copy_response(response, save_response)
            if not save_response.success:
                return self.copy_response(response, save_response)

        if self.status.mode != RobotMode.LOCALIZING:
            localize_response = Trigger.Response()
            self.start_localizing(localize_response)
            if not localize_response.success:
                return self.copy_response(response, localize_response)
        ready_response = Trigger.Response()
        ready_result = self.set_autonomous_ready(ready_response)
        if not ready_result.success:
            return self.copy_response(response, ready_result)
        return self.copy_response(response, ready_result)

    def finish_alignment(self, response):
        """Finalize manual alignment and continue autonomous preparation.

        The aligned pose is saved and republished immediately. If Nav2 is
        already fully ready, the workflow moves straight to
        ``AUTONOMOUS_READY``. Otherwise it returns to ``LOCALIZING`` so the
        operator is not blocked while the remaining Nav2 lifecycle nodes finish
        activating in the background.
        """
        if self.status.mode != RobotMode.ALIGNMENT:
            return self.reject_response(
                response,
                "finish_alignment requires ALIGNMENT mode",
            )

        if not self._save_pose_via_path_manager(
            response,
            "failed to save aligned pose",
        ):
            return response

        if not self._publish_initial_pose_via_path_manager(
            response,
            "failed to publish aligned pose",
        ):
            return response

        self.alignment_control_active = False
        self.notify_alignment_completed()
        if self.nav2_ready_now():
            return self._enter_autonomous_ready(
                response,
                command_name="finish_alignment",
            )

        return self.request_mode(
            response,
            RobotMode.LOCALIZING,
            "finish_alignment",
        )

    def _enter_autonomous_ready(self, response, command_name):
        """Enter ``AUTONOMOUS_READY`` and publish the ready cue once."""
        mode_response = self.request_mode(
            response,
            RobotMode.AUTONOMOUS_READY,
            command_name,
        )
        if mode_response.success:
            self.notify_autonomous_ready()
        return mode_response

    def prepare_and_start_saved_path_driving(self, response, reverse, command_name):
        """Prepare autonomous replay if needed, then start saved-path driving.

        Args:
            response: Trigger response returned to the caller.
            reverse: ``True`` to drive the saved path in reverse direction.
            command_name: Public command name stored in status.

        Returns:
            Updated response after preparation and replay start.
        """
        if self._reject_if_error_state(response):
            return response

        if self.status.mode not in self.AUTONOMOUS_MODES:
            prepare_response = Trigger.Response()
            self.prepare_autonomous(prepare_response)
            if not prepare_response.success:
                return self.copy_response(response, prepare_response)

        if self.status.mode == RobotMode.LOCALIZING:
            ready_response = Trigger.Response()
            self.set_autonomous_ready(ready_response)
            if not ready_response.success:
                return self.copy_response(response, ready_response)

        return self.start_saved_path_driving(
            response,
            reverse=reverse,
            command_name=command_name,
        )

    def start_saved_path_driving(self, response, reverse, command_name):
        """Validate replay preconditions and start saved-path driving.

        Args:
            response: Trigger response returned to the caller.
            reverse: ``True`` to drive reverse, ``False`` for forward.
            command_name: Public command name stored in status.

        Returns:
            Updated response after replay validation and FollowPath dispatch.
        """
        requested_direction = "reverse" if reverse else "forward"
        if requested_direction != self.next_drive_direction:
            return self.reject_response(
                response,
                f"{command_name} is not allowed now; next direction is "
                f"{self.next_drive_direction}",
            )

        if self.status.mode not in {
            RobotMode.AUTONOMOUS_READY,
            RobotMode.AUTONOMOUS_DRIVING,
        }:
            return self.reject_response(
                response,
                f"{command_name} requires AUTONOMOUS_READY "
                "or AUTONOMOUS_DRIVING mode",
            )

        if not self.ensure_path_manager_running(response):
            return response

        if not self._require_amcl_pose_ready(
            response,
            "AMCL pose is not ready; missing topics: ",
        ):
            return response

        ready_response = self.check_nav2_ready(response)
        if not ready_response.success:
            return response
        if not self.wait_for_stable_autonomous_readiness(response):
            return response

        self.set_operator_message(
            "저장 경로를 계산하고 있습니다.",
            publish=True,
        )
        if not self.refresh_local_costmap_before_replay(response):
            self.notify_autonomous_failed()
            return response

        if reverse:
            path_client = self.path_follow_saved_path_reverse_client
            path_service_name = "/path_manager/follow_saved_path_reverse"
            direction_text = "reverse"
        else:
            path_client = self.path_follow_saved_path_client
            path_service_name = "/path_manager/follow_saved_path"
            direction_text = "forward"

        self.pending_drive_direction = requested_direction
        path_service_succeeded = self.call_trigger_service(
            path_client,
            path_service_name,
            # Keep this longer than the two supplied generator timeouts. This
            # prevents a late FollowPath dispatch after mode_manager/GUI have
            # already reported that the button request failed.
            timeout_sec=float(self.path_replay_service_timeout_sec),
        )
        if not path_service_succeeded:
            # A response can be lost, or a repeated button request can find an
            # already active path. Reconcile from path_manager's periodic
            # status instead of leaving the robot driving on the READY page.
            if bool(self.latest_path_status.get("following_path", False)):
                self.get_logger().warning(
                    "saved-path service was not confirmed, but path_manager "
                    "reports an active FollowPath; reconciling drive state"
                )
                self.last_service_error = ""
            else:
                self.pending_drive_direction = ""
                detail = self.last_service_error or (
                    f"failed to start {direction_text} saved path following"
                )
                self.notify_autonomous_failed()
                return self.reject_response(
                    response,
                    detail,
                )

        # A very fast action rejection/completion can be reported by the path
        # status callback while this service call is still returning.
        if self.status.mode == RobotMode.ERROR:
            self.pending_drive_direction = ""
            detail = self.last_service_error or (
                self.status.last_error
                or f"failed to start {direction_text} saved path following"
            )
            return self.reject_response(
                response,
                detail,
            )
        if (
            self.status.mode == RobotMode.ALIGNMENT
            and self.last_path_follow_event == "completed"
        ):
            self.pending_drive_direction = ""
            response.success = True
            response.message = "saved path completed during start confirmation"
            self.publish_status()
            return response

        self.status.last_error = ""
        self.active_drive_direction = requested_direction
        self.pending_drive_direction = ""
        if self.status.mode == RobotMode.AUTONOMOUS_DRIVING:
            self.status.last_command = command_name
            response.success = True
            response.message = (
                f"{command_name} accepted; mode remains AUTONOMOUS_DRIVING"
            )
            self.get_logger().info(response.message)
            self.ensure_rviz_running_for_autonomous()
            self.publish_status()
            return response

        mode_response = self.request_mode(
            response,
            RobotMode.AUTONOMOUS_DRIVING,
            command_name,
        )
        if mode_response.success:
            self.start_autonomous_audio()
            self.start_autonomous_led()
            self.ensure_rviz_running_for_autonomous()
        return mode_response

    def refresh_local_costmap_before_replay(self, response):
        """Clear stale local obstacles and require a fresh laser observation.

        The clear service runs only after Nav2 readiness has stabilized. A new
        scan is required before path preprocessing starts so real obstacles can
        repopulate the costmap while stale cells from an earlier run stay gone.
        """
        service_name = "/local_costmap/clear_entirely_local_costmap"
        client = self.local_costmap_clear_client
        if not client.wait_for_service(timeout_sec=3.0):
            self.reject_response(
                response,
                f"service unavailable: {service_name}",
            )
            return False

        future = client.call_async(ClearEntireCostmap.Request())
        if not self.wait_for_future(future, timeout_sec=3.0):
            self.reject_response(
                response,
                f"service timeout: {service_name}",
            )
            return False
        if future.result() is None:
            self.reject_response(
                response,
                f"service returned no response: {service_name}",
            )
            return False

        self.get_logger().info(
            "local costmap cleared before saved-path replay; "
            "waiting for a fresh /scan message"
        )
        if not self.scan_message_readiness.wait_for_new_message(timeout_sec=2.0):
            self.reject_response(
                response,
                "fresh /scan was not received after local costmap clear",
            )
            return False

        self.get_logger().info(
            "fresh /scan received after local costmap clear"
        )
        return True

    def stop_callback(self, request, response):
        """Stop the current high-level mode and return to IDLE."""
        return self.run_exclusive_command(
            response,
            "stop",
            self.stop_workflow,
        )

    def stop_workflow(self, response):
        """Stop the active workflow safely and return to ``IDLE``.

        This command shuts down only the processes relevant to the current
        workflow state, cancels autonomous replay if needed, and clears cached
        readiness flags on the way back to ``IDLE``.
        """
        previous_mode = self.status.mode
        self.cancel_alignment_confirmation_request()
        self.alignment_control_active = False
        self.active_drive_direction = ""
        self.pending_drive_direction = ""
        self.reset_follow_runtime_state()
        self.stop_autonomous_audio()
        self.reset_autonomous_led_state()

        if previous_mode in {RobotMode.FOLLOW, RobotMode.RECORDING_FOLLOW}:
            self.status.busy = True
            self.status.last_command = "stop"
            if not self._stop_process_or_fail(
                response,
                self.follower_node,
                "follow shutdown timed out",
                running_flag="follower_running",
                ready_flag="follower_ready",
            ):
                return response

        if previous_mode == RobotMode.RECORDING_FOLLOW:
            self.status.busy = True
            self.status.last_command = "stop"
            if not self._stop_process_or_fail(
                response,
                self.path_process,
                "path manager shutdown timed out",
                running_flag="path_running",
                ready_flag="path_ready",
            ):
                return response

            self.status.busy = True
            self.status.last_command = "stop"
            if not self._stop_process_or_fail(
                response,
                self.slam_process,
                "SLAM shutdown timed out",
                running_flag="slam_running",
                ready_flag="slam_ready",
            ):
                return response
        if previous_mode in self.ACTIVE_NAV2_MODES:
            self.status.busy = True
            self.status.last_command = "stop"
            if previous_mode == RobotMode.AUTONOMOUS_DRIVING:
                self._cancel_active_path_follow(timeout_sec=3.0)
                self.stop_rviz(Trigger.Response())
            if not self._stop_process_or_fail(
                response,
                self.nav2_process,
                "Nav2 shutdown timed out",
                running_flag="nav2_running",
                ready_flag="nav2_ready",
            ):
                return response
            self.status.amcl_pose_ready = False
        if self.path_process.is_running():
            if not self._stop_process_or_fail(
                response,
                self.path_process,
                "path manager shutdown timed out",
                running_flag="path_running",
                ready_flag="path_ready",
            ):
                return response
        if self.slam_process.is_running():
            if not self._stop_process_or_fail(
                response,
                self.slam_process,
                "SLAM shutdown timed out",
                running_flag="slam_running",
                ready_flag="slam_ready",
            ):
                return response
        if self.follower_node.is_running():
            if not self._stop_process_or_fail(
                response,
                self.follower_node,
                "follow shutdown timed out",
                running_flag="follower_running",
                ready_flag="follower_ready",
            ):
                return response
        mode_response = self.request_mode(
            response,
            RobotMode.IDLE,
            "stop",
            force=True,
        )
        if mode_response.success and previous_mode != RobotMode.IDLE:
            self.notify_return_to_home()
        return mode_response

    def ensure_rviz_running_for_autonomous(self):
        """Start RViz once while autonomous driving is active."""
        if not self.enable_rviz_view:
            return
        if self.rviz_process.is_running():
            return
        rviz_response = Trigger.Response()
        self.start_rviz(rviz_response)

    def shutdown_callback(self, request, response):
        """Stop every process owned by the mode manager."""
        return self.run_exclusive_command(
            response,
            "shutdown",
            self.shutdown_manager,
        )

    def shutdown_manager(self, response):
        """Stop every managed process during node shutdown."""
        self.status.busy = True
        self.status.last_command = "shutdown"
        self.cancel_alignment_confirmation_request()
        self.stop_autonomous_audio()
        self.reset_autonomous_led_state()

        self.status.mode = RobotMode.IDLE
        failures = []
        if not self.rviz_process.stop(self.shutdown_timeout_sec):
            failures.append("RViz")
        if not self.follower_node.stop(self.shutdown_timeout_sec):
            failures.append("follow")
        self.log_nav2_stop_request("shutdown command")
        if not self.nav2_process.stop(self.shutdown_timeout_sec):
            failures.append("Nav2")
        if not self.path_process.stop(self.shutdown_timeout_sec):
            failures.append("path manager")
        if not self.slam_process.stop(self.shutdown_timeout_sec):
            failures.append("SLAM")
        if not self.base_process.stop(self.shutdown_timeout_sec):
            failures.append("base bringup")

        if failures:
            return self.fail_response(
                response,
                "shutdown timed out after attempting every process group: "
                + ", ".join(failures),
            )

        self.status.busy = False
        self.status.base_running = False
        self.status.base_ready = False
        self.status.slam_running = False
        self.status.slam_ready = False
        self.status.path_running = False
        self.status.path_ready = False
        self.status.follower_running = False
        self.status.follower_ready = False
        self.status.nav2_running = False
        self.status.nav2_ready = False
        self.status.amcl_pose_ready = False
        response.success = True
        response.message = "mode manager shutdown complete"
        self.publish_led_mode_event(RobotMode.IDLE)
        self.publish_status()
        return response

    def clear_error_callback(self, request, response):
        """Clear an error state and return to a clean IDLE workflow state."""
        return self.run_exclusive_command(
            response,
            "clear_error",
            self.clear_error,
        )

    def clear_error(self, response):
        """Recover from ``ERROR`` and return to a clean ``IDLE`` state.

        The method stops recoverable upper-layer processes, clears cached
        readiness flags, resets the stored error message, and forces the final
        mode transition back to ``IDLE``.
        """
        self.status.busy = True
        self.status.last_command = "clear_error"
        self.pending_drive_direction = ""
        self.cancel_alignment_confirmation_request()
        self.stop_autonomous_audio()
        self.reset_autonomous_led_state()
        self._cancel_active_path_follow(timeout_sec=2.0)
        cleanup_failures = []
        if not self.follower_node.stop(self.shutdown_timeout_sec):
            cleanup_failures.append("follower")
        self.log_nav2_stop_request("clear_error command")
        if not self.nav2_process.stop(self.shutdown_timeout_sec):
            cleanup_failures.append("Nav2")
        if not self.path_process.stop(self.shutdown_timeout_sec):
            cleanup_failures.append("path manager")
        if not self.slam_process.stop(self.shutdown_timeout_sec):
            cleanup_failures.append("SLAM")
        self._set_process_state("follower_running", "follower_ready")
        self._set_process_state("nav2_running", "nav2_ready")
        self.status.amcl_pose_ready = False
        self._set_process_state("path_running", "path_ready")
        self._set_process_state("slam_running", "slam_ready")
        if cleanup_failures:
            return self.fail_response(
                response,
                "error cleanup could not stop all process groups: "
                + ", ".join(cleanup_failures),
            )

        self.status.last_error = ""
        return self.request_mode(response, RobotMode.IDLE, "clear_error", force=True)

    def fail_response(self, response, message):
        """Fill a failed response and move the workflow into ``ERROR``."""
        if self.status.mode in self.AUTONOMOUS_MODES or self.active_drive_direction:
            self.notify_autonomous_failed()
            self.notify_autonomous_failed_led()
        else:
            self.publish_led_mode_event(RobotMode.ERROR)
        self.pending_drive_direction = ""
        self.status.busy = False
        self.status.last_error = message
        self.status.mode = RobotMode.ERROR
        response.success = False
        response.message = message
        self.get_logger().error(message)
        self.publish_status()
        return response

    def reject_response(self, response, message):
        """Fill a failed response without entering ``ERROR``."""
        self.status.busy = False
        self.status.last_error = message
        response.success = False
        response.message = message
        self.get_logger().warn(message)
        self.publish_status()
        return response
