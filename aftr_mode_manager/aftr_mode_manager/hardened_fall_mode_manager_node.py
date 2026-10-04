# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Repeated-test hardening for the fall-aware mode manager."""

import time
import traceback

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from aftr_mode_manager.fall_aware_mode_manager_node import FallAwareModeManagerNode
from aftr_mode_manager.mode_state import RobotMode


class HardenedFallModeManagerNode(FallAwareModeManagerNode):
    """Add clean reset boundaries, rollback, and detector-loss handling."""

    MOTION_COMMANDS = {
        "start_follow",
        "start_recording_follow",
        "start_alignment",
        "prepare_autonomous",
        "path_forward",
        "path_reverse",
        "path_forward_auto",
        "path_reverse_auto",
    }
    ACTIVE_SAFETY_MODES = {
        RobotMode.FOLLOW,
        RobotMode.RECORDING_FOLLOW,
        RobotMode.ALIGNMENT,
        RobotMode.LOCALIZING,
        RobotMode.AUTONOMOUS_READY,
        RobotMode.AUTONOMOUS_DRIVING,
    }

    def _initialize_runtime_state(self):
        super()._initialize_runtime_state()
        self.safety_cleanup_active = False
        self.last_map_message_at = None
        self.recording_session_started_at = None
        self.latest_follow_state_raw = ""
        self.pending_follow_state = ""
        self.pending_follow_state_since = None

    def _declare_runtime_parameters(self):
        super()._declare_runtime_parameters()
        self.fall_detection_required = bool(
            self.declare_parameter("fall_detection_required", True).value
        )
        self.fresh_map_timeout_sec = float(
            self.declare_parameter("fresh_map_timeout_sec", 12.0).value
        )
        self.follow_obstacle_led_hold_sec = float(
            self.declare_parameter("follow_obstacle_led_hold_sec", 0.5).value
        )
        self.follow_obstacle_clear_hold_sec = float(
            self.declare_parameter("follow_obstacle_clear_hold_sec", 0.4).value
        )

    def _create_publishers_and_subscribers(self):
        super()._create_publishers_and_subscribers()
        self.map_freshness_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_freshness_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

    def begin_command(self, response, command_name):
        """Block conflicting commands during safety or runtime recovery."""
        if (
            self.fall_detection_required
            and command_name in self.MOTION_COMMANDS
            and not self.status.fall_detector_alive
        ):
            response.success = False
            response.message = "command rejected: fall detector heartbeat is unavailable"
            self.get_logger().warning(response.message)
            self.publish_status()
            return False
        return super().begin_command(response, command_name)

    def follow_state_callback(self, msg):
        """Debounce obstacle indication without delaying the follower stop."""
        current_state = str(msg.data).strip().upper()
        self.latest_follow_state_raw = current_state
        confirmed_state = str(self.latest_follow_state).strip().upper()

        debounce_sec = 0.0
        if current_state == "OBSTACLE" and confirmed_state != "OBSTACLE":
            debounce_sec = max(0.0, self.follow_obstacle_led_hold_sec)
        elif confirmed_state == "OBSTACLE" and current_state != "OBSTACLE":
            debounce_sec = max(0.0, self.follow_obstacle_clear_hold_sec)

        if debounce_sec > 0.0:
            now_sec = time.monotonic()
            if current_state != self.pending_follow_state:
                self.pending_follow_state = current_state
                self.pending_follow_state_since = now_sec
                return
            if self.pending_follow_state_since is None:
                self.pending_follow_state_since = now_sec
                return
            if now_sec - self.pending_follow_state_since < debounce_sec:
                return

        self.pending_follow_state = ""
        self.pending_follow_state_since = None
        self._confirm_follow_state(current_state)

    def _confirm_follow_state(self, current_state):
        """Publish audio and LED only when the confirmed state changes."""
        previous_state = self.latest_follow_state
        self.latest_follow_state = current_state
        self.previous_follow_state = previous_state
        self._update_follow_audio_timestamps(previous_state, current_state)
        self.publish_follow_audio_transition(previous_state, current_state)
        if current_state != previous_state:
            self.publish_led_follow_state()

    def reset_follow_runtime_state(self):
        """Clear confirmed and pending follower state between sessions."""
        super().reset_follow_runtime_state()
        self.latest_follow_state_raw = ""
        self.pending_follow_state = ""
        self.pending_follow_state_since = None

    def map_freshness_callback(self, _message):
        """Track whether the current recording produced a fresh map."""
        self.last_map_message_at = time.monotonic()

    def fall_detected_callback(self, message):
        """Detector False updates observation only; operator reset releases latch."""
        detected = bool(message.data)
        self.status.fall_detected = detected
        if detected:
            super().fall_detected_callback(message)
            return
        self.publish_status()

    def notify_return_to_home(self):
        """Do not play the normal home cue during emergency cleanup."""
        if self.safety_cleanup_active:
            return
        super().notify_return_to_home()

    def _execute_fall_safety_stop(self):
        """Use normal stop mechanics, then enforce a clean IDLE boundary."""
        self.get_logger().warning("fall safety stop is waiting for workflow ownership")
        self.command_lock.acquire()
        try:
            self.command_active = True
            self.active_command_name = "fall_safety_stop"
            self.status.busy = True
            self.status.last_command = "fall_safety_stop"
            self.status.safety_stop_in_progress = True
            self.status.safety_stop_result = "stopping"
            self.safety_cleanup_active = True
            self.publish_status()

            response = Trigger.Response()
            result = super(FallAwareModeManagerNode, self).stop_workflow(response)
            if not result.success:
                result = self._recover_safety_cleanup(result)

            if result.success:
                self._reset_workflow_runtime_after_safety()
                self.status.safety_stop_result = "stopped_in_idle"
                self.get_logger().error(
                    "fall safety stop completed; workflow is IDLE and remains latched"
                )
            else:
                detail = result.message or "unknown workflow stop failure"
                self.status.safety_stop_result = f"failed: {detail}"
                self.status.last_error = f"fall safety stop failed: {detail}"
                self.get_logger().error(self.status.last_error)
        except Exception as exc:
            self.status.safety_stop_result = f"failed: {exc}"
            self.status.last_error = f"fall safety stop crashed: {exc}"
            self.get_logger().error(self.status.last_error)
            traceback.print_exc()
        finally:
            self.safety_cleanup_active = False
            self.command_active = False
            self.active_command_name = ""
            self.status.busy = False
            self.status.safety_stop_in_progress = False
            self.command_lock.release()
            self.publish_status()

    def _recover_safety_cleanup(self, failed_response):
        """Treat data-save failures as warnings when all motion processes stop."""
        warning = failed_response.message or "workflow cleanup reported failure"
        self.get_logger().warning(f"safety cleanup recovery started: {warning}")
        self._cancel_active_path_follow(timeout_sec=1.0)

        processes = (
            self.follower_node,
            self.nav2_process,
            self.path_process,
            self.slam_process,
        )
        for process in processes:
            process.stop(self.shutdown_timeout_sec)

        still_running = [process.name for process in processes if process.is_running()]
        response = Trigger.Response()
        if still_running:
            response.success = False
            response.message = "failed to stop: " + ", ".join(still_running)
            return response

        self.status.safety_stop_result = f"cleanup_warning: {warning}"
        self.request_mode(response, RobotMode.IDLE, "fall_safety_stop", force=True)
        response.success = True
        response.message = f"safe IDLE reached with cleanup warning: {warning}"
        return response

    def _reset_workflow_runtime_after_safety(self):
        """Clear process flags and cached session data before another run."""
        self.latest_path_status = {}
        self.last_path_follow_event = "idle"
        self.reset_follow_runtime_state()
        self.alignment_control_active = False
        self.active_drive_direction = ""
        self.pending_drive_direction = ""
        self.next_drive_direction = "reverse"
        self.reset_autonomous_audio_state()
        self.reset_autonomous_led_state()
        self.recording_session_started_at = None

        for running_attr, ready_attr in (
            ("follower_running", "follower_ready"),
            ("nav2_running", "nav2_ready"),
            ("path_running", "path_ready"),
            ("slam_running", "slam_ready"),
        ):
            setattr(self.status, running_attr, False)
            setattr(self.status, ready_attr, False)
        self.status.amcl_pose_ready = False
        self.status.last_error = ""
        self.status.mode = RobotMode.IDLE

    def clear_safety_stop_callback(self, _request, response):
        """Serialize reset with all other workflow commands."""
        if not self.command_lock.acquire(blocking=False):
            return self._reject_safety_clear(
                response,
                "다른 상태 변경을 처리하고 있습니다. 잠시 후 다시 시도해 주세요.",
            )

        self.command_active = True
        self.active_command_name = "clear_safety_stop"
        self.status.busy = True
        try:
            if not self.status.safety_stop_active:
                response.success = True
                response.message = "safety stop is already cleared"
                return response
            if self.status.safety_stop_in_progress:
                return self._reject_safety_clear(
                    response,
                    "안전 정지가 아직 진행 중입니다. 잠시 후 다시 시도해 주세요.",
                )
            if self.status.safety_stop_result.startswith("failed"):
                return self._reject_safety_clear(
                    response,
                    "안전 정지 완료를 확인할 수 없습니다. 오류 상태를 확인해 주세요.",
                )
            if not self.status.fall_detector_alive:
                return self._reject_safety_clear(
                    response,
                    "낙상 감지기 연결을 확인한 뒤 다시 시도해 주세요.",
                )
            if self.status.fall_detection_status == "FALL_DETECTED":
                return self._reject_safety_clear(
                    response,
                    "아직 쓰러짐 상태가 감지되고 있습니다. 작업자의 상태를 확인해 주세요.",
                )

            self.publish_led_event("SAFETY_CLEARING")
            if not self.call_trigger_service(
                self.fall_reset_client,
                "/fall_detection/reset",
                timeout_sec=5.0,
            ):
                return self._reject_safety_clear(
                    response,
                    f"감지기 초기화에 실패했습니다: {self.last_service_error}",
                )

            self._reset_workflow_runtime_after_safety()
            self.status.fall_detected = False
            self._clear_fall_safety_state(publish=False)
            self.publish_audio_event("safety_stop_cleared")
            self.publish_led_event("SAFETY_CLEARED")
            response.success = True
            response.message = "안전 정지가 해제되었습니다. 초기 화면으로 돌아갑니다."
            return response
        finally:
            self.command_active = False
            self.active_command_name = ""
            self.status.busy = False
            self.command_lock.release()
            self.publish_status()

    def start_recording_follow(self, response):
        """Start recording infrastructure before allowing follower motion."""
        initial_process_state = self._recording_process_state()
        self._prepare_recording_session()

        if not self.ensure_base_running(response):
            return response
        if not self.ensure_slam_running(response):
            return self._recording_start_failure(response, initial_process_state)
        if not self._wait_for_fresh_map(self.fresh_map_timeout_sec):
            response.message = "fresh /map message was not received after SLAM start"
            return self._recording_start_failure(response, initial_process_state)
        if not self.ensure_path_manager_running(response):
            return self._recording_start_failure(response, initial_process_state)
        if not self._start_path_recording(response):
            return self._recording_start_failure(response, initial_process_state)
        if not self.ensure_follower_node_running(response):
            return self._recording_start_failure(response, initial_process_state)

        previous_mode = self.status.mode
        mode_response = self.request_mode(
            response,
            RobotMode.RECORDING_FOLLOW,
            "start_recording_follow",
        )
        if not mode_response.success:
            return self._recording_start_failure(response, initial_process_state)
        if previous_mode != RobotMode.RECORDING_FOLLOW:
            self.notify_follow_start(recording=True)
        return mode_response

    def _recording_process_state(self):
        return {
            "slam": self.slam_process.is_running(),
            "path": self.path_process.is_running(),
            "follower": self.follower_node.is_running(),
        }

    def _prepare_recording_session(self):
        self.latest_path_status = {}
        self.last_path_follow_event = "idle"
        self.last_map_message_at = None
        self.recording_session_started_at = time.monotonic()
        self.reset_follow_runtime_state()

    def _start_path_recording(self, response):
        if not self.wait_for_path_manager_service(
            self.path_start_record_client,
            "/path_manager/start_record",
            timeout_sec=5.0,
        ):
            response.message = "failed to start path recording"
            return False

        if self.call_trigger_service(
            self.path_start_record_client,
            "/path_manager/start_record",
            timeout_sec=2.0,
        ):
            return True
        if self.wait_for_path_recording_active(timeout_sec=2.0):
            return True

        response.message = "failed to start path recording"
        return False

    def _wait_for_fresh_map(self, timeout_sec):
        started_at = self.recording_session_started_at or time.monotonic()
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            received_at = self.last_map_message_at
            if received_at is not None and received_at >= started_at:
                return True
            time.sleep(0.05)
        return False

    def _recording_start_failure(self, response, initial_process_state):
        """Rollback only processes started by the failed recording command."""
        detail = response.message or self.last_service_error or "recording start failed"
        if not initial_process_state["follower"]:
            self.follower_node.stop(self.shutdown_timeout_sec)
        if not initial_process_state["path"]:
            self.path_process.stop(self.shutdown_timeout_sec)
        if not initial_process_state["slam"]:
            self.slam_process.stop(self.shutdown_timeout_sec)

        self._reset_workflow_runtime_after_safety()
        self.status.last_error = detail
        self.status.mode = RobotMode.IDLE
        response.success = False
        response.message = detail
        self.publish_led_mode_event(RobotMode.IDLE)
        self.get_logger().warning(f"recording-follow startup rolled back: {detail}")
        self.publish_status()
        return response

    def check_fall_detector_heartbeat(self):
        """Reconcile detector timeout with active safety state."""
        previous_alive = self.last_fall_detector_alive
        super().check_fall_detector_heartbeat()
        alive = self.status.fall_detector_alive
        if (
            self.fall_detection_required
            and previous_alive
            and not alive
            and self.status.mode in self.ACTIVE_SAFETY_MODES
            and not self.status.safety_stop_active
        ):
            self.status.fall_detected = False
            self.status.safety_reason = "FALL_DETECTOR_LOST"
            self.status.safety_stop_active = True
            self.status.safety_interrupted_mode = self._mode_name()
            self.status.safety_interrupted_command = str(
                self.active_command_name or self.status.last_command
            )
            self.status.safety_stop_result = "requested"
            self._announce_safety_stop()
            self._start_safety_stop_thread()
            self.publish_status()


def main(args=None):
    """Run the hardened fall-aware manager and release ROS resources."""
    node = None
    executor = None
    try:
        rclpy.init(args=args)
        node = HardenedFallModeManagerNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("mode_manager interrupted by SIGINT")
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"mode_manager crashed: {exc}")
        else:
            print(f"mode_manager crashed before startup completed: {exc}")
        traceback.print_exc()
    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception as exc:
                node.get_logger().warning(
                    f"failed to remove mode_manager during shutdown: {exc}"
                )
        if node is not None:
            try:
                node.shutdown_processes()
            except Exception as exc:
                node.get_logger().warning(
                    f"failed to stop managed processes cleanly: {exc}"
                )
            try:
                node.destroy_node()
            except Exception as exc:
                node.get_logger().warning(
                    f"failed to destroy mode_manager node cleanly: {exc}"
                )
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
