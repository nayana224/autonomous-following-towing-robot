# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Mode manager extension for fall-triggered robot safety stop."""

import threading
import time
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from aftr_mode_manager.mode_manager_node import ModeManagerNode
from aftr_mode_manager.mode_state import ModeStatus
from aftr_mode_manager.operator_status import OperatorViewState


class FallAwareModeStatus(ModeStatus):
    """Extend the existing operator status with fall-safety fields."""

    def __init__(self):
        """Initialize fall-detector and safety-stop status fields."""
        super().__init__()
        self.fall_detector_alive = False
        self.fall_detection_status = "UNKNOWN"
        self.fall_detected = False
        self.safety_stop_active = False
        self.safety_stop_in_progress = False
        self.safety_reason = ""
        self.safety_stop_result = ""
        self.safety_interrupted_mode = ""
        self.safety_interrupted_command = ""

    def to_dict(self):
        """Include fall-detector and safety-latch fields in operator status."""
        status = super().to_dict()
        status.update(
            {
                "fall_detector_alive": self.fall_detector_alive,
                "fall_detection_status": self.fall_detection_status,
                "fall_detected": self.fall_detected,
                "safety_stop_active": self.safety_stop_active,
                "safety_stop_in_progress": self.safety_stop_in_progress,
                "safety_reason": self.safety_reason,
                "safety_stop_result": self.safety_stop_result,
                "safety_interrupted_mode": self.safety_interrupted_mode,
                "safety_interrupted_command": self.safety_interrupted_command,
            }
        )
        return status


class FallAwareModeManagerNode(ModeManagerNode):
    """Stop active work on a fall and require an explicit operator reset."""

    SAFETY_ALLOWED_COMMANDS = {"stop", "shutdown"}

    def _initialize_runtime_state(self):
        self.status = FallAwareModeStatus()
        super()._initialize_runtime_state()
        self.last_fall_heartbeat_at = None
        self.last_fall_detector_alive = False
        self.safety_stop_thread = None
        self.pending_safety_clear = False
        self.safety_feedback_announced = False

    def _declare_runtime_parameters(self):
        super()._declare_runtime_parameters()
        self.fall_heartbeat_timeout_sec = float(
            self.declare_parameter("fall_heartbeat_timeout_sec", 2.0).value
        )

    def _create_publishers_and_subscribers(self):
        super()._create_publishers_and_subscribers()
        self.fall_status_sub = self.create_subscription(
            String,
            "/fall_detection/status",
            self.fall_status_callback,
            10,
            callback_group=self.callback_group,
        )
        self.fall_detected_sub = self.create_subscription(
            Bool,
            "/fall_detection/detected",
            self.fall_detected_callback,
            10,
            callback_group=self.callback_group,
        )
        self.fall_heartbeat_sub = self.create_subscription(
            Empty,
            "/fall_detection/heartbeat",
            self.fall_heartbeat_callback,
            10,
            callback_group=self.callback_group,
        )
        self.fall_heartbeat_watchdog_timer = self.create_timer(
            0.5,
            self.check_fall_detector_heartbeat,
            callback_group=self.callback_group,
        )

    def _create_external_service_clients(self):
        super()._create_external_service_clients()
        self._create_trigger_client(
            "fall_reset_client",
            "/fall_detection/reset",
        )

    def _register_mode_manager_services(self):
        super()._register_mode_manager_services()
        self.clear_safety_stop_service = self.create_service(
            Trigger,
            "~/command/clear_safety_stop",
            self.clear_safety_stop_callback,
            callback_group=self.callback_group,
        )

    def begin_command(self, response, command_name):
        """Reject new work while the fall-safety latch is active."""
        if (
            self.status.safety_stop_active
            and command_name not in self.SAFETY_ALLOWED_COMMANDS
        ):
            response.success = False
            response.message = (
                "command rejected: fall safety stop is active; "
                "use /mode_manager/command/clear_safety_stop after checking the scene"
            )
            self.get_logger().warning(response.message)
            self.publish_status()
            return False
        return super().begin_command(response, command_name)

    def fall_status_callback(self, message):
        """Record the detector observation for operator status."""
        self.status.fall_detection_status = str(message.data)

    def fall_detected_callback(self, message):
        """Latch a detected fall and initiate workflow cleanup."""
        detected = bool(message.data)
        previously_active = self.status.safety_stop_active
        self.status.fall_detected = detected

        if detected:
            self.pending_safety_clear = False
            self.status.safety_stop_active = True
            self.status.safety_reason = "FALL_DETECTED"
            if not previously_active:
                self.status.safety_interrupted_mode = self._mode_name()
                self.status.safety_interrupted_command = str(
                    self.status.last_command or self.active_command_name
                )
                self.status.safety_stop_result = "requested"
                self._announce_safety_stop()
                self.get_logger().error(
                    "fall detector safety latch received; starting workflow stop"
                )
                self._start_safety_stop_thread()
            self.publish_status()
            return

        if self.status.safety_stop_in_progress:
            self.pending_safety_clear = True
            self.get_logger().warning(
                "fall latch cleared while safety stop is running; "
                "release will be applied after stop completion"
            )
            return

        self._clear_fall_safety_state()

    def _mode_name(self):
        mode = self.status.mode
        return str(getattr(mode, "value", mode))

    def _announce_safety_stop(self):
        if self.safety_feedback_announced:
            return
        self.publish_audio_event("autonomous_bgm_stop")
        self.publish_audio_event("fall_safety_stop")
        self.publish_led_event("SAFETY_STOP")
        self.safety_feedback_announced = True

    def _start_safety_stop_thread(self):
        if self.safety_stop_thread is not None and self.safety_stop_thread.is_alive():
            return
        self.safety_stop_thread = threading.Thread(
            target=self._execute_fall_safety_stop,
            name="fall-safety-stop",
            daemon=True,
        )
        self.safety_stop_thread.start()

    def _execute_fall_safety_stop(self):
        self.get_logger().warning("fall safety stop is waiting for workflow ownership")
        self.command_lock.acquire()
        try:
            self.command_active = True
            self.active_command_name = "fall_safety_stop"
            self.status.busy = True
            self.status.last_command = "fall_safety_stop"
            self.status.safety_stop_in_progress = True
            self.status.safety_stop_result = "stopping"
            self.publish_status()

            response = Trigger.Response()
            result = self.stop_workflow(response)
            if result.success:
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
            self.command_active = False
            self.active_command_name = ""
            self.status.busy = False
            self.status.safety_stop_in_progress = False
            self.command_lock.release()

            if self.pending_safety_clear and not self.status.fall_detected:
                self.pending_safety_clear = False
                self._clear_fall_safety_state(publish=False)
            self.publish_status()

    def clear_safety_stop_callback(self, _request, response):
        """Reset the detector latch only after the robot has fully stopped."""
        if not self.status.safety_stop_active:
            response.success = True
            response.message = "safety stop is already cleared"
            return response
        if self.status.safety_stop_in_progress or self.command_active:
            return self._reject_safety_clear(
                response,
                "안전 정지가 아직 진행 중입니다. 잠시 후 다시 시도해 주세요.",
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

        self.status.fall_detected = False
        self._clear_fall_safety_state(publish=False)
        self.publish_audio_event("safety_stop_cleared")
        self.publish_led_event("SAFETY_CLEARED")
        self.publish_status()
        response.success = True
        response.message = "안전 정지가 해제되었습니다. 초기 화면으로 돌아갑니다."
        return response

    def _reject_safety_clear(self, response, message):
        self.publish_audio_event("safety_clear_rejected")
        self.publish_led_event("SAFETY_STOP")
        response.success = False
        response.message = message
        self.publish_status()
        return response

    def _clear_fall_safety_state(self, publish=True):
        was_active = self.status.safety_stop_active
        self.status.safety_stop_active = False
        self.status.safety_reason = ""
        self.status.safety_stop_result = "cleared" if was_active else ""
        self.safety_feedback_announced = False
        if was_active:
            self.get_logger().warning(
                "fall detector safety latch cleared; operator controls are available"
            )
        if publish:
            self.publish_status()

    def fall_heartbeat_callback(self, _message):
        """Record detector liveness for the safety watchdog."""
        self.last_fall_heartbeat_at = time.monotonic()
        if not self.status.fall_detector_alive:
            self.status.fall_detector_alive = True
            self.get_logger().info("fall detector heartbeat connected")

    def check_fall_detector_heartbeat(self):
        """Start safety cleanup when detector heartbeat expires."""
        last_heartbeat = self.last_fall_heartbeat_at
        alive = (
            last_heartbeat is not None
            and time.monotonic() - last_heartbeat <= self.fall_heartbeat_timeout_sec
        )
        self.status.fall_detector_alive = alive

        if alive == self.last_fall_detector_alive:
            return
        if alive:
            self.get_logger().info("fall detector heartbeat restored")
        else:
            self.get_logger().warning("fall detector heartbeat is unavailable")
        self.last_fall_detector_alive = alive

    def build_operator_view_state(self):
        """Include fall-safety information in the operator view."""
        if self.status.safety_stop_active:
            return OperatorViewState(
                step="SAFETY_STOP",
                manual_control_allowed=False,
                allowed_commands=(),
            )
        return super().build_operator_view_state()

    def update_operator_status(self):
        """Refresh operator-facing fall-safety status."""
        super().update_operator_status()
        if self.status.safety_stop_in_progress:
            self.status.operator_message = (
                "안전 정지 중입니다. 현재 동작을 취소하고 로봇을 IDLE로 전환합니다."
            )
        elif self.status.safety_stop_active:
            self.status.operator_message = (
                "작업자 쓰러짐이 감지되어 안전 정지되었습니다. "
                "현장을 확인한 뒤 안전 정지 해제 버튼을 눌러 주세요."
            )


def main(args=None):
    """Run the fall-aware mode manager and clean up ROS resources."""
    node = None
    executor = None
    try:
        rclpy.init(args=args)
        node = FallAwareModeManagerNode()
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
