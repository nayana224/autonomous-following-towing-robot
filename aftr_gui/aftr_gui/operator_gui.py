# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""AFTR operator GUI backed by ``mode_manager`` services.

The GUI stays intentionally passive. It renders the latest operator-facing
status from ``/mode_manager/status``, sends button-driven service requests back
to ``mode_manager``, and publishes joystick teleoperation commands only when
manual control is explicitly allowed.
"""

import os
import signal
import sys
import traceback

from ament_index_python.packages import get_package_share_directory
from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow

from aftr_gui.core.mode_manager_client import COMMANDS
from aftr_gui.core.mode_manager_client import CommandWorker
from aftr_gui.core.mode_manager_client import ModeManagerClient
from aftr_gui.operator_step_mapper import alignment_decision_message
from aftr_gui.operator_step_mapper import alignment_decision_title
from aftr_gui.operator_step_mapper import autonomous_page_title
from aftr_gui.operator_step_mapper import autonomous_ready_message
from aftr_gui.operator_step_mapper import busy_page_message
from aftr_gui.operator_step_mapper import busy_page_title
from aftr_gui.operator_step_mapper import busy_status_message
from aftr_gui.operator_step_mapper import direction_button_visibility
from aftr_gui.operator_step_mapper import error_status_message
from aftr_gui.operator_step_mapper import follow_mode_available
from aftr_gui.operator_step_mapper import mode_text
from aftr_gui.operator_step_mapper import page_name_for_step
from aftr_gui.operator_step_mapper import robot_status_kind
from aftr_gui.status_parser import as_bool
from aftr_gui.status_parser import parse_allowed_commands
from aftr_gui.status_parser import parse_status_payload
from aftr_gui.teleop_mapper import TeleopConfig
from aftr_gui.teleop_mapper import map_joystick_to_cmd
from aftr_gui.views.responsive_dashboard import ResponsiveDashboardMixin
from aftr_gui.views.safety_dashboard import SafetyDashboardMixin

ACTIVE_MAIN_WINDOW = None


def close_active_window_on_sigint(_signum, _frame):
    """Close the active Qt window when the process receives ``SIGINT``."""
    if ACTIVE_MAIN_WINDOW is not None:
        ACTIVE_MAIN_WINDOW.close()


class RobotDashboard(QMainWindow):
    """Main operator dashboard window."""

    def __init__(self, window_title="MDBOT 운영 화면", client_factory=ModeManagerClient):
        super().__init__()

        self.share_dir = get_package_share_directory("aftr_gui")
        ui_path = os.path.join(self.share_dir, "gui", "mdbot_gui.ui")
        uic.loadUi(ui_path, self)
        self.setWindowTitle(window_title)

        self.mode_manager_client = client_factory()
        self.active_command_worker = None

        self.joystick_x_axis = 0.0
        self.joystick_y_axis = 0.0
        self.teleop_config = TeleopConfig()
        self.command_button_map = {}

        self.initialize_display_state()

        self.apply_check_icon()
        self.connect_ui_signals()
        self.connect_ros_bridge_signals()

        self.teleop_publish_timer = QTimer(self)
        self.teleop_publish_timer.timeout.connect(self.publish_teleop)
        self.teleop_publish_timer.start(100)

        self.render_operator_step()
        self.update_button_enabled_state(True)
        self.refresh_manual_control()

    def initialize_display_state(self):
        """Initialize GUI-only state used before the first ROS status update."""
        self.current_mode = "IDLE"
        self.last_command = ""
        self.last_error = ""
        self.operator_step = "HOME"
        self.operator_message = "대기 중입니다."
        self.tracking_message = "추종 상태를 확인하는 중입니다."
        self.recording_message = "경로 저장 상태를 확인하는 중입니다."
        self.path_message = "경로 주행 상태를 확인하는 중입니다."
        self.driving_message = "주행 상태를 확인하는 중입니다."
        self.manual_control_enabled = True
        self.allowed_commands = {"follow", "recording_follow"}
        self.allow_map_updates = False
        self.pending_map_message = None
        self.pending_pose_message = None
        self.pending_path_message = None
        self.map_enable_timer = QTimer(self)
        self.map_enable_timer.setSingleShot(True)
        self.map_enable_timer.timeout.connect(self.enable_deferred_map_updates)

    def apply_check_icon(self):
        """Show the original checkbox check image."""
        check_img = os.path.join(
            self.share_dir,
            "images",
            "check.png",
        ).replace(
            "\\", "/"
        )
        self.save_path_check.setStyleSheet(
            self.save_path_check.styleSheet()
            + (
                " QCheckBox::indicator:checked "
                f"{{ image: url({check_img}); }}"
            )
        )

    def connect_ui_signals(self):
        """Connect UI buttons to mode-manager command keys."""
        self.btn_follow.clicked.connect(self.handle_follow_mode)
        self.command_button_map = self.create_command_button_bindings()
        for button in self.command_button_map:
            button.clicked.connect(self.handle_command_button_clicked)

        self.joystick_widget.moved.connect(self.handle_joystick)

    def create_command_button_bindings(self):
        """Return the shared button-to-command binding table."""
        return {
            self.btn_autonomous: ("prepare_autonomous", True),
            self.btn_back_follow: ("stop", True),
            self.btn_back_path_save: ("finish_recording", True),
            self.btn_after_recording_align: ("alignment", True),
            self.btn_after_recording_later: ("skip_alignment", True),
            self.btn_after_recording_home: ("stop", True),
            self.btn_alignment_done: ("finish_alignment", True),
            self.btn_alignment_stop: ("stop", True),
            self.btn_auto_reverse: ("path_reverse_auto", True),
            self.btn_auto_forward: ("path_forward_auto", True),
            self.btn_back_auto: ("stop", True),
            self.btn_auto_driving_stop: ("stop", True),
            self.btn_error_clear: ("clear_error", False),
        }

    def handle_command_button_clicked(self):
        """Dispatch one button click through the shared command-binding table."""
        button = self.sender()
        binding = self.command_button_map.get(button)
        if binding is None:
            return
        command_key, enforce_allowed = binding
        self.request_command(
            command_key,
            enforce_allowed=enforce_allowed,
        )

    def handle_follow_mode(self):
        """Choose follow or recording-follow from the home checkbox."""
        if self.save_path_check.isChecked():
            self.request_command("recording_follow")
            return
        self.request_command("follow")

    def connect_ros_bridge_signals(self):
        """Connect ROS signals to GUI update slots."""
        self.mode_manager_client.mode_status.connect(self.update_mode_status)
        self.mode_manager_client.odom_velocity.connect(self.update_velocity_display)
        self.mode_manager_client.map_received.connect(self.handle_map_message)
        self.mode_manager_client.amcl_pose_received.connect(self.handle_pose_message)
        self.mode_manager_client.planned_path_received.connect(self.handle_path_message)

    def request_command(self, command_key, enforce_allowed=True):
        """Call one mode-manager command service."""
        if self.is_command_request_running():
            self.statusbar.showMessage("다른 명령을 처리하고 있습니다.", 3000)
            return
        if enforce_allowed and not self.is_command_allowed(command_key):
            self.show_blocked_command_message(command_key)
            return

        command = COMMANDS[command_key]
        self.update_button_enabled_state(False)
        self.refresh_manual_control()
        self.statusbar.showMessage(f"{command.label} 요청을 보냈습니다.", 4000)

        self.active_command_worker = CommandWorker(
            self.mode_manager_client,
            command_key,
        )
        self.active_command_worker.finished.connect(self.handle_command_result)
        self.active_command_worker.start()

    def is_command_request_running(self):
        """Return whether one GUI command worker is still running."""
        return (
            self.active_command_worker is not None
            and self.active_command_worker.isRunning()
        )

    def is_command_allowed(self, command_key):
        """Return whether the current operator state allows one command key."""
        if self.is_command_request_running():
            return False
        return command_key in self.allowed_commands

    def show_blocked_command_message(self, command_key):
        """Show a status-bar message when one command is not allowed."""
        command = COMMANDS.get(command_key)
        label = command.label if command else command_key
        self.statusbar.showMessage(
            f"현재 단계에서는 '{label}' 명령을 사용할 수 없습니다.",
            5000,
        )

    def handle_command_result(self, label, success, message):
        """Handle one completed mode-manager command request."""
        if success:
            status_message = f"{label} 요청이 완료되었습니다."
        else:
            detail = str(message).strip()
            status_message = f"{label} 요청에 실패했습니다."
            if detail:
                status_message += f" 원인: {detail}"
        self.statusbar.showMessage(status_message, 7000)
        self.finish_active_command_worker()
        self.update_button_enabled_state(True)
        self.refresh_manual_control()

    def finish_active_command_worker(self):
        """Safely release the active command worker after it finishes."""
        worker = self.active_command_worker
        if worker is None:
            return

        # The finished signal can arrive while Qt is still unwinding the thread
        # shutdown path. Wait briefly before dropping the last Python reference.
        worker.wait(200)
        worker.deleteLater()
        self.active_command_worker = None

    def update_mode_status(self, status_text):
        """Apply the latest ``/mode_manager/status`` payload to the GUI."""
        values = parse_status_payload(status_text)
        mode = values.get("mode", "UNKNOWN")
        busy = as_bool(values.get("busy", False))

        self.current_mode = mode
        self.last_command = str(values.get("last_command", self.last_command))
        self.last_error = str(values.get("last_error", self.last_error))
        self.operator_step = str(
            values.get("operator_step", self.operator_step)
        )
        self.operator_message = str(
            values.get("operator_message", self.operator_message),
        )
        self.tracking_message = str(
            values.get("tracking_message", self.tracking_message),
        )
        self.recording_message = str(
            values.get("recording_message", self.recording_message),
        )
        self.path_message = str(values.get("path_message", self.path_message))
        self.driving_message = str(
            values.get("driving_message", self.driving_message),
        )
        self.manual_control_enabled = as_bool(
            values.get("manual_control_allowed", False),
        )
        self.allowed_commands = parse_allowed_commands(
            values.get("allowed_commands", []),
        )

        suffix = " (처리 중)" if busy and mode != "ERROR" else ""
        self.set_label_text(
            self.label_robot_status,
            f"로봇 상태: {mode_text(mode)}{suffix}",
        )
        self.set_robot_status_style(robot_status_kind(mode))
        self.update_map_render_policy()
        self.render_operator_step()
        self.update_button_enabled_state(True)
        self.refresh_manual_control()

    def render_operator_step(self):
        """Render the current operator step on the stacked pages."""
        step = self.operator_step

        page_name = page_name_for_step(step)
        page = getattr(self, page_name, None) if page_name else None
        if page is not None and self.stackedWidget.currentWidget() is not page:
            self.stackedWidget.setCurrentWidget(page)

        if step == "BUSY":
            self.apply_busy_view()
            self.statusbar.showMessage(
                self.operator_message or busy_status_message(self.last_command),
                3000,
            )
            return

        if step == "FOLLOWING":
            self.apply_follow_view()
        elif step == "RECORDING_FOLLOW":
            self.apply_recording_follow_view()
        elif step == "ALIGNMENT_DECISION":
            self.apply_alignment_decision_view()
        elif step == "ALIGNING":
            self.apply_alignment_view()
        elif step == "LOCALIZING":
            self.apply_localizing_view()
        elif step == "AUTONOMOUS_READY":
            self.apply_autonomous_ready_view()
        elif step == "AUTONOMOUS_DRIVING":
            self.apply_autonomous_driving_view()
        elif step == "ERROR":
            self.apply_error_view()

        self.configure_autonomous_direction_buttons()

    def apply_follow_view(self):
        """Update labels for the standard follow page."""
        self.set_label_text(
            self.label_follow_guide,
            "작업자를 추종하고 있습니다.",
        )
        self.set_label_text(
            self.label_tracking_status,
            self.tracking_message,
        )

    def apply_recording_follow_view(self):
        """Update labels for the recording-follow page."""
        self.set_label_text(
            self.label_recording_status,
            self.recording_message,
        )
        self.set_label_text(
            self.label_path_tracking_status,
            self.tracking_message,
        )

    def apply_alignment_decision_view(self):
        """Update the shared alignment-decision page."""
        self.set_label_text(
            self.label_after_recording_title,
            alignment_decision_title(self.last_command),
        )
        self.set_label_text(
            self.label_after_recording_status,
            alignment_decision_message(self.last_command),
        )
        self.set_label_text(
            self.label_after_recording_hint,
            (
                "정렬을 진행하면 조이스틱으로 위치를 맞출 수 있습니다. "
                "정렬 없이 진행을 누르면 현재 위치 기준으로 자율주행 준비 화면으로 이동합니다."
            ),
        )

    def apply_alignment_view(self):
        """Update the manual alignment page."""
        self.set_label_text(
            self.label_alignment_status,
            "조이스틱을 통해 로봇을 정렬해 주세요.",
        )
        self.set_label_text(
            self.label_alignment_hint,
            "정렬이 끝나면 아래의 정렬 완료 버튼을 눌러 다음 단계로 진행합니다.",
        )

    def apply_localizing_view(self):
        """Update labels while localization is still running."""
        self.update_auto_ready_card(
            badge_text="준비 중",
            title=autonomous_page_title("LOCALIZING"),
            status=(
                self.operator_message
                or "위치 추정을 진행하고 있습니다. 준비가 완료되면 "
                "주행 버튼이 자동으로 표시됩니다."
            ),
            hint=(
                "로봇이 현재 위치를 안정적으로 확인하는 중입니다. "
                "잠시만 기다려 주세요."
            ),
        )

    def apply_busy_view(self):
        """Update visible labels so busy commands do not look visually frozen."""
        title = busy_page_title(self.last_command, self.current_mode)
        message = busy_page_message(
            self.last_command,
            self.current_mode,
            self.operator_message,
        )
        if title is None or message is None:
            return
        self.update_auto_ready_card(
            badge_text="처리 중",
            title=title,
            status=message,
            hint="안전하게 상태를 정리한 뒤 다음 화면으로 이동합니다.",
        )

    def apply_autonomous_ready_view(self):
        """Update labels for the autonomous-ready page."""
        self.update_auto_ready_card(
            badge_text="준비 완료",
            title=autonomous_page_title("AUTONOMOUS_READY"),
            status=autonomous_ready_message(self.allowed_commands),
            hint=(
                "버튼을 누르면 저장된 경로를 따라 "
                "이전 위치로 자동 이동합니다."
            ),
        )

    def apply_autonomous_driving_view(self):
        """Update labels for the active autonomous-driving page."""
        self.set_label_text(
            self.label_auto_driving_status,
            self.driving_message,
        )
        self.set_label_text(self.label_path_status, self.path_message)

    def apply_error_view(self):
        """Update labels for the error page."""
        self.set_label_text(
            self.label_error_status,
            error_status_message(self.last_error),
        )

    def configure_autonomous_direction_buttons(self):
        """Show only one autonomous button while keeping direction internal."""
        reverse_visible, forward_visible = direction_button_visibility(
            self.operator_step,
            self.allowed_commands,
        )
        self.btn_auto_reverse.setVisible(reverse_visible)
        self.btn_auto_forward.setVisible(forward_visible)
        self.btn_auto_reverse.setText("이전 위치로 이동")
        self.btn_auto_forward.setText("이전 위치로 이동")

    def update_map_render_policy(self):
        """Allow heavy map updates only after autonomous pages settle."""
        should_allow = self.operator_step in {
            "AUTONOMOUS_READY",
            "AUTONOMOUS_DRIVING",
        }
        if not should_allow:
            self.map_enable_timer.stop()
            self.allow_map_updates = False
            return
        if self.allow_map_updates or self.map_enable_timer.isActive():
            return
        self.map_enable_timer.start(500)

    def enable_deferred_map_updates(self):
        """Enable queued map updates after the autonomous page is visible."""
        self.allow_map_updates = True
        self.flush_pending_map_updates()

    def handle_map_message(self, msg):
        """Cache or forward map data based on the current render policy."""
        self.pending_map_message = msg
        if self.allow_map_updates:
            self.map_view.update_map(msg)

    def handle_pose_message(self, msg):
        """Cache or forward pose data based on the current render policy."""
        self.pending_pose_message = msg
        if self.allow_map_updates:
            self.map_view.update_pose(msg)

    def handle_path_message(self, msg):
        """Cache or forward path data based on the current render policy."""
        self.pending_path_message = msg
        if self.allow_map_updates:
            self.map_view.update_path(msg)

    def flush_pending_map_updates(self):
        """Apply the most recent deferred autonomous-visualization messages."""
        if self.pending_map_message is not None:
            self.map_view.update_map(self.pending_map_message)
        if self.pending_pose_message is not None:
            self.map_view.update_pose(self.pending_pose_message)
        if self.pending_path_message is not None:
            self.map_view.update_path(self.pending_path_message)

    def update_button_enabled_state(self, enabled):
        """Refresh button enablement from ``allowed_commands``."""
        enabled = bool(enabled) and not self.is_command_request_running()
        for button, (command, _enforce_allowed) in self.command_button_map.items():
            button.setEnabled(enabled and command in self.allowed_commands)

        self.btn_follow.setEnabled(
            enabled and follow_mode_available(self.allowed_commands)
        )
        self.btn_autonomous.setEnabled(
            enabled and "prepare_autonomous" in self.allowed_commands
        )
        self.save_path_check.setEnabled(
            enabled and follow_mode_available(self.allowed_commands)
        )

    def handle_joystick(self, x_axis, y_axis):
        """Store the latest joystick position for periodic publishing."""
        self.joystick_x_axis = float(x_axis)
        self.joystick_y_axis = float(y_axis)
        if not self.is_manual_control_allowed():
            self.joystick_x_axis = 0.0
            self.joystick_y_axis = 0.0
            self.mode_manager_client.publish_cmd(0.0, 0.0)

    def publish_teleop(self):
        """Publish manual velocity commands only when the workflow permits it."""
        if not self.is_manual_control_allowed():
            return
        if not self.joystick_x_axis and not self.joystick_y_axis:
            return
        linear, angular = map_joystick_to_cmd(
            self.joystick_x_axis,
            self.joystick_y_axis,
            self.teleop_config,
        )
        self.mode_manager_client.publish_cmd(linear, angular)

    def is_manual_control_allowed(self):
        """Return whether manual joystick control is allowed right now."""
        return self.manual_control_enabled

    def refresh_manual_control(self):
        """Enable the joystick only when manual control is allowed."""
        self.joystick_widget.setEnabled(self.is_manual_control_allowed())

    def update_velocity_display(self, lin_vel, ang_vel):
        """Refresh the top velocity readouts."""
        if abs(lin_vel) < 0.005:
            lin_vel = 0.0
        if abs(ang_vel) < 0.005:
            ang_vel = 0.0
        self.set_label_text(
            self.label_linear_vel,
            f"선속도: {lin_vel:+.2f} m/s",
        )
        self.set_label_text(
            self.label_angular_vel,
            f"각속도: {ang_vel:+.2f} rad/s",
        )

    @staticmethod
    def set_label_text(label, text):
        """Update one label only when the text actually changed."""
        if label.text() != text:
            label.setText(text)

    def update_auto_ready_card(self, badge_text, title, status, hint):
        """Refresh the shared autonomous-preparation card."""
        self.set_label_text(self.label_auto_ready_badge, badge_text)
        self.set_label_text(self.label_auto_ready_title, title)
        self.set_label_text(self.label_auto_ready_status, status)
        self.set_label_text(self.label_auto_ready_hint, hint)

    def set_robot_status_style(self, kind):
        """Apply one of the predefined top-card color styles."""
        styles = {
            "idle": ("#FFFFFF", "#D8E0EA", "#111827"),
            "warning": ("#FEF3C7", "#F59E0B", "#B45309"),
            "ready": ("#D1FAE5", "#34D399", "#047857"),
            "error": ("#FEE2E2", "#F87171", "#B91C1C"),
        }
        bg, border, fg = styles.get(kind, styles["idle"])
        self.label_robot_status.setStyleSheet(
            f"background-color: {bg};"
            f"border: 1px solid {border};"
            "border-radius: 8px;"
            "padding: 18px 22px;"
            f"color: {fg};"
            "font-weight: bold;"
        )

    def closeEvent(self, event):
        """Stop manual motion output and ROS resources before closing."""
        if self.active_command_worker is not None:
            self.active_command_worker.wait(500)
            self.active_command_worker.deleteLater()
            self.active_command_worker = None
        self.mode_manager_client.publish_cmd(0.0, 0.0)
        self.mode_manager_client.stop()
        super().closeEvent(event)


class OperatorDashboard(ResponsiveDashboardMixin, SafetyDashboardMixin, RobotDashboard):
    """Production operator window with responsive safety controls."""


def main(args=None):
    """Run the Qt application and connect it to the ROS bridge."""
    global ACTIVE_MAIN_WINDOW
    app = None
    window = None
    try:
        app = QApplication(sys.argv if args is None else args)
        window = OperatorDashboard()
        ACTIVE_MAIN_WINDOW = window
        signal.signal(signal.SIGINT, close_active_window_on_sigint)
        window.show_for_available_screen()
        return app.exec_()
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"operator_gui crashed: {exc}", file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        ACTIVE_MAIN_WINDOW = None
        if window is not None:
            try:
                if window.isVisible():
                    window.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
