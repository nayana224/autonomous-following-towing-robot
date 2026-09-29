# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Safety-aware operator GUI with a frozen fall-detection snapshot."""

import os
import signal
import sys
import traceback

from PyQt5 import uic
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QDialog
from sensor_msgs.msg import Image

from aftr_gui import operator_gui
from aftr_gui.core.mode_manager_client import COMMANDS, GuiCommand, ModeManagerClient
from aftr_gui.status_parser import as_bool, parse_status_payload


COMMANDS.setdefault(
    "clear_safety_stop",
    GuiCommand(
        "clear_safety_stop",
        "안전 정지 해제",
        "/mode_manager/command/clear_safety_stop",
        timeout_sec=10.0,
    ),
)


class SafetyModeManagerClient(ModeManagerClient):
    """Cache the latest camera frame so the GUI can freeze it on a safety event."""

    fall_snapshot_ready = pyqtSignal(object)

    def __init__(self):
        self.latest_color_image = None
        super().__init__()
        self.node.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._handle_color_image,
            1,
        )

    def _handle_color_image(self, message):
        self.latest_color_image = message

    def capture_fall_snapshot(self):
        """Freeze the latest camera frame when safety stop first activates."""
        if self.latest_color_image is not None:
            self.fall_snapshot_ready.emit(self.latest_color_image)


class SafetyRobotDashboard(operator_gui.RobotDashboard):
    """Add the safety page and adapt the dashboard to the available screen."""

    DESIGN_WIDTH = 1256
    DESIGN_HEIGHT = 900
    COMPACT_HEIGHT_THRESHOLD = 980

    def initialize_display_state(self):
        """Reset safety indicators before the dashboard receives status."""
        super().initialize_display_state()
        self.safety_stop_active = False
        self.safety_stop_in_progress = False
        self.fall_detector_alive = False
        self.fall_detection_status = "UNKNOWN"
        self.safety_interrupted_mode = ""
        self.safety_snapshot_pixmap = None

    def __init__(self):
        super().__init__()
        self._load_safety_page()
        self._apply_responsive_layout()
        self.mode_manager_client.fall_snapshot_ready.connect(self._show_fall_snapshot)
        self.btn_clear_safety_stop.clicked.connect(self._request_safety_clear)

    def start_base_once(self):
        """Do nothing because mode_manager now owns automatic base startup."""

    def _apply_responsive_layout(self):
        """Scale spacing, typography, and joystick size to the usable screen area."""
        screen = QApplication.primaryScreen()
        if screen is None:
            return

        available = screen.availableGeometry()
        width_scale = available.width() / self.DESIGN_WIDTH
        height_scale = available.height() / self.DESIGN_HEIGHT
        scale = max(0.72, min(1.0, width_scale, height_scale))
        compact = available.height() < self.COMPACT_HEIGHT_THRESHOLD or scale < 0.95

        margin = max(10, round(30 * scale))
        spacing = max(8, round(18 * scale))
        top_spacing = max(12, round(28 * scale))

        self.mainVerticalLayout.setContentsMargins(margin, margin, margin, margin)
        self.mainVerticalLayout.setSpacing(spacing)
        self.topHorizontalLayout.setSpacing(top_spacing)
        self.statusVerticalLayout.setSpacing(max(6, round(12 * scale)))
        self.tabContentLayout.setContentsMargins(
            max(8, round(12 * scale)),
            max(8, round(12 * scale)),
            max(8, round(12 * scale)),
            max(8, round(12 * scale)),
        )
        self.homeLayout.setSpacing(max(8, round(12 * scale)))
        self.homeButtonsLayout.setSpacing(max(8, round(12 * scale)))

        joystick_size = max(190, min(320, round(320 * scale)))
        self.joystick_widget.setMinimumSize(170, 170)
        self.joystick_widget.setMaximumSize(joystick_size, joystick_size)

        label_font = max(13, round(18 * scale))
        button_font = max(30, round(48 * scale))
        checkbox_font = max(21, round(30 * scale))
        button_padding = max(8, round(18 * scale))
        card_padding_v = max(8, round(18 * scale))
        card_padding_h = max(12, round(22 * scale))
        indicator_size = max(24, round(34 * scale))

        responsive_style = f"""
            QLabel {{
                font-size: {label_font}pt;
            }}
            QLabel#label_linear_vel,
            QLabel#label_angular_vel,
            QLabel#label_robot_status {{
                padding: {card_padding_v}px {card_padding_h}px;
            }}
            QTabBar::tab {{
                padding: {max(5, round(8 * scale))}px {max(10, round(18 * scale))}px;
            }}
            QCheckBox#save_path_check {{
                padding: {card_padding_v}px {card_padding_h}px;
                font-size: {checkbox_font}px;
                spacing: {max(8, round(16 * scale))}px;
            }}
            QCheckBox#save_path_check::indicator {{
                width: {indicator_size}px;
                height: {indicator_size}px;
            }}
            QPushButton {{
                padding: {button_padding}px;
            }}
            QPushButton#btn_follow,
            QPushButton#btn_autonomous {{
                font-size: {button_font}px;
            }}
        """
        self.setStyleSheet(self.styleSheet() + responsive_style)

        if compact:
            self.mainVerticalLayout.setStretch(0, 1)
            self.mainVerticalLayout.setStretch(1, 3)
            self.homeLayout.setStretch(0, 1)
            self.homeLayout.setStretch(1, 3)

        target_width = min(self.DESIGN_WIDTH, available.width())
        target_height = min(self.DESIGN_HEIGHT, available.height())
        self.resize(target_width, target_height)

    def show_for_available_screen(self):
        """Show maximized only when the usable screen is smaller than the design."""
        screen = QApplication.primaryScreen()
        if screen is None:
            self.show()
            return

        available = screen.availableGeometry()
        if (
            available.width() < self.DESIGN_WIDTH
            or available.height() < self.DESIGN_HEIGHT
        ):
            self.showMaximized()
        else:
            self.show()

    def _load_safety_page(self):
        """Load the complete safety page from a Qt Designer ``.ui`` file."""
        ui_path = os.path.join(self.share_dir, "gui", "safety_stop.ui")
        self.page_safety_stop = uic.loadUi(ui_path)
        self.label_safety_image = self.page_safety_stop.label_safety_image
        self.label_safety_title = self.page_safety_stop.label_safety_title
        self.label_safety_message = self.page_safety_stop.label_safety_message
        self.label_safety_detail = self.page_safety_stop.label_safety_detail
        self.btn_clear_safety_stop = self.page_safety_stop.btn_clear_safety_stop
        self.stackedWidget.addWidget(self.page_safety_stop)

    def update_mode_status(self, status_text):
        """Update safety state and capture a snapshot on latch entry."""
        values = parse_status_payload(status_text)
        was_active = self.safety_stop_active
        self.safety_stop_active = as_bool(values.get("safety_stop_active", False))
        self.safety_stop_in_progress = as_bool(
            values.get("safety_stop_in_progress", False)
        )
        self.fall_detector_alive = as_bool(values.get("fall_detector_alive", False))
        self.fall_detection_status = str(
            values.get("fall_detection_status", "UNKNOWN")
        )
        self.safety_interrupted_mode = str(
            values.get("safety_interrupted_mode", "")
        )
        super().update_mode_status(status_text)

        if self.safety_stop_active:
            if not was_active:
                self.safety_snapshot_pixmap = None
                self.label_safety_image.setText("감지 스냅샷을 준비하고 있습니다.")
                self.mode_manager_client.capture_fall_snapshot()
            self._render_safety_page()
        elif was_active:
            self.safety_snapshot_pixmap = None
            self.label_safety_image.clear()
            self.label_safety_image.setText("감지 스냅샷을 준비하고 있습니다.")

    def _render_safety_page(self):
        self.stackedWidget.setCurrentWidget(self.page_safety_stop)
        detector = "정상" if self.fall_detector_alive else "연결 확인 필요"
        observation_labels = {
            "FALL_DETECTED": "쓰러짐 감지",
            "FALL_CANDIDATE": "쓰러짐 여부 확인 중",
            "NORMAL": "정상 자세",
            "NO_PERSON": "사람이 보이지 않음",
            "PARTIAL_POSE": "자세 확인 중",
            "STALE_IMAGE": "카메라 영상 확인 필요",
            "INFERENCE_ERROR": "추론 상태 확인 필요",
        }
        observation = observation_labels.get(
            self.fall_detection_status,
            self.fall_detection_status or "확인 중",
        )
        interrupted = self.safety_interrupted_mode or "대기 중"
        stopping = "정지 처리 중" if self.safety_stop_in_progress else "정지 완료"
        self.label_safety_detail.setText(
            f"로봇 동작: {stopping}\n"
            f"긴급 정지 전 상태: {interrupted}\n"
            f"감지기 연결: {detector}\n"
            f"현재 관측: {observation}"
        )
        self.btn_clear_safety_stop.setEnabled(
            not self.safety_stop_in_progress and not self.is_command_request_running()
        )

    def _show_fall_snapshot(self, message):
        image = self._qimage_from_ros_image(message)
        if image is None:
            self.label_safety_image.setText("감지 이미지를 불러올 수 없습니다.")
            return
        self.safety_snapshot_pixmap = QPixmap.fromImage(image.copy())
        self._scale_safety_snapshot()

    def _scale_safety_snapshot(self):
        if self.safety_snapshot_pixmap is None:
            return
        self.label_safety_image.setPixmap(
            self.safety_snapshot_pixmap.scaled(
                self.label_safety_image.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def resizeEvent(self, event):
        """Keep the safety snapshot fitted to the resized view."""
        super().resizeEvent(event)
        self._scale_safety_snapshot()

    @staticmethod
    def _qimage_from_ros_image(message):
        encoding = str(message.encoding).lower()
        if encoding not in {"rgb8", "bgr8"}:
            return None
        bytes_per_line = int(message.step)
        image = QImage(
            bytes(message.data),
            int(message.width),
            int(message.height),
            bytes_per_line,
            QImage.Format_RGB888,
        )
        return image.rgbSwapped() if encoding == "bgr8" else image

    def _request_safety_clear(self):
        dialog_path = os.path.join(
            self.share_dir,
            "gui",
            "safety_clear_confirm.ui",
        )
        dialog = uic.loadUi(dialog_path, QDialog(self))
        dialog.btn_confirm_cancel.clicked.connect(dialog.reject)
        dialog.btn_confirm_clear.clicked.connect(dialog.accept)
        if dialog.exec_() != QDialog.Accepted:
            return
        self.request_command("clear_safety_stop", enforce_allowed=False)


operator_gui.ModeManagerClient = SafetyModeManagerClient


def main(args=None):
    """Run the safety-aware dashboard and close its ROS client on exit."""
    global_window = None
    try:
        app = QApplication(sys.argv if args is None else args)
        global_window = SafetyRobotDashboard()
        operator_gui.ACTIVE_MAIN_WINDOW = global_window
        signal.signal(signal.SIGINT, operator_gui.close_active_window_on_sigint)
        global_window.show_for_available_screen()
        return app.exec_()
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"safety operator GUI crashed: {exc}", file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        operator_gui.ACTIVE_MAIN_WINDOW = None
        if global_window is not None and global_window.isVisible():
            global_window.close()


if __name__ == "__main__":
    sys.exit(main())
