# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Safety page and status handling for the operator dashboard."""

import os

from PyQt5 import uic
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QDialog

from aftr_gui.core.safety_client import SafetyModeManagerClient
from aftr_gui.status_parser import as_bool, parse_status_payload


class SafetyDashboardMixin:
    """Add the safety page and safety status handling."""

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
        super().__init__(client_factory=SafetyModeManagerClient)
        self._load_safety_page()
        self._apply_responsive_layout()
        self.mode_manager_client.fall_snapshot_ready.connect(self._show_fall_snapshot)
        self.btn_clear_safety_stop.clicked.connect(self._request_safety_clear)

    def start_base_once(self):
        """Do nothing because mode_manager now owns automatic base startup."""



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
