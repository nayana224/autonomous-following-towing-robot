# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Safety-aware ROS client for the operator GUI."""

from PyQt5.QtCore import pyqtSignal
from sensor_msgs.msg import Image

from aftr_gui.core.mode_manager_client import COMMANDS, GuiCommand, ModeManagerClient


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
