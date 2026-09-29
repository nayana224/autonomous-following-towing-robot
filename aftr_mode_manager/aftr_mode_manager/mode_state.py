# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Mode state definitions for AFTR mode manager."""

from dataclasses import dataclass
from enum import Enum
import json


class RobotMode(str, Enum):
    """High-level robot modes managed by the mode manager."""

    IDLE = "IDLE"
    FOLLOW = "FOLLOW"
    RECORDING_FOLLOW = "RECORDING_FOLLOW"
    ALIGNMENT = "ALIGNMENT"
    LOCALIZING = "LOCALIZING"
    AUTONOMOUS_READY = "AUTONOMOUS_READY"
    AUTONOMOUS_DRIVING = "AUTONOMOUS_DRIVING"
    ERROR = "ERROR"


@dataclass
class ModeStatus:
    """Small status object published by the mode manager."""

    mode: RobotMode = RobotMode.IDLE
    busy: bool = False
    base_running: bool = False
    base_ready: bool = False
    slam_running: bool = False
    slam_ready: bool = False
    path_running: bool = False
    path_ready: bool = False
    follower_running: bool = False
    follower_ready: bool = False
    nav2_running: bool = False
    nav2_ready: bool = False
    amcl_pose_ready: bool = False
    last_command: str = ""
    last_error: str = ""
    transition_count: int = 0
    operator_step: str = "HOME"
    operator_message: str = "대기 중입니다."
    tracking_message: str = "추종 상태 대기 중..."
    recording_message: str = "경로 저장 대기 중입니다."
    path_message: str = "주행 대기 중입니다."
    driving_message: str = "주행 상태를 확인 중입니다."
    manual_control_allowed: bool = True
    allowed_commands: tuple[str, ...] = (
        "follow",
        "recording_follow",
    )

    def to_dict(self):
        """Return status values with stable keys for GUI parsing."""
        return {
            "mode": self.mode.value,
            "busy": self.busy,
            "base_running": self.base_running,
            "base_ready": self.base_ready,
            "slam_running": self.slam_running,
            "slam_ready": self.slam_ready,
            "path_running": self.path_running,
            "path_ready": self.path_ready,
            "follower_running": self.follower_running,
            "follower_ready": self.follower_ready,
            "nav2_running": self.nav2_running,
            "nav2_ready": self.nav2_ready,
            "amcl_pose_ready": self.amcl_pose_ready,
            "last_command": self.last_command,
            "last_error": self.last_error,
            "transition_count": self.transition_count,
            "operator_step": self.operator_step,
            "operator_message": self.operator_message,
            "tracking_message": self.tracking_message,
            "recording_message": self.recording_message,
            "path_message": self.path_message,
            "driving_message": self.driving_message,
            "manual_control_allowed": self.manual_control_allowed,
            "allowed_commands": list(self.allowed_commands),
        }

    def to_status_text(self) -> str:
        """Return JSON status text for stable GUI parsing."""
        return json.dumps(self.to_dict(), ensure_ascii=False, separators=(",", ":"))
