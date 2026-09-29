# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Build GUI-facing operator state owned by the mode manager.

This module converts internal workflow state into a smaller operator-facing
view model. The GUI is expected to stay passive and render these derived fields
instead of inferring workflow rules on its own.
"""

from dataclasses import dataclass

from aftr_mode_manager.mode_state import RobotMode


@dataclass(frozen=True)
class OperatorViewState:
    """Immutable container for GUI-facing operator state.

    Attributes:
        step: High-level GUI page or step identifier.
        manual_control_allowed: Whether joystick/manual control is allowed now.
        allowed_commands: Public command keys currently allowed by the workflow.
    """

    step: str
    manual_control_allowed: bool
    allowed_commands: tuple[str, ...]


class OperatorStatusMixin:
    """Populate ``ModeStatus`` with GUI-ready operator state and detail labels."""

    def update_operator_status(self):
        """Refresh operator-facing fields before publishing status."""
        self.update_operator_detail_messages()
        view = self.build_operator_view_state()
        self.apply_operator_view_state(view)

    def build_operator_view_state(self):
        """Build one operator-facing state snapshot from internal workflow state."""
        if self.status.busy:
            return OperatorViewState(
                step="BUSY",
                manual_control_allowed=False,
                allowed_commands=(),
            )
        return self.operator_view_for_mode(self.status.mode)

    def apply_operator_view_state(self, view):
        """Apply one computed operator view state onto ``ModeStatus``."""
        self.status.operator_step = view.step
        self.status.manual_control_allowed = view.manual_control_allowed
        self.status.allowed_commands = view.allowed_commands

    def operator_view_for_mode(self, mode):
        """Return the operator-facing GUI state for one workflow mode."""
        if mode == RobotMode.ERROR:
            return self.error_view_state()
        if mode == RobotMode.IDLE:
            return OperatorViewState(
                step="HOME",
                manual_control_allowed=True,
                allowed_commands=(
                    "follow",
                    "recording_follow",
                ),
            )
        if mode == RobotMode.FOLLOW:
            return OperatorViewState(
                step="FOLLOWING",
                manual_control_allowed=False,
                allowed_commands=("stop",),
            )
        if mode == RobotMode.RECORDING_FOLLOW:
            return OperatorViewState(
                step="RECORDING_FOLLOW",
                manual_control_allowed=False,
                allowed_commands=("finish_recording", "stop"),
            )
        if mode == RobotMode.ALIGNMENT:
            return self.alignment_view_state()
        if mode == RobotMode.LOCALIZING:
            return OperatorViewState(
                step="LOCALIZING",
                manual_control_allowed=False,
                allowed_commands=("stop",),
            )
        if mode == RobotMode.AUTONOMOUS_READY:
            return self.autonomous_ready_view_state()
        if mode == RobotMode.AUTONOMOUS_DRIVING:
            return OperatorViewState(
                step="AUTONOMOUS_DRIVING",
                manual_control_allowed=False,
                allowed_commands=("stop",),
            )
        return OperatorViewState(
            step="HOME",
            manual_control_allowed=False,
            allowed_commands=(
                "follow",
                "recording_follow",
            ),
        )

    @staticmethod
    def error_view_state():
        """Return the operator view used while the workflow is in ``ERROR``."""
        return OperatorViewState(
            step="ERROR",
            manual_control_allowed=False,
            allowed_commands=("clear_error",),
        )

    def alignment_view_state(self):
        """Return the operator view used for alignment-related screens."""
        if self.status.last_command == "path_completed_alignment":
            return OperatorViewState(
                step="ALIGNMENT_DECISION",
                manual_control_allowed=False,
                allowed_commands=("alignment", "skip_alignment", "stop"),
            )
        if self.alignment_control_active:
            return OperatorViewState(
                step="ALIGNING",
                manual_control_allowed=True,
                allowed_commands=("finish_alignment", "stop"),
            )
        return OperatorViewState(
            step="ALIGNMENT_DECISION",
            manual_control_allowed=False,
            allowed_commands=("alignment", "skip_alignment", "stop"),
        )

    def autonomous_ready_view_state(self):
        """Return the operator view used while autonomous replay is ready."""
        return OperatorViewState(
            step="AUTONOMOUS_READY",
            manual_control_allowed=False,
            allowed_commands=(f"path_{self.next_drive_direction}_auto", "stop"),
        )

    def update_operator_detail_messages(self):
        """Derive GUI detail labels from cached runtime and workflow state."""
        self.status.tracking_message = self.tracking_text(self.latest_follow_state)
        self.status.recording_message = self.recording_text()
        self.status.path_message = self.path_text()
        self.status.driving_message = self.current_driving_text()

    def recording_text(self):
        """Return stable path-recording status text for the GUI."""
        if bool(self.latest_path_status.get("recording", False)):
            return "경로 저장 중입니다."
        return "경로 저장 대기 중입니다."

    def path_text(self):
        """Return saved-path progress text for the GUI."""
        remaining = self.path_remaining_distance()
        if bool(self.latest_path_status.get("following_path", False)):
            return self.arrival_estimate_text(remaining)
        return "주행 대기 중입니다."

    def current_driving_text(self):
        """Return the current autonomous driving summary text."""
        event = str(self.latest_path_status.get("last_follow_event", "idle"))
        return self.driving_text(event)

    def path_remaining_distance(self):
        """Return remaining saved-path distance from cached path status."""
        try:
            return float(self.latest_path_status.get("remaining_distance_m", 0.0))
        except (TypeError, ValueError):
            return 0.0

    def arrival_estimate_text(self, remaining):
        """Return operator-facing arrival estimate text.

        Args:
            remaining: Remaining path distance in meters.

        Returns:
            Human-readable ETA text for the GUI.
        """
        if remaining <= 0.05:
            return "도착 확인 중입니다."
        speed = max(float(self.latest_linear_speed), 0.20)
        seconds = int(round(float(remaining) / speed))
        if seconds < 60:
            return f"예상 도착까지 약 {seconds}초 남았습니다. · 남은 거리 {remaining:.1f} m"
        minutes = seconds // 60
        rest = seconds % 60
        if rest < 10:
            return f"예상 도착까지 약 {minutes}분 남았습니다. · 남은 거리 {remaining:.1f} m"
        return (
            f"예상 도착까지 약 {minutes}분 {rest}초 남았습니다. "
            f"· 남은 거리 {remaining:.1f} m"
        )

    @staticmethod
    def driving_text(event):
        """Return stable autonomous-driving status text from path events."""
        if "temporary failure" in event or "obstacle" in event or "blocked" in event:
            return "장애물 감지: 정지 후 경로 주행을 재시도합니다."
        if event == "completed":
            return "도착했습니다. 다음 정렬 단계로 전환합니다."
        return "저장 경로를 주행 중입니다."

    @staticmethod
    def tracking_text(state):
        """Return stable person-tracking status text for the GUI."""
        if state == "SEARCH":
            return "사람을 탐색하는 중입니다..."
        if state == "FOLLOW":
            return "사람을 추종 중입니다."
        if state == "OBSTACLE":
            return "장애물을 감지했습니다."
        return str(state) if state else "추종 상태 대기 중..."
