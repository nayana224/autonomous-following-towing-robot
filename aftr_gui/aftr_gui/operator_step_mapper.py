# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""GUI presentation rules derived from ``mode_manager`` operator steps."""

from __future__ import annotations


def page_name_for_step(step: str) -> str | None:
    """Map an ``operator_step`` value to a Qt Designer page attribute."""
    pages = {
        "HOME": "page_home",
        "FOLLOWING": "page_follow",
        "RECORDING_FOLLOW": "page_recording_follow",
        "ALIGNMENT_DECISION": "page_after_recording",
        "ALIGNING": "page_alignment",
        "LOCALIZING": "page_autonomous_ready",
        "AUTONOMOUS_READY": "page_autonomous_ready",
        "AUTONOMOUS_DRIVING": "page_autonomous_driving",
        "ERROR": "page_error",
    }
    return pages.get(step)


def busy_status_message(last_command: str) -> str:
    """Return a stable busy message for the current mode-manager command."""
    messages = {
        "auto_start_base": "기본 시스템을 시작하고 있습니다.",
        "start_base": "기본 시스템을 시작하고 있습니다.",
        "check_base_ready": "기본 시스템 준비 상태를 확인하고 있습니다.",
        "start_follow": "추종 모드를 준비하고 있습니다.",
        "start_recording_follow": "경로 저장 추종을 준비하고 있습니다.",
        "finish_recording_for_alignment": (
            "경로와 지도를 저장한 뒤 위치 추정을 준비하고 있습니다."
        ),
        "start_alignment": "정렬 모드로 전환하고 있습니다.",
        "skip_alignment": "정렬 없이 현재 위치를 기준으로 자율주행을 준비하고 있습니다.",
        "finish_alignment": (
            "정렬된 위치를 저장하고 자율주행을 준비하고 있습니다."
        ),
        "save_pose": "현재 위치를 저장하고 있습니다.",
        "start_localizing": "위치 추정을 시작하고 있습니다.",
        "set_autonomous_ready": "자율주행 준비 상태를 확인하고 있습니다.",
        "prepare_autonomous": "자율주행을 준비하고 있습니다.",
        "path_forward": "이전 위치로 이동하고 있습니다.",
        "path_reverse": "이전 위치로 이동하고 있습니다.",
        "path_forward_auto": "이전 위치로 이동할 준비를 하고 있습니다.",
        "path_reverse_auto": "이전 위치로 이동할 준비를 하고 있습니다.",
        "stop": "현재 작업을 중단하고 초기 화면으로 돌아가고 있습니다.",
        "clear_error": "오류를 해제하고 있습니다.",
        "shutdown": "시스템을 종료하고 있습니다.",
    }
    return messages.get(
        last_command,
        "요청을 처리하고 있습니다. 잠시만 기다려 주세요.",
    )


def busy_page_title(last_command: str, current_mode: str) -> str | None:
    """Return an optional page title override while one command is busy."""
    if last_command == "stop" and current_mode in {
        "LOCALIZING",
        "AUTONOMOUS_READY",
        "AUTONOMOUS_DRIVING",
        "ALIGNMENT",
    }:
        return "주행 중단 및 초기 화면 이동"
    return None


def busy_page_message(
    last_command: str,
    current_mode: str,
    operator_message: str = "",
) -> str | None:
    """Return an optional page message override while one command is busy."""
    detail = str(operator_message).strip()
    if detail:
        return detail
    if last_command == "stop" and current_mode in {
        "LOCALIZING",
        "AUTONOMOUS_READY",
        "AUTONOMOUS_DRIVING",
        "ALIGNMENT",
    }:
        return "현재 작업을 안전하게 중단하고 초기 화면으로 돌아가고 있습니다."
    return None


def alignment_decision_message(last_command: str) -> str:
    """Return the alignment-decision guidance message."""
    if last_command == "path_completed_alignment":
        return (
            "목적지에 도착했습니다.\n"
            "정렬을 진행할지 선택해 주세요."
        )
    return (
        "위치 추정이 준비되었습니다.\n"
        "정렬을 진행할지 선택해 주세요."
    )


def alignment_decision_title(last_command: str) -> str:
    """Return the title used on the shared alignment-decision page."""
    if last_command == "path_completed_alignment":
        return "주행이 완료되었습니다"
    return "경로 저장이 완료되었습니다"


def autonomous_page_title(step: str) -> str:
    """Return the title used on the autonomous preparation page."""
    if step == "LOCALIZING":
        return "자율주행 준비 중"
    return "자율주행 준비 완료"


def autonomous_ready_message(_allowed_commands: set[str]) -> str:
    """Return the autonomous-ready guidance message."""
    return "자율주행 준비가 완료되었습니다. 이전 위치로 이동할 수 있습니다."


def error_status_message(_last_error: str) -> str:
    """Return a Korean error-page message without exposing raw backend text."""
    return "오류가 발생했습니다. 오류를 해제한 뒤 처음부터 다시 시작하세요."


def mode_text(mode: str) -> str:
    """Return the Korean operator-facing label for one robot mode."""
    labels = {
        "IDLE": "대기 중",
        "FOLLOW": "작업자 추종 중",
        "RECORDING_FOLLOW": "작업자 추종 및 경로 저장 중",
        "ALIGNMENT": "수동 정렬 중",
        "LOCALIZING": "위치 추정 중",
        "AUTONOMOUS_READY": "자율주행 준비 완료",
        "AUTONOMOUS_DRIVING": "자율주행 중",
        "ERROR": "오류 발생",
    }
    return labels.get(mode, "상태 확인 중")


def direction_button_visibility(
    operator_step: str,
    allowed_commands: set[str],
) -> tuple[bool, bool]:
    """Return visibility for the shared autonomous-drive button slot."""
    ready = operator_step == "AUTONOMOUS_READY"
    reverse_allowed = "path_reverse_auto" in allowed_commands
    forward_allowed = "path_forward_auto" in allowed_commands
    if ready and reverse_allowed:
        return True, False
    if ready and forward_allowed:
        return False, True
    return False, False


def follow_mode_available(allowed_commands: set[str]) -> bool:
    """Return whether the home follow button should be enabled."""
    return (
        "follow" in allowed_commands
        or "recording_follow" in allowed_commands
    )


def robot_status_kind(mode: str) -> str:
    """Map a mode-manager mode into the top status-card palette."""
    if mode == "ERROR":
        return "error"
    if mode in {"ALIGNMENT", "LOCALIZING"}:
        return "warning"
    if mode in {"AUTONOMOUS_READY", "AUTONOMOUS_DRIVING"}:
        return "ready"
    return "idle"
