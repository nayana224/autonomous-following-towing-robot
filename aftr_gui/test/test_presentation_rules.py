# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

"""Tests for pure GUI presentation and input mapping rules."""

from aftr_gui.operator_step_mapper import direction_button_visibility
from aftr_gui.operator_step_mapper import follow_mode_available
from aftr_gui.operator_step_mapper import mode_text
from aftr_gui.operator_step_mapper import page_name_for_step
from aftr_gui.operator_step_mapper import robot_status_kind
from aftr_gui.status_parser import as_bool
from aftr_gui.status_parser import parse_allowed_commands
from aftr_gui.status_parser import parse_status_payload
from aftr_gui.teleop_mapper import TeleopConfig
from aftr_gui.teleop_mapper import map_joystick_to_cmd


def test_status_parser_normalizes_gui_contract_fields():
    """Status helpers should accept the mode-manager payload shapes in use."""
    assert parse_status_payload('{"mode":"IDLE","busy":false}') == {
        "mode": "IDLE",
        "busy": False,
    }
    assert parse_status_payload("") == {}
    assert parse_status_payload("not-json") == {}
    assert as_bool(True)
    assert as_bool("YES")
    assert not as_bool("false")
    assert parse_allowed_commands(["follow", "stop"]) == {"follow", "stop"}
    assert parse_allowed_commands("follow, stop") == {"follow", "stop"}


def test_operator_step_mapping_preserves_gui_page_contract():
    """Operator steps must keep rendering the same Qt Designer pages."""
    assert page_name_for_step("HOME") == "page_home"
    assert page_name_for_step("FOLLOWING") == "page_follow"
    assert page_name_for_step("RECORDING_FOLLOW") == "page_recording_follow"
    assert page_name_for_step("ALIGNMENT_DECISION") == "page_after_recording"
    assert page_name_for_step("ALIGNING") == "page_alignment"
    assert page_name_for_step("LOCALIZING") == "page_autonomous_ready"
    assert page_name_for_step("AUTONOMOUS_READY") == "page_autonomous_ready"
    assert page_name_for_step("AUTONOMOUS_DRIVING") == "page_autonomous_driving"
    assert page_name_for_step("ERROR") == "page_error"
    assert page_name_for_step("UNKNOWN") is None


def test_autonomous_direction_visibility_comes_from_allowed_commands():
    """Only the direction allowed by mode_manager should be visible."""
    assert direction_button_visibility(
        "AUTONOMOUS_READY",
        {"path_reverse_auto"},
    ) == (True, False)
    assert direction_button_visibility(
        "AUTONOMOUS_READY",
        {"path_forward_auto"},
    ) == (False, True)
    assert direction_button_visibility(
        "AUTONOMOUS_DRIVING",
        {"path_reverse_auto"},
    ) == (False, False)


def test_home_follow_and_status_presentation_rules_stay_stable():
    """Home command availability and status styling are presentation-only rules."""
    assert follow_mode_available({"follow"})
    assert follow_mode_available({"recording_follow"})
    assert not follow_mode_available({"stop"})
    assert mode_text("AUTONOMOUS_READY") == "자율주행 준비 완료"
    assert mode_text("UNKNOWN") == "상태 확인 중"
    assert robot_status_kind("ERROR") == "error"
    assert robot_status_kind("LOCALIZING") == "warning"
    assert robot_status_kind("AUTONOMOUS_DRIVING") == "ready"
    assert robot_status_kind("FOLLOW") == "idle"


def test_teleop_mapping_preserves_default_reverse_joystick_behavior():
    """Default joystick mapping must keep the existing velocity signs and scales."""
    config = TeleopConfig()

    linear, angular = map_joystick_to_cmd(0.5, 1.0, config)

    assert linear == -0.20
    assert angular == 0.25


def test_teleop_mapping_preserves_non_reversed_turning_rule():
    """Non-reversed mapping flips steering while backing up."""
    config = TeleopConfig(reverse_joystick=False)

    forward = map_joystick_to_cmd(0.5, 1.0, config)
    reverse = map_joystick_to_cmd(0.5, -1.0, config)

    assert forward == (0.20, 0.25)
    assert reverse == (-0.20, -0.25)
