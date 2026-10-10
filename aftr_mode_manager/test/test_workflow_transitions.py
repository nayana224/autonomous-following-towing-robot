# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for workflow transition policy."""

from aftr_mode_manager.mode_state import RobotMode
from aftr_mode_manager.workflow.transitions import ACTIVE_NAV2_MODES
from aftr_mode_manager.workflow.transitions import AUTONOMOUS_MODES
from aftr_mode_manager.workflow.transitions import is_transition_allowed
from aftr_mode_manager.workflow.transitions import next_drive_direction


def test_expected_workflow_transitions_are_allowed():
    assert is_transition_allowed(RobotMode.IDLE, RobotMode.FOLLOW)
    assert is_transition_allowed(
        RobotMode.IDLE,
        RobotMode.RECORDING_FOLLOW,
    )
    assert is_transition_allowed(
        RobotMode.RECORDING_FOLLOW,
        RobotMode.ALIGNMENT,
    )
    assert is_transition_allowed(
        RobotMode.AUTONOMOUS_READY,
        RobotMode.AUTONOMOUS_DRIVING,
    )


def test_invalid_workflow_transitions_are_rejected():
    assert not is_transition_allowed(
        RobotMode.FOLLOW,
        RobotMode.AUTONOMOUS_DRIVING,
    )
    assert not is_transition_allowed(
        RobotMode.IDLE,
        RobotMode.AUTONOMOUS_DRIVING,
    )
    assert not is_transition_allowed(
        RobotMode.ERROR,
        RobotMode.IDLE,
    )


def test_mode_groups_match_existing_workflow_scope():
    assert AUTONOMOUS_MODES == {
        RobotMode.AUTONOMOUS_READY,
        RobotMode.AUTONOMOUS_DRIVING,
    }
    assert ACTIVE_NAV2_MODES == {
        RobotMode.ALIGNMENT,
        RobotMode.LOCALIZING,
        RobotMode.AUTONOMOUS_READY,
        RobotMode.AUTONOMOUS_DRIVING,
    }


def test_next_drive_direction_preserves_existing_toggle():
    assert next_drive_direction("reverse") == "forward"
    assert next_drive_direction("forward") == "reverse"
    assert next_drive_direction("") == "reverse"
