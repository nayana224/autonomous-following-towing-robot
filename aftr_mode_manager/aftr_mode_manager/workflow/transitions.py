# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Workflow transition policy."""

from aftr_mode_manager.mode_state import RobotMode


AUTONOMOUS_MODES = {
    RobotMode.AUTONOMOUS_READY,
    RobotMode.AUTONOMOUS_DRIVING,
}

ACTIVE_NAV2_MODES = {
    RobotMode.ALIGNMENT,
    RobotMode.LOCALIZING,
    RobotMode.AUTONOMOUS_READY,
    RobotMode.AUTONOMOUS_DRIVING,
}


ALLOWED_TRANSITIONS = {
    RobotMode.IDLE: {
        RobotMode.FOLLOW,
        RobotMode.RECORDING_FOLLOW,
        RobotMode.LOCALIZING,
    },
    RobotMode.FOLLOW: {
        RobotMode.IDLE,
    },
    RobotMode.RECORDING_FOLLOW: {
        RobotMode.IDLE,
        RobotMode.ALIGNMENT,
    },
    RobotMode.ALIGNMENT: {
        RobotMode.IDLE,
        RobotMode.LOCALIZING,
        RobotMode.AUTONOMOUS_READY,
    },
    RobotMode.LOCALIZING: {
        RobotMode.IDLE,
        RobotMode.ALIGNMENT,
        RobotMode.AUTONOMOUS_READY,
    },
    RobotMode.AUTONOMOUS_READY: {
        RobotMode.IDLE,
        RobotMode.ALIGNMENT,
        RobotMode.AUTONOMOUS_DRIVING,
    },
    RobotMode.AUTONOMOUS_DRIVING: {
        RobotMode.IDLE,
        RobotMode.ALIGNMENT,
        RobotMode.AUTONOMOUS_DRIVING,
    },
}


def is_transition_allowed(
    current_mode: RobotMode,
    next_mode: RobotMode,
) -> bool:
    return next_mode in ALLOWED_TRANSITIONS.get(current_mode, set())


def next_drive_direction(current_direction: str) -> str:
    return "forward" if current_direction == "reverse" else "reverse"
