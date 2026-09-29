# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for safe mode convergence after managed-process failure."""

from types import SimpleNamespace

from aftr_mode_manager.mode_manager_node import ModeManagerNode
from aftr_mode_manager.mode_state import RobotMode


class _Process:
    """Minimal stoppable managed-process double."""

    def __init__(self, running=True):
        """Store the initial running state."""
        self.running = running
        self.stop_calls = 0

    def is_running(self):
        """Return the current running state."""
        return self.running

    def stop(self, _timeout_sec):
        """Record one stop and clear the running state."""
        self.stop_calls += 1
        self.running = False
        return True


def test_path_manager_exit_during_driving_stops_motion_and_sets_error():
    """An action-client crash must never leave Nav2 driving unattended."""
    manager = ModeManagerNode.__new__(ModeManagerNode)
    manager.status = SimpleNamespace(
        mode=RobotMode.AUTONOMOUS_DRIVING,
        follower_running=False,
        follower_ready=False,
        nav2_running=True,
        nav2_ready=True,
        amcl_pose_ready=True,
    )
    manager.follower_node = _Process(running=False)
    manager.nav2_process = _Process(running=True)
    manager.shutdown_timeout_sec = 0.1
    manager.active_drive_direction = "reverse"
    manager.events = []
    manager.cancel_alignment_confirmation_request = lambda: None
    manager.notify_autonomous_failed = lambda: manager.events.append("audio_failed")
    manager.notify_autonomous_failed_led = lambda: manager.events.append("led_failed")
    manager.stop_autonomous_audio = lambda: manager.events.append("audio_stop")
    manager.log_nav2_stop_request = lambda reason: manager.events.append(reason)
    manager.set_error = lambda message: manager.events.append(("error", message))

    manager.handle_unexpected_managed_process_exit("path_manager")

    assert manager.nav2_process.stop_calls == 1
    assert manager.active_drive_direction == ""
    assert manager.status.nav2_ready is False
    assert manager.status.amcl_pose_ready is False
    assert "audio_failed" in manager.events
    assert any(
        isinstance(event, tuple) and event[0] == "error"
        for event in manager.events
    )


def test_irrelevant_process_exit_does_not_change_active_mode():
    """RViz loss must not stop an otherwise healthy autonomous runtime."""
    manager = ModeManagerNode.__new__(ModeManagerNode)
    manager.status = SimpleNamespace(mode=RobotMode.AUTONOMOUS_DRIVING)
    manager.follower_node = _Process(running=False)
    manager.nav2_process = _Process(running=True)
    manager.active_drive_direction = "forward"

    manager.handle_unexpected_managed_process_exit("rviz")

    assert manager.nav2_process.stop_calls == 0
    assert manager.active_drive_direction == "forward"


def test_path_rejection_during_start_moves_ready_mode_to_error():
    """An early action rejection must not leave the GUI on READY."""
    manager = ModeManagerNode.__new__(ModeManagerNode)
    manager.status = SimpleNamespace(mode=RobotMode.AUTONOMOUS_READY)
    manager.pending_drive_direction = "reverse"
    manager.active_drive_direction = ""
    manager.last_path_follow_event = "reverse path following started"
    manager.events = []
    manager.notify_autonomous_failed = lambda: manager.events.append("audio_failed")
    manager.notify_autonomous_failed_led = lambda: manager.events.append("led_failed")
    manager.set_error = lambda message: manager.events.append(("error", message))

    manager.path_status_callback(
        SimpleNamespace(
            data=(
                '{"following_path": false, '
                '"last_follow_event": "goal_rejected"}'
            )
        )
    )

    assert manager.pending_drive_direction == ""
    assert manager.active_drive_direction == ""
    assert "audio_failed" in manager.events
    assert any(
        event[0] == "error"
        for event in manager.events
        if isinstance(event, tuple)
    )


def test_path_completion_during_start_moves_ready_mode_to_alignment():
    """A fast completion must reach the completion page despite start races."""
    manager = ModeManagerNode.__new__(ModeManagerNode)
    manager.status = SimpleNamespace(
        mode=RobotMode.AUTONOMOUS_READY,
        last_command="path_reverse_auto",
        transition_count=3,
    )
    manager.pending_drive_direction = "reverse"
    manager.active_drive_direction = ""
    manager.last_path_follow_event = "reverse path following started"
    manager.alignment_control_active = True
    manager.notify_autonomous_arrived = lambda: None
    manager.notify_autonomous_arrived_led = lambda: None
    manager.schedule_alignment_confirmation_request = lambda: None
    manager.publish_status = lambda: None
    manager.get_logger = lambda: SimpleNamespace(info=lambda _message: None)

    manager.path_status_callback(
        SimpleNamespace(
            data='{"following_path": false, "last_follow_event": "completed"}'
        )
    )

    assert manager.status.mode == RobotMode.ALIGNMENT
    assert manager.status.last_command == "path_completed_alignment"
    assert manager.next_drive_direction == "forward"
    assert manager.pending_drive_direction == ""


def test_unconfirmed_service_reconciles_when_path_is_already_active():
    """A lost/duplicate service response must still advance the GUI state."""
    manager = ModeManagerNode.__new__(ModeManagerNode)
    manager.status = SimpleNamespace(
        mode=RobotMode.AUTONOMOUS_READY,
        last_command="path_reverse_auto",
        last_error="old error",
    )
    manager.next_drive_direction = "reverse"
    manager.pending_drive_direction = ""
    manager.active_drive_direction = ""
    manager.latest_path_status = {"following_path": True}
    manager.path_replay_service_timeout_sec = 120.0
    manager.path_follow_saved_path_reverse_client = object()
    manager.path_follow_saved_path_client = object()
    manager.last_service_error = "a saved path is already being followed"
    manager.ensure_path_manager_running = lambda _response: True
    manager._require_amcl_pose_ready = lambda _response, _message: True
    manager.check_nav2_ready = lambda _response: SimpleNamespace(success=True)
    manager.wait_for_stable_autonomous_readiness = lambda _response: True
    manager.set_operator_message = lambda _message, publish=False: None
    manager.refresh_local_costmap_before_replay = lambda _response: True
    manager.call_trigger_service = lambda *_args, **_kwargs: False
    manager.notify_autonomous_failed = lambda: None
    manager.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)
    manager.start_autonomous_audio = lambda: None
    manager.start_autonomous_led = lambda: None
    manager.ensure_rviz_running_for_autonomous = lambda: None
    manager.request_mode = lambda response, mode, _command: (
        setattr(manager.status, "mode", mode)
        or setattr(response, "success", True)
        or response
    )

    response = SimpleNamespace(success=False, message="")
    result = manager.start_saved_path_driving(
        response,
        reverse=True,
        command_name="path_reverse_auto",
    )

    assert result.success
    assert manager.status.mode == RobotMode.AUTONOMOUS_DRIVING
    assert manager.active_drive_direction == "reverse"
    assert manager.pending_drive_direction == ""
    assert manager.status.last_error == ""
