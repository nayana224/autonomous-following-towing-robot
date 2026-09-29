# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for bounded stale-runtime confirmation and recovery."""

import threading

from aftr_mode_manager.workflow.manager import WorkflowManagerMixin


class _Logger:
    """Collect log messages without a ROS node."""

    def __init__(self):
        """Initialize the captured message list."""
        self.messages = []

    def info(self, message):
        """Capture an informational message."""
        self.messages.append(("info", message))

    def warning(self, message):
        """Capture a warning message."""
        self.messages.append(("warning", message))

    def error(self, message):
        """Capture an error message."""
        self.messages.append(("error", message))

    def fatal(self, message):
        """Capture a fatal message."""
        self.messages.append(("fatal", message))


class _AuditHarness(WorkflowManagerMixin):
    """Provide deterministic graph snapshots to the audit workflow."""

    def __init__(self, snapshots):
        """Store graph snapshots returned on successive checks."""
        self.snapshots = list(snapshots)
        self.stale_runtime_audit_count = 0
        self.stale_runtime_audit_attempts = 2
        self.stale_runtime_cleanup_attempted = False
        self.stale_runtime_audit_lock = threading.Lock()
        self.cleanup_stale_runtime_processes = True
        self.stale_runtime_cleanup_timeout_sec = 0.1
        self.managed_process_registry_dir = "/unused"
        self.finished = False
        self.logger = _Logger()

    def visible_node_names(self):
        """Return the next predetermined graph snapshot."""
        return self.snapshots.pop(0)

    def finish_stale_runtime_audit(self):
        """Record that startup was allowed to continue."""
        self.finished = True

    def get_logger(self):
        """Return the capturing logger."""
        return self.logger


def test_transient_graph_entry_is_rechecked_instead_of_fatal_shutdown():
    """One stale discovery sample must not terminate mode_manager."""
    harness = _AuditHarness(
        snapshots=[["/robot_state_publisher"], []],
    )

    harness.audit_stale_runtime_nodes_once()
    assert not harness.finished
    assert harness.stale_runtime_audit_count == 1

    harness.audit_stale_runtime_nodes_once()
    assert harness.finished


def test_verified_cleanup_gets_a_graph_settle_window(monkeypatch):
    """Successful cleanup must be followed by another graph check."""
    cleanup_calls = []
    monkeypatch.setattr(
        "aftr_mode_manager.workflow.manager.cleanup_registered_process_groups",
        lambda registry_dir, timeout_sec: (
            cleanup_calls.append((registry_dir, timeout_sec)) or True,
            ["stopped stale managed process base_bringup: pgid=10"],
        ),
    )
    harness = _AuditHarness(
        snapshots=[
            [],
        ],
    )

    harness.audit_stale_runtime_nodes_once()
    assert cleanup_calls == [("/unused", 0.1)]
    assert not harness.finished
    assert harness.stale_runtime_audit_count == 0

    harness.audit_stale_runtime_nodes_once()
    assert harness.finished


def test_terminal_path_failure_classification():
    """Only non-recoverable replay events should leave driving mode."""
    assert WorkflowManagerMixin.is_terminal_path_failure_event("goal_rejected")
    assert WorkflowManagerMixin.is_terminal_path_failure_event(
        "blocked: obstacle did not clear before retry limit (3/3)"
    )
    assert WorkflowManagerMixin.is_terminal_path_failure_event(
        "failed: not enough remaining path points to retry"
    )
    assert not WorkflowManagerMixin.is_terminal_path_failure_event(
        "blocked by obstacle; retry 1 scheduled"
    )
