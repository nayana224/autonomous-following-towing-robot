# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for managed-process exit diagnostics."""

import json
import os
import signal
import subprocess
import time

from aftr_mode_manager.runtime.process_supervisor import ManagedProcess
from aftr_mode_manager.runtime.process_supervisor import (
    cleanup_registered_process_groups,
)


def test_managed_process_retains_exit_code():
    """An unexpected process exit must remain visible to diagnostic logs."""
    process = ManagedProcess(
        name="diagnostic_test",
        command=["/bin/sh", "-c", "exit 7"],
    )
    process.start()

    deadline = time.monotonic() + 2.0
    while process.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)

    assert not process.is_running()
    assert process.last_exit_returncode == 7
    assert process.last_exit_at is not None
    assert "last_exit_returncode=7" in process.describe()


def test_registered_process_group_can_be_recovered_after_owner_loss(tmp_path):
    """A token-verified stale process group must be stopped and unregistered."""
    registry_dir = str(tmp_path / "processes")
    registry_path = str(tmp_path / "processes" / "sleep.json")
    process = ManagedProcess(
        name="recoverable_sleep",
        command=["/bin/sleep", "30"],
        registry_path=registry_path,
    )
    process.start()

    try:
        assert os.path.isfile(registry_path)
        success, messages = cleanup_registered_process_groups(
            registry_dir,
            timeout_sec=0.5,
        )
        assert success
        assert any("stopped stale managed process" in item for item in messages)

        deadline = time.monotonic() + 1.0
        while process.is_running() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert not process.is_running()
        assert not os.path.exists(registry_path)
    finally:
        process.stop(0.2)


def test_cleanup_refuses_process_without_matching_ownership_token(tmp_path):
    """A registry record must never authorize killing an unrelated process."""
    registry_dir = tmp_path / "processes"
    registry_dir.mkdir()
    process = subprocess.Popen(
        ["/bin/sleep", "30"],
        start_new_session=True,
    )
    registry_path = registry_dir / "unverified.json"
    registry_path.write_text(
        json.dumps(
            {
                "name": "unverified",
                "uid": os.getuid(),
                "pgid": process.pid,
                "token": "not-present-in-process-environment",
                "command": ["/bin/sleep", "30"],
            }
        ),
        encoding="utf-8",
    )

    try:
        success, messages = cleanup_registered_process_groups(
            str(registry_dir),
            timeout_sec=0.1,
        )
        assert not success
        assert process.poll() is None
        assert any("refused unverified process group" in item for item in messages)
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=1.0)
