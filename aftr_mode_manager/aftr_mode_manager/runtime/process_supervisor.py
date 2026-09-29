# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Process supervision helpers for external runtime commands.

Every managed command starts in its own Unix session and process group. Shutdown
therefore targets the complete ROS launch tree rather than only the top-level
``ros2 launch`` process. A process is considered stopped only after both the
parent process has been reaped and its process group has disappeared.
"""

import os
import json
import signal
import subprocess
import time
import uuid
from dataclasses import dataclass


@dataclass
class ManagedProcess:
    """Represent one managed OS process group.

    Attributes:
        name: Human-readable runtime name used by the mode manager.
        command: Command list passed directly to ``subprocess.Popen``.
        process: Top-level subprocess handle, or ``None`` after it is reaped.
        process_group_id: Unix process-group ID retained until the entire launch
            tree has disappeared.
    """

    name: str
    command: list[str]
    process: subprocess.Popen | None = None
    process_group_id: int | None = None
    last_exit_returncode: int | None = None
    last_exit_at: float | None = None
    registry_path: str | None = None
    runtime_token: str | None = None

    def is_running(self) -> bool:
        """Return whether the parent or any process in its group is still alive."""
        self._reap_parent_if_exited()
        return self._parent_is_alive() or self._process_group_exists()

    def start(self) -> None:
        """Start the command in a dedicated session and process group.

        ``start_new_session=True`` avoids the thread-safety problems of using
        ``preexec_fn=os.setsid`` inside the multi-threaded mode manager.
        """
        if self.is_running():
            return

        self._clear_handles()
        self.last_exit_returncode = None
        self.last_exit_at = None
        self.runtime_token = uuid.uuid4().hex
        process_env = os.environ.copy()
        process_env["MDBOT_MANAGED_PROCESS"] = "1"
        process_env["MDBOT_RUNTIME_TOKEN"] = self.runtime_token
        self.process = subprocess.Popen(
            self.command,
            start_new_session=True,
            env=process_env,
        )
        # A new session leader's PID is also its process-group ID. Store it
        # independently so descendants can still be terminated after the parent
        # launch process exits.
        self.process_group_id = self.process.pid
        try:
            self._write_registry()
        except OSError:
            self.stop(1.0)
            raise

    def stop(self, timeout_sec: float) -> bool:
        """Stop the complete managed process group with bounded escalation.

        The sequence is ``SIGINT`` -> ``SIGTERM`` -> ``SIGKILL``. Success is
        reported only when the top-level process has been reaped and no process
        remains in the stored process group.
        """
        self._reap_parent_if_exited()
        if not self._parent_is_alive() and not self._process_group_exists():
            self._clear_handles()
            return True

        if self.process_group_id is None:
            # This should only occur for an old/incomplete handle. Reap the
            # parent directly rather than pretending shutdown succeeded.
            return self._wait_for_complete_exit(timeout_sec)

        if self._signal_and_wait(signal.SIGINT, timeout_sec):
            return True
        if self._signal_and_wait(signal.SIGTERM, 2.0):
            return True
        if self._signal_and_wait(signal.SIGKILL, 3.0):
            return True

        # Keep the PID/PGID handles when shutdown fails. This prevents a later
        # start from launching a duplicate stack while descendants still exist.
        return False

    def describe(self) -> str:
        """Return compact process information for shutdown-failure logs."""
        parent_pid = self.process.pid if self.process is not None else None
        return (
            f"name={self.name}, pid={parent_pid}, pgid={self.process_group_id}, "
            f"last_exit_returncode={self.last_exit_returncode}, "
            f"last_exit_at={self.last_exit_at}, command={' '.join(self.command)}"
        )

    def _signal_and_wait(
        self,
        stop_signal: signal.Signals,
        timeout_sec: float,
    ) -> bool:
        """Signal the stored process group and wait for its complete removal."""
        process_group_id = self.process_group_id
        if process_group_id is None:
            return self._wait_for_complete_exit(timeout_sec)

        try:
            os.killpg(process_group_id, stop_signal)
        except ProcessLookupError:
            # The group may have disappeared between the existence check and
            # signal delivery. Still reap the parent before clearing handles.
            pass
        except PermissionError:
            return False

        return self._wait_for_complete_exit(timeout_sec)

    def _wait_for_complete_exit(self, timeout_sec: float) -> bool:
        """Wait until the parent is reaped and the whole process group is gone."""
        deadline = time.monotonic() + max(0.0, float(timeout_sec))

        while True:
            self._reap_parent_if_exited()
            parent_alive = self._parent_is_alive()
            group_alive = self._process_group_exists()
            if not parent_alive and not group_alive:
                self._clear_handles()
                return True

            if time.monotonic() >= deadline:
                return False
            time.sleep(0.1)

    def _parent_is_alive(self) -> bool:
        """Return whether the top-level subprocess has not exited yet."""
        return self.process is not None and self.process.poll() is None

    def _reap_parent_if_exited(self) -> None:
        """Collect an exited top-level child without discarding its PGID."""
        if self.process is None:
            return
        returncode = self.process.poll()
        if returncode is not None:
            # ``poll`` performs waitpid-style collection, preventing a true
            # Linux zombie for the top-level child. Keep the PGID until every
            # descendant has also exited.
            self.last_exit_returncode = int(returncode)
            self.last_exit_at = time.time()
            self.process = None

    def _process_group_exists(self) -> bool:
        """Return whether any process still belongs to the stored group."""
        process_group_id = self.process_group_id
        if process_group_id is None:
            return False

        try:
            os.killpg(process_group_id, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            # The group exists even if the current user cannot signal it.
            return True
        return True

    def _clear_handles(self) -> None:
        """Forget process handles only after complete shutdown is confirmed."""
        self.process = None
        self.process_group_id = None
        self.runtime_token = None
        self._remove_registry()

    def _write_registry(self) -> None:
        """Persist enough ownership data to clean up after a manager crash."""
        if self.registry_path is None:
            return
        registry_dir = os.path.dirname(self.registry_path)
        os.makedirs(registry_dir, mode=0o700, exist_ok=True)
        os.chmod(registry_dir, 0o700)
        payload = {
            "name": self.name,
            "uid": os.getuid(),
            "pgid": self.process_group_id,
            "token": self.runtime_token,
            "command": self.command,
        }
        temporary_path = f"{self.registry_path}.{os.getpid()}.tmp"
        with open(temporary_path, "w", encoding="utf-8") as registry_file:
            json.dump(payload, registry_file, ensure_ascii=False)
            registry_file.flush()
            os.fsync(registry_file.fileno())
        os.replace(temporary_path, self.registry_path)

    def _remove_registry(self) -> None:
        """Remove the ownership record after the process group is gone."""
        if self.registry_path is None:
            return
        try:
            os.unlink(self.registry_path)
        except FileNotFoundError:
            pass


def cleanup_registered_process_groups(registry_dir, timeout_sec=2.0):
    """Stop process groups proven to belong to an older AFTR manager.

    Returns:
        Tuple ``(success, messages)`` describing every registry record.
    """
    if not os.path.isdir(registry_dir):
        return True, ["no managed-process registry directory"]

    success = True
    messages = []
    registry_names = sorted(
        name for name in os.listdir(registry_dir) if name.endswith(".json")
    )
    if not registry_names:
        return True, ["no registered managed process groups"]

    for registry_name in registry_names:
        registry_path = os.path.join(registry_dir, registry_name)
        record_success, message = _cleanup_registered_group(
            registry_path,
            timeout_sec=max(0.0, float(timeout_sec)),
        )
        success = success and record_success
        messages.append(message)
    return success, messages


def _cleanup_registered_group(registry_path, timeout_sec):
    """Validate and stop one registered process group."""
    try:
        with open(registry_path, "r", encoding="utf-8") as registry_file:
            record = json.load(registry_file)
        pgid = int(record["pgid"])
        token = str(record["token"])
        uid = int(record["uid"])
        name = str(record.get("name", os.path.basename(registry_path)))
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        return False, f"invalid managed-process registry {registry_path}: {exc}"

    if uid != os.getuid() or pgid <= 1 or not token:
        return False, f"refused unsafe registry for {name}: uid={uid}, pgid={pgid}"

    members = _process_group_members(pgid)
    if not members:
        _unlink_registry(registry_path)
        return True, f"removed inactive registry for {name}"

    invalid_members = [
        pid for pid in members if not _process_has_runtime_token(pid, uid, token)
    ]
    if invalid_members:
        return (
            False,
            f"refused unverified process group for {name}: "
            f"pgid={pgid}, pids={invalid_members}",
        )

    signal_timeouts = (
        (signal.SIGINT, timeout_sec),
        (signal.SIGTERM, 1.0),
        (signal.SIGKILL, 1.0),
    )
    for stop_signal, wait_sec in signal_timeouts:
        try:
            os.killpg(pgid, stop_signal)
        except ProcessLookupError:
            pass
        except PermissionError:
            return False, f"permission denied stopping {name}: pgid={pgid}"
        if _wait_for_group_exit(pgid, wait_sec):
            _unlink_registry(registry_path)
            return True, f"stopped stale managed process {name}: pgid={pgid}"

    return False, f"stale managed process survived SIGKILL: {name}, pgid={pgid}"


def _process_group_members(pgid):
    """Return Linux process IDs currently belonging to one process group."""
    members = []
    try:
        proc_entries = os.listdir("/proc")
    except OSError:
        return members
    for entry in proc_entries:
        if not entry.isdigit():
            continue
        stat_path = os.path.join("/proc", entry, "stat")
        try:
            with open(stat_path, "r", encoding="utf-8") as stat_file:
                stat_text = stat_file.read()
            closing_paren = stat_text.rfind(")")
            fields_text = stat_text[closing_paren + 2:]
            fields = fields_text.split()
            process_state = fields[0]
            process_group = int(fields[2])
        except (OSError, ValueError, IndexError):
            continue
        if process_group == pgid and process_state != "Z":
            members.append(int(entry))
    return members


def _process_has_runtime_token(pid, uid, token):
    """Return whether one process carries the expected AFTR ownership token."""
    process_dir = os.path.join("/proc", str(pid))
    try:
        if os.stat(process_dir).st_uid != uid:
            return False
        with open(
            os.path.join(process_dir, "environ"),
            "rb",
        ) as environ_file:
            entries = set(environ_file.read().split(b"\0"))
    except OSError:
        return False
    return (
        b"MDBOT_MANAGED_PROCESS=1" in entries
        and f"MDBOT_RUNTIME_TOKEN={token}".encode() in entries
    )


def _wait_for_group_exit(pgid, timeout_sec):
    """Wait until Linux reports no members in one process group."""
    deadline = time.monotonic() + max(0.0, float(timeout_sec))
    while True:
        if not _process_group_members(pgid):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.05)


def _unlink_registry(registry_path):
    """Remove one stale ownership record if it still exists."""
    try:
        os.unlink(registry_path)
    except FileNotFoundError:
        pass
