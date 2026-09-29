# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Process-level single-instance lock for the AFTR mode manager."""

import fcntl
import os


class ModeManagerInstanceLock:
    """Hold an automatically released lock while one mode manager is alive."""

    def __init__(self, path=None):
        """Create the lock descriptor without acquiring it yet."""
        self.path = path or f"/tmp/mdbot_mode_manager_{os.getuid()}.lock"
        self._file = None

    def acquire(self):
        """Acquire lock, raising ``RuntimeError`` if another owner exists."""
        lock_file = open(self.path, "a+", encoding="utf-8")
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            lock_file.close()
            raise RuntimeError(
                "another mode_manager process is already running; "
                f"single-instance lock is held: {self.path}"
            ) from exc

        lock_file.seek(0)
        lock_file.truncate()
        lock_file.write(f"pid={os.getpid()}\n")
        lock_file.flush()
        self._file = lock_file

    def release(self):
        """Release the lock; the kernel also does this after abnormal exit."""
        if self._file is None:
            return
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
        finally:
            self._file.close()
            self._file = None
