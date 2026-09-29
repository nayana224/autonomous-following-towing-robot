# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for the mode-manager single-instance lock."""

import pytest

from aftr_mode_manager.runtime.instance_lock import ModeManagerInstanceLock


def test_second_instance_is_rejected_and_release_allows_reacquire(tmp_path):
    """Only one lock owner may exist, and release must permit restart."""
    lock_path = str(tmp_path / "mode_manager.lock")
    first = ModeManagerInstanceLock(lock_path)
    second = ModeManagerInstanceLock(lock_path)

    first.acquire()
    with pytest.raises(RuntimeError, match="already running"):
        second.acquire()

    first.release()
    second.acquire()
    second.release()
