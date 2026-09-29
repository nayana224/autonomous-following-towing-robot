# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Shared data structures for ``aftr_path_manager``."""

from dataclasses import dataclass


@dataclass
class RecordedPose:
    """A compact 2D pose stored by the path manager."""

    stamp_sec: float
    frame_id: str
    x: float
    y: float
    yaw: float
