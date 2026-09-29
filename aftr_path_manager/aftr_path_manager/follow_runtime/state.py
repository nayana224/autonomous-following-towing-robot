# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Helpers for saved-path FollowPath runtime state."""

from __future__ import annotations

from dataclasses import dataclass
import math

from aftr_path_manager.path_geometry import nearest_pose_index
from aftr_path_manager.path_types import RecordedPose


@dataclass
class FollowPathRuntimeState:
    """Store local runtime state for one saved-path replay session."""

    active: bool = False
    goal_handle: object | None = None
    cancel_requested: bool = False
    retry_count: int = 0
    reverse: bool = False
    poses: list[RecordedPose] | None = None
    retry_timer: object | None = None
    last_event: str = "idle"

    def __post_init__(self) -> None:
        """Normalize mutable defaults after dataclass initialization."""
        if self.poses is None:
            self.poses = []

    def reset(self) -> None:
        """Reset saved-path replay state except for the last event string."""
        self.active = False
        self.goal_handle = None
        self.cancel_requested = False
        self.retry_count = 0
        self.reverse = False
        self.poses = []
        self.retry_timer = None


def compute_retry_delay_seconds(
    retry_count: int,
    base_delay_s: float,
    max_delay_s: float,
) -> float:
    """Return the bounded retry delay used after a FollowPath failure."""
    safe_base_delay_s = max(0.1, float(base_delay_s))
    safe_max_delay_s = max(safe_base_delay_s, float(max_delay_s))
    scaled_delay_s = safe_base_delay_s * max(1, min(int(retry_count), 4))
    return min(safe_max_delay_s, scaled_delay_s)


def build_remaining_follow_poses(
    active_poses: list[RecordedPose],
    current_pose: RecordedPose | None,
    global_frame: str,
    lookback_points: int,
) -> list[RecordedPose]:
    """Return the remaining replay path starting near the current robot pose."""
    if not active_poses:
        return []

    if current_pose is None:
        return list(active_poses)

    closest_index = find_closest_pose_index(active_poses, current_pose)
    start_index = max(0, closest_index - max(0, int(lookback_points)))
    remaining = list(active_poses[start_index:])
    remaining[0] = RecordedPose(
        stamp_sec=current_pose.stamp_sec,
        frame_id=global_frame,
        x=current_pose.x,
        y=current_pose.y,
        yaw=current_pose.yaw,
    )
    return remaining


def estimate_remaining_follow_distance(
    active_poses: list[RecordedPose],
    current_pose: RecordedPose | None,
) -> float:
    """Estimate the remaining distance from the robot to the final saved pose."""
    if not active_poses or current_pose is None:
        return 0.0

    start_index = nearest_pose_index(active_poses, current_pose)
    remaining = [current_pose, *active_poses[start_index:]]
    distance = 0.0
    for start_pose, end_pose in zip(remaining, remaining[1:]):
        distance += math.hypot(end_pose.x - start_pose.x, end_pose.y - start_pose.y)
    return distance


def find_closest_pose_index(
    active_poses: list[RecordedPose],
    current_pose: RecordedPose,
) -> int:
    """Return the index of the stored pose closest to the current robot pose."""
    closest_index = 0
    closest_distance = float("inf")

    for index, stored_pose in enumerate(active_poses):
        distance = math.hypot(
            stored_pose.x - current_pose.x,
            stored_pose.y - current_pose.y,
        )
        if distance < closest_distance:
            closest_distance = distance
            closest_index = index

    return closest_index
