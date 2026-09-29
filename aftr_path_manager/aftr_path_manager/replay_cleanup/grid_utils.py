# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Occupancy-grid helpers used by replay-path cleanup."""

from __future__ import annotations

from dataclasses import dataclass
import math

from nav_msgs.msg import OccupancyGrid

from aftr_path_manager.path_types import RecordedPose


@dataclass
class OccupancyGridSnapshot:
    """A lightweight occupancy grid view used during path replay cleanup."""

    resolution_m: float
    width: int
    height: int
    origin_x_m: float
    origin_y_m: float
    cells: list[int]
    unknown_is_occupied: bool


def build_grid_snapshot(
    map_msg: OccupancyGrid,
    unknown_is_occupied: bool,
) -> OccupancyGridSnapshot | None:
    """Build a lightweight grid snapshot from a ROS occupancy grid message."""
    resolution_m = float(map_msg.info.resolution)
    width = int(map_msg.info.width)
    height = int(map_msg.info.height)
    if resolution_m <= 0.0 or width <= 0 or height <= 0:
        return None

    origin = map_msg.info.origin.position
    return OccupancyGridSnapshot(
        resolution_m=resolution_m,
        width=width,
        height=height,
        origin_x_m=float(origin.x),
        origin_y_m=float(origin.y),
        cells=list(map_msg.data),
        unknown_is_occupied=unknown_is_occupied,
    )


def world_to_grid(
    x_m: float,
    y_m: float,
    grid: OccupancyGridSnapshot,
) -> tuple[int, int] | None:
    """Convert world coordinates to occupancy-grid indices."""
    x_index = int(math.floor((x_m - grid.origin_x_m) / grid.resolution_m))
    y_index = int(math.floor((y_m - grid.origin_y_m) / grid.resolution_m))
    if not is_cell_inside_grid(x_index, y_index, grid):
        return None
    return x_index, y_index


def is_cell_inside_grid(
    x_index: int,
    y_index: int,
    grid: OccupancyGridSnapshot,
) -> bool:
    """Return True when the occupancy-grid cell indices are valid."""
    if x_index < 0 or y_index < 0:
        return False
    if x_index >= grid.width or y_index >= grid.height:
        return False
    return True


def is_cell_occupied(
    x_index: int,
    y_index: int,
    grid: OccupancyGridSnapshot,
) -> bool:
    """Return True when a cell should be treated as blocked."""
    linear_index = y_index * grid.width + x_index
    cell_value = grid.cells[linear_index]
    if cell_value < 0:
        return grid.unknown_is_occupied
    return cell_value >= 50


def is_pose_safe(
    pose: RecordedPose,
    grid: OccupancyGridSnapshot,
    clearance_m: float,
) -> bool:
    """Return True when all cells inside the requested clearance are free."""
    center_indices = world_to_grid(pose.x, pose.y, grid)
    if center_indices is None:
        return False

    radius_in_cells = max(0, int(math.ceil(clearance_m / grid.resolution_m)))
    center_x_index, center_y_index = center_indices

    for y_offset in range(-radius_in_cells, radius_in_cells + 1):
        for x_offset in range(-radius_in_cells, radius_in_cells + 1):
            sample_x_index = center_x_index + x_offset
            sample_y_index = center_y_index + y_offset
            if not is_cell_inside_grid(sample_x_index, sample_y_index, grid):
                return False
            if math.hypot(
                x_offset * grid.resolution_m,
                y_offset * grid.resolution_m,
            ) > clearance_m:
                continue
            if is_cell_occupied(sample_x_index, sample_y_index, grid):
                return False
    return True


def measure_directional_clearance(
    pose: RecordedPose,
    yaw: float,
    grid: OccupancyGridSnapshot,
    max_search_distance_m: float,
) -> float:
    """Measure free distance from one pose along a single heading direction."""
    step_distance_m = max(grid.resolution_m * 0.5, 0.02)
    travelled_distance_m = 0.0

    while travelled_distance_m <= max_search_distance_m:
        sample_x_m = pose.x + math.cos(yaw) * travelled_distance_m
        sample_y_m = pose.y + math.sin(yaw) * travelled_distance_m
        sample_indices = world_to_grid(sample_x_m, sample_y_m, grid)
        if sample_indices is None:
            return max(0.0, travelled_distance_m - step_distance_m)

        sample_x_index, sample_y_index = sample_indices
        if is_cell_occupied(sample_x_index, sample_y_index, grid):
            return max(0.0, travelled_distance_m - step_distance_m)

        travelled_distance_m += step_distance_m

    return max_search_distance_m
