# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Filesystem helpers for ``aftr_path_manager`` path and pose storage."""

from __future__ import annotations

import csv
import os

from aftr_path_manager.path_types import RecordedPose


def ensure_parent_dir(file_path: str) -> None:
    """Create a parent directory for a file path when needed."""
    parent_dir = os.path.dirname(file_path)
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)


def save_path_csv(file_path: str, poses: list[RecordedPose]) -> None:
    """Save recorded path points as CSV."""
    ensure_parent_dir(file_path)
    with open(file_path, "w", newline="", encoding="utf-8") as csv_handle:
        writer = csv.writer(csv_handle)
        writer.writerow(["stamp_sec", "frame_id", "x", "y", "yaw"])
        for pose in poses:
            writer.writerow(
                [
                    f"{pose.stamp_sec:.9f}",
                    pose.frame_id,
                    f"{pose.x:.6f}",
                    f"{pose.y:.6f}",
                    f"{pose.yaw:.6f}",
                ]
            )


def load_path_csv(file_path: str) -> tuple[list[RecordedPose], str | None]:
    """Load a CSV path file."""
    if not os.path.exists(file_path):
        return [], None

    poses: list[RecordedPose] = []
    skipped_invalid_row = False
    with open(file_path, "r", newline="", encoding="utf-8") as csv_handle:
        reader = csv.DictReader(csv_handle)
        required_fields = {"stamp_sec", "frame_id", "x", "y", "yaw"}
        if not reader.fieldnames or not required_fields.issubset(reader.fieldnames):
            return [], f"invalid path CSV header: {file_path}"

        for row in reader:
            try:
                poses.append(
                    RecordedPose(
                        stamp_sec=float(row["stamp_sec"]),
                        frame_id=row["frame_id"],
                        x=float(row["x"]),
                        y=float(row["y"]),
                        yaw=float(row["yaw"]),
                    )
                )
            except (KeyError, ValueError):
                skipped_invalid_row = True

    if skipped_invalid_row:
        return poses, f"invalid path CSV row skipped: {file_path}"
    return poses, None


def save_pose_yaml(file_path: str, pose: RecordedPose) -> None:
    """Save the latest pose as a small YAML file."""
    ensure_parent_dir(file_path)
    with open(file_path, "w", encoding="utf-8") as yaml_handle:
        yaml_handle.write(f"frame_id: {pose.frame_id}\n")
        yaml_handle.write(f"stamp_sec: {pose.stamp_sec:.9f}\n")
        yaml_handle.write(f"x: {pose.x:.6f}\n")
        yaml_handle.write(f"y: {pose.y:.6f}\n")
        yaml_handle.write(f"yaw: {pose.yaw:.6f}\n")


def load_pose_yaml(file_path: str) -> tuple[RecordedPose | None, str | None]:
    """Load the last saved pose from YAML."""
    if not os.path.exists(file_path):
        return None, None

    values: dict[str, str] = {}
    with open(file_path, "r", encoding="utf-8") as yaml_handle:
        for line in yaml_handle:
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()

    required_keys = {"frame_id", "stamp_sec", "x", "y", "yaw"}
    if not required_keys.issubset(values):
        return None, f"invalid pose file: {file_path}"

    try:
        pose = RecordedPose(
            stamp_sec=float(values["stamp_sec"]),
            frame_id=values["frame_id"],
            x=float(values["x"]),
            y=float(values["y"]),
            yaw=float(values["yaw"]),
        )
    except ValueError:
        return None, f"invalid numeric value in pose file: {file_path}"

    return pose, None
