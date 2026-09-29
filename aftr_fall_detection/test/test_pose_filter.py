# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Unit tests for fall-pose geometry without loading ROS or YOLO."""

import numpy as np

from aftr_fall_detection.pose_geometry import analyze_pose_geometry


def _keypoints(points: dict[int, tuple[float, float]]) -> np.ndarray:
    keypoints = np.zeros((17, 3), dtype=float)
    for index, (x, y) in points.items():
        keypoints[index] = (x, y, 0.95)
    return keypoints


def _analyze(
    bbox: tuple[int, int, int, int],
    points: dict[int, tuple[float, float]],
) -> dict:
    return analyze_pose_geometry(
        np.array(bbox),
        _keypoints(points),
        confidence_threshold=0.50,
        full_angle_threshold=60.0,
        partial_angle_threshold=70.0,
        full_body_aspect_threshold=0.80,
        partial_upper_body_aspect_threshold=1.0,
    )


def test_full_lying_pose_is_candidate():
    analysis = _analyze(
        (20, 60, 240, 150),
        {
            0: (35, 100),
            5: (75, 95),
            6: (75, 105),
            11: (135, 100),
            12: (140, 110),
            13: (190, 100),
            14: (195, 115),
        },
    )

    assert analysis["visibility"] == "FULL"
    assert analysis["fall_candidate"] is True


def test_crouching_pose_with_horizontal_torso_is_rejected():
    analysis = _analyze(
        (60, 35, 210, 230),
        {
            0: (70, 55),
            5: (100, 70),
            6: (105, 78),
            11: (165, 75),
            12: (170, 85),
            13: (145, 150),
            14: (175, 155),
            15: (130, 215),
            16: (190, 215),
        },
    )

    assert analysis["torso_angle_deg"] >= 60.0
    assert analysis["body_aspect_ratio"] < 0.80
    assert analysis["fall_candidate"] is False


def test_partial_lying_upper_body_remains_candidate():
    analysis = _analyze(
        (20, 65, 170, 135),
        {
            0: (30, 100),
            5: (65, 95),
            6: (65, 105),
            11: (130, 100),
            12: (135, 108),
        },
    )

    assert analysis["visibility"] == "PARTIAL"
    assert analysis["fall_candidate"] is True


def test_legs_only_are_ambiguous_and_never_candidate():
    analysis = _analyze(
        (100, 200, 240, 450),
        {
            13: (140, 270),
            14: (190, 275),
            15: (130, 420),
            16: (205, 420),
        },
    )

    assert analysis["visibility"] == "AMBIGUOUS"
    assert analysis["fall_candidate"] is False
