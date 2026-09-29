#!/usr/bin/env python3
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

"""Stricter partial-pose filtering for MDBOT fall detection."""

import cv2
import numpy as np
import rclpy

from aftr_fall_detection.fall_detection_node import FallDetectionNode
from aftr_fall_detection.fall_detection_node import PersonTrack
from aftr_fall_detection.pose_geometry import analyze_pose_geometry


class FilteredFallDetectionNode(FallDetectionNode):
    """Reject clipped or low-confidence partial poses before fall scoring."""

    def _declare_parameters(self) -> None:
        super()._declare_parameters()
        self.declare_parameter("critical_keypoint_confidence", 0.50)
        self.declare_parameter("full_torso_angle_threshold_deg", 60.0)
        self.declare_parameter("partial_torso_angle_threshold_deg", 70.0)
        self.declare_parameter("full_candidate_hold_seconds", 1.5)
        self.declare_parameter("partial_candidate_hold_seconds", 2.0)
        self.declare_parameter("full_body_aspect_threshold", 0.80)
        self.declare_parameter("partial_upper_body_aspect_threshold", 1.0)

    def _read_parameters(self) -> None:
        super()._read_parameters()
        self.critical_keypoint_confidence = float(
            self.get_parameter("critical_keypoint_confidence").value
        )
        self.full_torso_angle_threshold_deg = float(
            self.get_parameter("full_torso_angle_threshold_deg").value
        )
        self.partial_torso_angle_threshold_deg = float(
            self.get_parameter("partial_torso_angle_threshold_deg").value
        )
        self.full_candidate_hold_seconds = float(
            self.get_parameter("full_candidate_hold_seconds").value
        )
        self.partial_candidate_hold_seconds = float(
            self.get_parameter("partial_candidate_hold_seconds").value
        )
        self.full_body_aspect_threshold = float(
            self.get_parameter("full_body_aspect_threshold").value
        )
        self.partial_upper_body_aspect_threshold = float(
            self.get_parameter("partial_upper_body_aspect_threshold").value
        )

    def _analyze_people(self, result) -> list[dict]:
        if result.boxes is None or result.keypoints is None:
            return []

        boxes = result.boxes.xyxy.detach().cpu().numpy()
        keypoints = result.keypoints.data.detach().cpu().numpy()
        return [
            self._analyze_person_filtered(
                bbox,
                person_keypoints,
            )
            for bbox, person_keypoints in zip(boxes, keypoints)
        ]

    def _analyze_person_filtered(
        self,
        bbox: np.ndarray,
        keypoints: np.ndarray,
    ) -> dict:
        return analyze_pose_geometry(
            bbox,
            keypoints,
            confidence_threshold=self.critical_keypoint_confidence,
            full_angle_threshold=self.full_torso_angle_threshold_deg,
            partial_angle_threshold=self.partial_torso_angle_threshold_deg,
            full_body_aspect_threshold=self.full_body_aspect_threshold,
            partial_upper_body_aspect_threshold=(
                self.partial_upper_body_aspect_threshold
            ),
        )

    def _update_track_state(self, track: PersonTrack, analysis: dict, now: float) -> None:
        """Apply separate confirmation holds for full and partial pose candidates."""
        if analysis["partial_pose"]:
            if track.partial_started_at is None:
                track.partial_started_at = now
            if now - track.partial_started_at > self.partial_pose_grace_seconds:
                track.candidate_started_at = None
                track.candidate_kind = ""
                track.state = "PARTIAL_POSE"
            return

        track.partial_started_at = None
        if not analysis["fall_candidate"]:
            track.candidate_started_at = None
            track.candidate_kind = ""
            track.state = "NORMAL"
            return

        candidate_kind = analysis["visibility"]
        if track.candidate_kind != candidate_kind:
            track.candidate_started_at = now
            track.candidate_kind = candidate_kind
        elif track.candidate_started_at is None:
            track.candidate_started_at = now

        hold_seconds = (
            self.full_candidate_hold_seconds
            if candidate_kind == "FULL"
            else self.partial_candidate_hold_seconds
        )
        elapsed = now - track.candidate_started_at
        track.state = (
            "FALL_DETECTED" if elapsed >= hold_seconds else "FALL_CANDIDATE"
        )
        if track.state == "FALL_DETECTED":
            self._latch_safety("FALL_DETECTED")

    def _draw_analyses(
        self,
        frame: np.ndarray,
        analyses: list[dict],
        inference_time_ms: float,
    ) -> None:
        super()._draw_analyses(frame, analyses, inference_time_ms)
        for analysis in analyses:
            x1, _y1, _x2, y2 = analysis["bbox"]
            visibility = analysis.get("visibility", "UNKNOWN")
            reason = analysis.get("partial_reason", "")
            if reason:
                diagnostic = f"{visibility}:{reason}"
            else:
                diagnostic = (
                    f"{visibility} "
                    f"B:{analysis.get('body_aspect_ratio', 0.0):.2f} "
                    f"U:{analysis.get('upper_body_aspect_ratio', 0.0):.2f}"
                )
            cv2.putText(
                frame,
                diagnostic,
                (x1, min(y2 + 40, frame.shape[0] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                self._state_color(analysis.get("track_state", "UNKNOWN")),
                1,
                cv2.LINE_AA,
            )


def main(args=None) -> None:
    """Run filtered fall detection and release ROS resources on exit."""
    rclpy.init(args=args)
    node = FilteredFallDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
