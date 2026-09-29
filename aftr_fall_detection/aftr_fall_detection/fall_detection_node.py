#!/usr/bin/env python3
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

import math
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Optional

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, Int32, String
from std_srvs.srv import Trigger
from ultralytics import YOLO

LEFT_SHOULDER = 5
RIGHT_SHOULDER = 6
LEFT_HIP = 11
RIGHT_HIP = 12


@dataclass
class PersonTrack:
    """Small per-person state used for temporal fall confirmation."""

    track_id: int
    bbox: tuple[int, int, int, int]
    last_seen_at: float
    state: str = "NORMAL"
    candidate_started_at: Optional[float] = None
    candidate_kind: str = ""
    partial_started_at: Optional[float] = None


class FallDetectionNode(Node):
    """Latch a safety event when any visible person remains fallen."""

    def __init__(self) -> None:
        super().__init__("fall_detection")
        self._declare_parameters()
        self._read_parameters()

        self.bridge = CvBridge()
        self.frame_lock = Lock()
        self.latest_frame: Optional[np.ndarray] = None
        self.last_image_at = time.monotonic()

        self.device = self._select_device()
        self.model = self._load_model()

        self.tracks: dict[int, PersonTrack] = {}
        self.next_track_id = 1
        self.observation_status = "NO_PERSON"
        self.safety_latched = False
        self.safety_reason = ""
        self.last_log_signature: Optional[tuple[str, bool, str]] = None

        self.status_publisher = self.create_publisher(String, "~/status", 10)
        self.detected_publisher = self.create_publisher(Bool, "~/detected", 10)
        self.person_count_publisher = self.create_publisher(Int32, "~/person_count", 10)
        self.heartbeat_publisher = self.create_publisher(Empty, "~/heartbeat", 10)
        self.reset_service = self.create_service(Trigger, "~/reset", self._reset_callback)

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos,
        )

        self.inference_timer = self.create_timer(1.0 / self.inference_hz, self.run_inference)
        self.watchdog_timer = self.create_timer(0.2, self._watchdog_callback)
        self.heartbeat_timer = self.create_timer(0.5, self._heartbeat_callback)

        self.get_logger().info(f"이미지 토픽: {self.image_topic}")
        self.get_logger().info(f"모델: {self.model_path}")
        self.get_logger().info(f"추론 장치: {self.device}")
        self.get_logger().info(f"추론 주기: {self.inference_hz:.1f} Hz")

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "model_path",
            "/home/mechatukka/inpyo_ws/mdbot_models/fall_detection/yolov8n-pose.pt",
        )
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("keypoint_confidence", 0.35)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("inference_hz", 4.0)
        self.declare_parameter("show_image", True)
        self.declare_parameter("torso_angle_threshold_deg", 55.0)
        self.declare_parameter("box_aspect_ratio_threshold", 0.95)
        self.declare_parameter("torso_height_ratio_threshold", 0.30)
        self.declare_parameter("candidate_hold_seconds", 1.0)
        self.declare_parameter("partial_pose_grace_seconds", 0.5)
        self.declare_parameter("track_lost_grace_seconds", 0.75)
        self.declare_parameter("image_timeout_seconds", 3.0)
        self.declare_parameter("track_match_distance_ratio", 0.35)
        self.declare_parameter("track_match_iou_threshold", 0.10)
        self.declare_parameter("candidate_track_iou_threshold", 0.20)
        self.declare_parameter("track_match_area_ratio_min", 0.45)
        self.declare_parameter("track_match_area_ratio_max", 2.20)
        self.declare_parameter("latch_sensor_fault", False)

    def _read_parameters(self) -> None:
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.requested_device = str(self.get_parameter("device").value)
        self.confidence = float(self.get_parameter("confidence").value)
        self.keypoint_confidence = float(self.get_parameter("keypoint_confidence").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.inference_hz = float(self.get_parameter("inference_hz").value)
        self.show_image = bool(self.get_parameter("show_image").value)
        self.torso_angle_threshold_deg = float(
            self.get_parameter("torso_angle_threshold_deg").value
        )
        self.box_aspect_ratio_threshold = float(
            self.get_parameter("box_aspect_ratio_threshold").value
        )
        self.torso_height_ratio_threshold = float(
            self.get_parameter("torso_height_ratio_threshold").value
        )
        self.candidate_hold_seconds = float(
            self.get_parameter("candidate_hold_seconds").value
        )
        self.partial_pose_grace_seconds = float(
            self.get_parameter("partial_pose_grace_seconds").value
        )
        self.track_lost_grace_seconds = float(
            self.get_parameter("track_lost_grace_seconds").value
        )
        self.image_timeout_seconds = float(
            self.get_parameter("image_timeout_seconds").value
        )
        self.track_match_distance_ratio = float(
            self.get_parameter("track_match_distance_ratio").value
        )
        self.track_match_iou_threshold = float(
            self.get_parameter("track_match_iou_threshold").value
        )
        self.candidate_track_iou_threshold = float(
            self.get_parameter("candidate_track_iou_threshold").value
        )
        self.track_match_area_ratio_min = float(
            self.get_parameter("track_match_area_ratio_min").value
        )
        self.track_match_area_ratio_max = float(
            self.get_parameter("track_match_area_ratio_max").value
        )
        self.latch_sensor_fault = bool(self.get_parameter("latch_sensor_fault").value)

        if self.inference_hz <= 0.0:
            raise ValueError("inference_hz는 0보다 커야 합니다.")

    def _select_device(self) -> str:
        """Use the requested CUDA device only when available; otherwise use CPU."""
        if self.requested_device.startswith("cuda") and torch.cuda.is_available():
            return self.requested_device
        if self.requested_device.startswith("cuda"):
            self.get_logger().warning("CUDA를 사용할 수 없어 CPU로 전환합니다.")
        return "cpu"

    def _load_model(self) -> YOLO:
        """Load the configured YOLO model or fail when its file is missing."""
        model_path = Path(self.model_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"모델을 찾을 수 없습니다: {model_path}")
        self.get_logger().info("YOLO Pose 모델을 불러오는 중입니다.")
        model = YOLO(str(model_path))
        self.get_logger().info("YOLO Pose 모델 로드 완료")
        return model

    def image_callback(self, message: Image) -> None:
        """Store the latest camera frame for timer-driven inference."""
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"이미지 변환 실패: {exc}")
            return

        with self.frame_lock:
            self.latest_frame = frame.copy()
            self.last_image_at = time.monotonic()

    def _take_latest_frame(self) -> Optional[np.ndarray]:
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            frame = self.latest_frame
            self.latest_frame = None
        return frame

    def run_inference(self) -> None:
        """Analyze the latest frame and publish fall and safety state."""
        frame = self._take_latest_frame()
        if frame is None:
            return

        try:
            results = self.model.predict(
                source=frame,
                device=self.device,
                imgsz=self.image_size,
                conf=self.confidence,
                verbose=False,
            )
        except Exception as exc:
            self.observation_status = "INFERENCE_ERROR"
            self._handle_sensor_fault("INFERENCE_ERROR")
            self.get_logger().error(f"YOLO 추론 실패: {exc}")
            self._publish_state(0)
            return

        result = results[0] if results else None
        analyses = [] if result is None else self._analyze_people(result)
        self._update_tracks(analyses, time.monotonic(), frame.shape)
        self._publish_state(len(analyses))

        if self.show_image and result is not None:
            annotated_frame = result.plot()
            self._draw_analyses(
                annotated_frame,
                analyses,
                result.speed.get("inference", 0.0),
            )
            cv2.imshow("MDBOT Fall Detection", annotated_frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                rclpy.shutdown()

    def _analyze_people(self, result) -> list[dict]:
        if result.boxes is None or result.keypoints is None:
            return []

        boxes = result.boxes.xyxy.detach().cpu().numpy()
        keypoints = result.keypoints.data.detach().cpu().numpy()
        return [
            self._analyze_person(bbox, person_keypoints)
            for bbox, person_keypoints in zip(boxes, keypoints)
        ]

    def _analyze_person(self, bbox: np.ndarray, keypoints: np.ndarray) -> dict:
        x1, y1, x2, y2 = bbox.astype(int)
        box_width = max(float(x2 - x1), 1.0)
        box_height = max(float(y2 - y1), 1.0)
        aspect_ratio = box_width / box_height

        required_points = [
            keypoints[LEFT_SHOULDER],
            keypoints[RIGHT_SHOULDER],
            keypoints[LEFT_HIP],
            keypoints[RIGHT_HIP],
        ]
        if not all(self._keypoint_valid(point) for point in required_points):
            return {
                "bbox": (x1, y1, x2, y2),
                "partial_pose": True,
                "fall_candidate": False,
                "torso_angle_deg": 0.0,
                "box_aspect_ratio": aspect_ratio,
                "torso_height_ratio": 1.0,
                "evidence_count": 0,
            }

        shoulder_center = self._center_point(required_points[0], required_points[1])
        hip_center = self._center_point(required_points[2], required_points[3])
        torso_dx = hip_center[0] - shoulder_center[0]
        torso_dy = hip_center[1] - shoulder_center[1]
        torso_angle_deg = math.degrees(
            math.atan2(abs(torso_dx), max(abs(torso_dy), 1e-6))
        )
        torso_height_ratio = abs(torso_dy) / box_height
        evidence_count = sum(
            [
                torso_angle_deg >= self.torso_angle_threshold_deg,
                aspect_ratio >= self.box_aspect_ratio_threshold,
                torso_height_ratio <= self.torso_height_ratio_threshold,
            ]
        )

        return {
            "bbox": (x1, y1, x2, y2),
            "partial_pose": False,
            "fall_candidate": evidence_count >= 2,
            "torso_angle_deg": torso_angle_deg,
            "box_aspect_ratio": aspect_ratio,
            "torso_height_ratio": torso_height_ratio,
            "evidence_count": evidence_count,
        }

    def _update_tracks(
        self,
        analyses: list[dict],
        now: float,
        frame_shape: tuple[int, ...],
    ) -> None:
        assignments = self._match_tracks(analyses, frame_shape)
        seen_track_ids: set[int] = set()

        for analysis_index, analysis in enumerate(analyses):
            track_id = assignments.get(analysis_index)
            if track_id is None:
                track_id = self.next_track_id
                self.next_track_id += 1
                self.tracks[track_id] = PersonTrack(
                    track_id=track_id,
                    bbox=analysis["bbox"],
                    last_seen_at=now,
                )

            track = self.tracks[track_id]
            track.bbox = analysis["bbox"]
            track.last_seen_at = now
            self._update_track_state(track, analysis, now)
            analysis["track_id"] = track_id
            analysis["track_state"] = track.state
            analysis["candidate_seconds"] = (
                0.0
                if track.candidate_started_at is None
                else now - track.candidate_started_at
            )
            seen_track_ids.add(track_id)

        self._expire_tracks(seen_track_ids, now)
        self._update_observation_status(analyses)

    def _match_tracks(
        self,
        analyses: list[dict],
        frame_shape: tuple[int, ...],
    ) -> dict[int, int]:
        """Associate people without transferring a fall timer to a nearby person."""
        del frame_shape  # Track matching uses box-relative distances.
        assignments: dict[int, int] = {}
        candidates: list[tuple[float, int, int]] = []

        for analysis_index, analysis in enumerate(analyses):
            detection_bbox = analysis["bbox"]
            detection_area = self._bbox_area(detection_bbox)
            detection_center = self._bbox_center(detection_bbox)
            detection_diagonal = self._bbox_diagonal(detection_bbox)

            for track_id, track in self.tracks.items():
                track_area = self._bbox_area(track.bbox)
                area_ratio = detection_area / max(track_area, 1.0)
                area_in_range = (
                    area_ratio >= self.track_match_area_ratio_min
                    and area_ratio <= self.track_match_area_ratio_max
                )
                if not area_in_range:
                    continue

                center_distance = math.dist(
                    detection_center,
                    self._bbox_center(track.bbox),
                )
                distance_scale = max(
                    detection_diagonal,
                    self._bbox_diagonal(track.bbox),
                    1.0,
                )
                normalized_distance = center_distance / distance_scale
                iou = self._bbox_iou(detection_bbox, track.bbox)

                # Once a fall candidate is accumulating time, only a clearly
                # overlapping detection may inherit it. This prevents a nearby
                # person from receiving the previous person's candidate timer.
                if (
                    track.candidate_started_at is not None
                    and iou < self.candidate_track_iou_threshold
                ):
                    continue

                # A close center alone is unsafe when two people overlap. Require
                # either box overlap or only a small, box-relative displacement.
                if (
                    iou < self.track_match_iou_threshold
                    and normalized_distance > self.track_match_distance_ratio
                ):
                    continue

                proximity = max(0.0, 1.0 - normalized_distance)
                area_similarity = min(area_ratio, 1.0 / area_ratio)
                score = 0.60 * iou + 0.25 * proximity + 0.15 * area_similarity
                candidates.append((score, analysis_index, track_id))

        used_analyses: set[int] = set()
        used_tracks: set[int] = set()
        for _score, analysis_index, track_id in sorted(candidates, reverse=True):
            if analysis_index in used_analyses or track_id in used_tracks:
                continue
            assignments[analysis_index] = track_id
            used_analyses.add(analysis_index)
            used_tracks.add(track_id)
        return assignments

    def _update_track_state(self, track: PersonTrack, analysis: dict, now: float) -> None:
        """Latch a fall only after a candidate persists for the hold interval."""
        if analysis["partial_pose"]:
            if track.partial_started_at is None:
                track.partial_started_at = now
            if now - track.partial_started_at > self.partial_pose_grace_seconds:
                track.state = "PARTIAL_POSE"
                track.candidate_started_at = None
                track.candidate_kind = ""
            return

        track.partial_started_at = None
        if analysis["fall_candidate"]:
            if track.candidate_started_at is None:
                track.candidate_started_at = now
            elapsed = now - track.candidate_started_at
            track.state = (
                "FALL_DETECTED"
                if elapsed >= self.candidate_hold_seconds
                else "FALL_CANDIDATE"
            )
            if track.state == "FALL_DETECTED":
                self._latch_safety("FALL_DETECTED")
            return

        track.candidate_started_at = None
        track.candidate_kind = ""
        track.state = "NORMAL"

    def _expire_tracks(self, seen_track_ids: set[int], now: float) -> None:
        expired_track_ids = [
            track_id
            for track_id, track in self.tracks.items()
            if track_id not in seen_track_ids
            and now - track.last_seen_at > self.track_lost_grace_seconds
        ]
        for track_id in expired_track_ids:
            del self.tracks[track_id]

    def _update_observation_status(self, analyses: list[dict]) -> None:
        if not analyses:
            self.observation_status = "NO_PERSON"
        elif any(track.state == "FALL_DETECTED" for track in self.tracks.values()):
            self.observation_status = "FALL_DETECTED"
        elif any(track.state == "FALL_CANDIDATE" for track in self.tracks.values()):
            self.observation_status = "FALL_CANDIDATE"
        elif all(analysis["partial_pose"] for analysis in analyses):
            self.observation_status = "PARTIAL_POSE"
        else:
            self.observation_status = "NORMAL"

    def _watchdog_callback(self) -> None:
        """Report a sensor fault when camera frames exceed the timeout."""
        if self._image_stream_is_fresh():
            return
        if self.observation_status != "STALE_IMAGE":
            self.observation_status = "STALE_IMAGE"
            self._handle_sensor_fault("STALE_IMAGE")
            self._publish_state(0)

    def _image_stream_is_fresh(self) -> bool:
        """Return whether the safety detector is receiving current images."""
        return time.monotonic() - self.last_image_at <= self.image_timeout_seconds

    def _handle_sensor_fault(self, reason: str) -> None:
        if self.latch_sensor_fault:
            self._latch_safety(reason)

    def _latch_safety(self, reason: str) -> None:
        if self.safety_latched:
            return
        self.safety_latched = True
        self.safety_reason = reason
        self.get_logger().error(f"안전 정지 래치 활성화: {reason}")

    def _reset_callback(self, _request: Trigger.Request, response: Trigger.Response):
        """Clear the safety latch and pending fall timers on explicit reset."""
        self.safety_latched = False
        self.safety_reason = ""
        for track in self.tracks.values():
            track.candidate_started_at = None
            track.candidate_kind = ""
            if track.state in ("FALL_CANDIDATE", "FALL_DETECTED"):
                track.state = "NORMAL"
        response.success = True
        response.message = "쓰러짐 안전 래치를 해제했습니다."
        self.get_logger().warning(response.message)
        self._publish_state(len(self.tracks))
        return response

    def _heartbeat_callback(self) -> None:
        """Withhold heartbeat when the camera stream is stale."""
        # A live process without camera frames is not a healthy safety detector.
        # Withholding the heartbeat makes the mode manager stop active motion;
        # heartbeat recovery never resumes that motion automatically.
        if self._image_stream_is_fresh():
            self.heartbeat_publisher.publish(Empty())

    def _publish_state(self, person_count: int) -> None:
        status_message = String()
        status_message.data = self.observation_status
        detected_message = Bool()
        detected_message.data = self.safety_latched
        person_count_message = Int32()
        person_count_message.data = person_count

        self.status_publisher.publish(status_message)
        self.detected_publisher.publish(detected_message)
        self.person_count_publisher.publish(person_count_message)

        signature = (self.observation_status, self.safety_latched, self.safety_reason)
        if signature != self.last_log_signature:
            self.get_logger().info(
                f"관측 상태: {self.observation_status}, "
                f"안전 래치: {self.safety_latched}, "
                f"사유: {self.safety_reason or '-'}"
            )
            self.last_log_signature = signature

    def _draw_analyses(
        self,
        frame: np.ndarray,
        analyses: list[dict],
        inference_time_ms: float,
    ) -> None:
        for analysis in analyses:
            x1, y1, x2, y2 = analysis["bbox"]
            state = analysis.get("track_state", "UNKNOWN")
            color = self._state_color(state)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame,
                f"P{analysis.get('track_id', '?')} {state}",
                (x1, max(22, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                color,
                2,
                cv2.LINE_AA,
            )
            if not analysis["partial_pose"]:
                metrics = (
                    f"A:{analysis['torso_angle_deg']:.0f} "
                    f"R:{analysis['box_aspect_ratio']:.2f} "
                    f"T:{analysis['torso_height_ratio']:.2f} "
                    f"E:{analysis['evidence_count']}/"
                    f"{analysis.get('evidence_total', 3)} "
                    f"S:{analysis.get('candidate_seconds', 0.0):.1f}s"
                )
                cv2.putText(
                    frame,
                    metrics,
                    (x1, min(y2 + 22, frame.shape[0] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.43,
                    color,
                    1,
                    cv2.LINE_AA,
                )

        global_text = (
            f"{self.observation_status}  "
            f"LATCH:{'ON' if self.safety_latched else 'OFF'}  "
            f"Persons:{len(analyses)}  "
            f"Inference:{inference_time_ms:.1f}ms"
        )
        global_state = "FALL_DETECTED" if self.safety_latched else self.observation_status
        cv2.putText(
            frame,
            global_text,
            (15, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            self._state_color(global_state),
            2,
            cv2.LINE_AA,
        )

    def _keypoint_valid(self, point: np.ndarray) -> bool:
        return len(point) >= 3 and float(point[2]) >= self.keypoint_confidence

    @staticmethod
    def _center_point(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
        return (
            float(first[0] + second[0]) / 2.0,
            float(first[1] + second[1]) / 2.0,
        )

    @staticmethod
    def _bbox_center(bbox: tuple[int, int, int, int]) -> tuple[float, float]:
        x1, y1, x2, y2 = bbox
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    @staticmethod
    def _bbox_area(bbox: tuple[int, int, int, int]) -> float:
        x1, y1, x2, y2 = bbox
        return max(float(x2 - x1), 0.0) * max(float(y2 - y1), 0.0)

    @staticmethod
    def _bbox_diagonal(bbox: tuple[int, int, int, int]) -> float:
        x1, y1, x2, y2 = bbox
        return math.hypot(x2 - x1, y2 - y1)

    @staticmethod
    def _bbox_iou(
        first: tuple[int, int, int, int],
        second: tuple[int, int, int, int],
    ) -> float:
        left = max(first[0], second[0])
        top = max(first[1], second[1])
        right = min(first[2], second[2])
        bottom = min(first[3], second[3])
        intersection = max(float(right - left), 0.0) * max(
            float(bottom - top), 0.0
        )
        first_area = FallDetectionNode._bbox_area(first)
        second_area = FallDetectionNode._bbox_area(second)
        union = first_area + second_area - intersection
        return intersection / union if union > 0.0 else 0.0

    @staticmethod
    def _state_color(state: str) -> tuple[int, int, int]:
        if state in ("FALL_DETECTED", "STALE_IMAGE", "INFERENCE_ERROR"):
            return (0, 0, 255)
        if state in ("FALL_CANDIDATE", "PARTIAL_POSE"):
            return (0, 165, 255)
        if state == "NORMAL":
            return (0, 255, 0)
        return (200, 200, 200)

    def destroy_node(self) -> bool:
        """Close OpenCV windows before destroying the ROS node."""
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    """Run fall detection and release ROS resources on exit."""
    rclpy.init(args=args)
    node = FallDetectionNode()
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
