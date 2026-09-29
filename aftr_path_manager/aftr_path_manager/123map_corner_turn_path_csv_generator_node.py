#!/usr/bin/env python3
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

import csv
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


NODE_VERSION = "wide_turn_v2_2026-07-24"


class MapCornerTurnPathGeneratorNode(Node):
    """Build a wide, collision-checked turn around matched map corners from safe_path.csv."""

    def __init__(self) -> None:
        super().__init__("map_corner_turn_path_csv_generator_node")
        self.get_logger().info(f"실행 코드 버전             : {NODE_VERSION}")

        # 1. CSV file / optional ROS publish parameters
        self.declare_parameter("input_csv_path", "/data/paths/safe_path.csv")
        self.declare_parameter("output_csv_path", "/data/paths/corner_turn_path.csv")
        self.declare_parameter("output_frame_id", "map")

        self.declare_parameter("publish_output_path", True)
        self.declare_parameter("output_path_topic", "/corner_turn_path")

        # 2. Input path preprocessing
        self.declare_parameter("min_point_gap", 0.02)
        self.declare_parameter("working_resample_interval", 0.05)
        self.declare_parameter("output_sample_interval", 0.05)

        # 3. Corner detection
        self.declare_parameter("corner_angle_threshold_deg", 25.0)
        self.declare_parameter("corner_max_angle_deg", 150.0)
        self.declare_parameter("corner_lookahead_distance", 0.80)
        self.declare_parameter("corner_min_separation", 1.60)
        self.declare_parameter("preserve_start_distance", 1.00)
        self.declare_parameter("preserve_goal_distance", 1.00)

        self.declare_parameter("use_map_corner_detection", True)
        self.declare_parameter("fallback_to_path_corner_detection", False)
        self.declare_parameter("map_corner_approx_epsilon", 0.12)
        self.declare_parameter("map_corner_min_angle_deg", 55.0)
        self.declare_parameter("map_corner_max_angle_deg", 125.0)
        self.declare_parameter("map_corner_path_max_distance", 1.30)
        self.declare_parameter("map_corner_merge_distance", 0.30)
        self.declare_parameter("map_corner_min_component_area_m2", 0.50)
        self.declare_parameter("map_corner_inside_margin", 0.02)

        # 4. Pre-shift + wide-turn + merge-back geometry
        # Preserve parameter names for compatibility with existing YAML files.
        self.declare_parameter("corner_turn_radius", 0.80)  # legacy compatibility

        # Limit the outward offset in both corridors.
        self.declare_parameter("approach_outer_shift", 0.45)

        # Set the distance available for the pre-shift.
        self.declare_parameter("approach_shift_length", 1.20)

        # Set the wide-turn entry and exit distance from the corner.
        self.declare_parameter("turn_entry_distance", 1.00)
        self.declare_parameter("turn_exit_distance", 1.00)

        # Set the offset hold and merge-back distances.
        self.declare_parameter("exit_hold_length", 0.50)
        self.declare_parameter("merge_back_length", 1.10)

        # Set the longitudinal Bezier handle ratio for shift and merge.
        self.declare_parameter("shift_handle_ratio", 0.45)

        # Larger handles increase the wide-turn radius.
        self.declare_parameter("turn_handle_ratio", 0.70)

        # Extend the turn handles to widen the cornering path.
        self.declare_parameter("corner_outer_offset", 0.30)

        # Reduce shift and turn amplitudes in stages when clearance fails.
        self.declare_parameter("reduce_outer_offset_on_collision", True)
        self.declare_parameter("outer_offset_reduction_step", 0.05)

        # 5. Map-based safety check
        self.declare_parameter("use_map_safety_check", True)
        self.declare_parameter("map_yaml_path", "/data/maps/mdbot_map.yaml")
        self.declare_parameter("robot_radius", 0.35)
        self.declare_parameter("safety_margin", 0.15)
        self.declare_parameter("treat_unknown_as_obstacle", True)
        self.declare_parameter("closing_radius", 0.15)
        self.declare_parameter("safety_check_step", 0.02)

        # Remove only small isolated obstacle remnants near the input path; do not alter the
        # source PGM.
        self.declare_parameter("use_path_corridor_cleanup", True)
        self.declare_parameter("path_corridor_radius", 0.50)
        self.declare_parameter("dynamic_component_max_area_m2", 0.80)
        self.declare_parameter("dynamic_component_max_extent_m", 1.50)
        self.declare_parameter("dynamic_component_max_aspect_ratio", 4.0)

        # Fall back to the original safe_path segment when a curve fails safety checks.
        self.declare_parameter("fallback_to_original_on_unsafe", True)

        # Read parameters
        self.input_csv_path = str(self.get_parameter("input_csv_path").value)
        self.output_csv_path = str(self.get_parameter("output_csv_path").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.publish_output_path = bool(
            self.get_parameter("publish_output_path").value
        )
        self.output_path_topic = str(self.get_parameter("output_path_topic").value)

        self.min_point_gap = float(self.get_parameter("min_point_gap").value)
        self.working_resample_interval = float(
            self.get_parameter("working_resample_interval").value
        )
        self.output_sample_interval = float(
            self.get_parameter("output_sample_interval").value
        )

        self.corner_angle_threshold_deg = float(
            self.get_parameter("corner_angle_threshold_deg").value
        )
        self.corner_max_angle_deg = float(
            self.get_parameter("corner_max_angle_deg").value
        )
        self.corner_lookahead_distance = float(
            self.get_parameter("corner_lookahead_distance").value
        )
        self.corner_min_separation = float(
            self.get_parameter("corner_min_separation").value
        )
        self.preserve_start_distance = float(
            self.get_parameter("preserve_start_distance").value
        )
        self.preserve_goal_distance = float(
            self.get_parameter("preserve_goal_distance").value
        )

        self.use_map_corner_detection = bool(
            self.get_parameter("use_map_corner_detection").value
        )
        self.fallback_to_path_corner_detection = bool(
            self.get_parameter("fallback_to_path_corner_detection").value
        )
        self.map_corner_approx_epsilon = float(
            self.get_parameter("map_corner_approx_epsilon").value
        )
        self.map_corner_min_angle_deg = float(
            self.get_parameter("map_corner_min_angle_deg").value
        )
        self.map_corner_max_angle_deg = float(
            self.get_parameter("map_corner_max_angle_deg").value
        )
        self.map_corner_path_max_distance = float(
            self.get_parameter("map_corner_path_max_distance").value
        )
        self.map_corner_merge_distance = float(
            self.get_parameter("map_corner_merge_distance").value
        )
        self.map_corner_min_component_area_m2 = float(
            self.get_parameter("map_corner_min_component_area_m2").value
        )
        self.map_corner_inside_margin = float(
            self.get_parameter("map_corner_inside_margin").value
        )

        self.corner_turn_radius = float(
            self.get_parameter("corner_turn_radius").value
        )
        self.approach_outer_shift = float(
            self.get_parameter("approach_outer_shift").value
        )
        self.approach_shift_length = float(
            self.get_parameter("approach_shift_length").value
        )
        self.turn_entry_distance = float(
            self.get_parameter("turn_entry_distance").value
        )
        self.turn_exit_distance = float(
            self.get_parameter("turn_exit_distance").value
        )
        self.exit_hold_length = float(
            self.get_parameter("exit_hold_length").value
        )
        self.merge_back_length = float(
            self.get_parameter("merge_back_length").value
        )
        self.shift_handle_ratio = float(
            self.get_parameter("shift_handle_ratio").value
        )
        self.turn_handle_ratio = float(
            self.get_parameter("turn_handle_ratio").value
        )
        self.corner_outer_offset = float(
            self.get_parameter("corner_outer_offset").value
        )
        self.reduce_outer_offset_on_collision = bool(
            self.get_parameter("reduce_outer_offset_on_collision").value
        )
        self.outer_offset_reduction_step = float(
            self.get_parameter("outer_offset_reduction_step").value
        )

        self.use_map_safety_check = bool(
            self.get_parameter("use_map_safety_check").value
        )
        self.map_yaml_path = str(self.get_parameter("map_yaml_path").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.safety_margin = float(self.get_parameter("safety_margin").value)
        self.min_clearance = self.robot_radius + self.safety_margin
        self.treat_unknown_as_obstacle = bool(
            self.get_parameter("treat_unknown_as_obstacle").value
        )
        self.closing_radius = float(self.get_parameter("closing_radius").value)
        self.safety_check_step = float(self.get_parameter("safety_check_step").value)
        self.use_path_corridor_cleanup = bool(
            self.get_parameter("use_path_corridor_cleanup").value
        )
        self.path_corridor_radius = float(
            self.get_parameter("path_corridor_radius").value
        )
        self.dynamic_component_max_area_m2 = float(
            self.get_parameter("dynamic_component_max_area_m2").value
        )
        self.dynamic_component_max_extent_m = float(
            self.get_parameter("dynamic_component_max_extent_m").value
        )
        self.dynamic_component_max_aspect_ratio = float(
            self.get_parameter("dynamic_component_max_aspect_ratio").value
        )
        self.fallback_to_original_on_unsafe = bool(
            self.get_parameter("fallback_to_original_on_unsafe").value
        )

        self._validate_parameters()

        self.map_ready = False
        self.map_corners_world = np.empty((0, 2), dtype=np.float64)
        self.last_output_xy: Optional[np.ndarray] = None

        if self.use_map_safety_check or self.use_map_corner_detection:
            self.load_map_from_yaml(self.map_yaml_path)

        self.path_pub = None
        if self.publish_output_path:
            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.path_pub = self.create_publisher(
                Path, self.output_path_topic, qos
            )

        self.get_logger().info("======= Map Corner Turn CSV Generator =======")
        self.get_logger().info(f"입력 CSV                   : {self.input_csv_path}")
        self.get_logger().info(f"출력 CSV                   : {self.output_csv_path}")
        self.get_logger().info(f"결과 토픽 발행             : {self.publish_output_path}")
        if self.publish_output_path:
            self.get_logger().info(f"출력 topic                 : {self.output_path_topic}")
        self.get_logger().info(
            f"코너 각도 기준             : {self.corner_angle_threshold_deg:.1f} deg"
        )
        self.get_logger().info(
            f"코너 lookahead             : {self.corner_lookahead_distance:.2f} m"
        )
        self.get_logger().info(
            f"접근 바깥 shift            : {self.approach_outer_shift:.2f} m"
        )
        self.get_logger().info(
            f"접근 shift 길이            : {self.approach_shift_length:.2f} m"
        )
        self.get_logger().info(
            f"회전 진입/이탈 거리        : {self.turn_entry_distance:.2f} / "
            f"{self.turn_exit_distance:.2f} m"
        )
        self.get_logger().info(
            f"B통로 hold/복귀 길이       : {self.exit_hold_length:.2f} / "
            f"{self.merge_back_length:.2f} m"
        )
        self.get_logger().info(
            f"wide-turn 추가 handle      : {self.corner_outer_offset:.2f} m"
        )
        self.get_logger().info(
            f"맵 코너 검출               : {self.use_map_corner_detection}"
        )
        if self.use_map_corner_detection:
            self.get_logger().info(
                "지도 코너 후보              : 입력 경로 기반 맵 정제 후 추출"
            )
            self.get_logger().info(
                f"지도 코너-경로 최대 거리    : {self.map_corner_path_max_distance:.2f} m"
            )
        self.get_logger().info(
            f"맵 안전 검사               : {self.use_map_safety_check}"
        )
        if self.use_map_safety_check:
            self.get_logger().info(
                f"최소 장애물 이격 거리       : {self.min_clearance:.2f} m"
            )
        self.get_logger().info("================================================")

        self.process_csv_path()

    # CSV input processing
    def process_csv_path(self) -> None:
        """Load the recorded path and build a map-safe corner-turn path."""
        input_xy = self.load_path_csv(self.input_csv_path)
        if len(input_xy) < 3:
            raise RuntimeError("입력 safe_path.csv에는 최소 3개 이상의 점이 필요합니다.")

        filtered_xy = self.remove_near_duplicate_points(
            input_xy, self.min_point_gap
        )
        working_xy = self.resample_path(
            filtered_xy, self.working_resample_interval
        )

        # Use the same cleaned map for corner extraction and safety checks.
        if self.map_ready:
            self.refine_map_near_input_path(working_xy)

        if self.use_map_corner_detection:
            self.map_corners_world = self.extract_map_corners()

        if self.use_map_corner_detection:
            corners = self.detect_map_corners_on_path(working_xy)
            if not corners and self.fallback_to_path_corner_detection:
                self.get_logger().warn(
                    "지도 코너와 일치하는 경로 코너가 없어 경로 형상 기반 검출로 fallback합니다."
                )
                corners = self.detect_corners(working_xy)
        else:
            corners = self.detect_corners(working_xy)

        # Preserve input coordinates when no corner matches; resampling would change the saved CSV.
        if not corners:
            self.get_logger().warn(
                "적용 가능한 코너가 없습니다. 입력 safe_path 좌표를 그대로 저장합니다."
            )
            output_xy = input_xy.copy()
            stats = {
                "curve_applied": 0,
                "curve_offset_reduced": 0,
                "curve_unsafe_fallback": 0,
            }
        else:
            output_xy, stats = self.generate_corner_turn_path(working_xy, corners)

        if len(output_xy) < 2:
            raise RuntimeError("생성된 corner_turn_path의 점 개수가 2개 미만입니다.")

        output_xy = self.remove_near_duplicate_points(
            output_xy, max(1e-4, self.output_sample_interval * 0.15)
        )
        yaw = self.compute_yaw_from_path(output_xy)

        invalid_count = self.validate_path(output_xy)
        stats["final_invalid_count"] = invalid_count

        if invalid_count > 0:
            self.get_logger().warn(
                f"최종 corner_turn_path 안전 검사 실패 구간/점={invalid_count}"
            )
        else:
            self.get_logger().info("최종 corner_turn_path 안전 검사 통과")

        self.save_path_csv(self.output_csv_path, output_xy, yaw)
        self.get_logger().info(
            f"corner_turn_path.csv 저장 완료: {self.output_csv_path}"
        )

        if self.publish_output_path:
            self.publish_path(output_xy, yaw, self.output_frame_id)

        self.last_output_xy = output_xy
        self.print_statistics(
            input_count=len(input_xy),
            working_count=len(working_xy),
            output_count=len(output_xy),
            corner_count=len(corners),
            stats=stats,
        )

    # Corner detection
    def extract_map_corners(self) -> np.ndarray:
        """Extract physical obstacle corners from occupied cells, excluding unknown boundaries."""
        if not hasattr(self, "occupied_geometry_map"):
            return np.empty((0, 2), dtype=np.float64)

        source = self.occupied_geometry_map.astype(np.uint8)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            source, connectivity=8
        )

        min_area_cells = max(
            1,
            int(round(
                self.map_corner_min_component_area_m2 / (self.resolution ** 2)
            )),
        )
        epsilon_cells = max(
            1.0, self.map_corner_approx_epsilon / self.resolution
        )

        corners: List[np.ndarray] = []

        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < min_area_cells:
                continue

            component = np.zeros_like(source, dtype=np.uint8)
            component[labels == label] = 255
            contours, _ = cv2.findContours(
                component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                if len(contour) < 3:
                    continue

                approx = cv2.approxPolyDP(
                    contour, epsilon_cells, closed=True
                ).reshape(-1, 2)
                if len(approx) < 3:
                    continue

                for i in range(len(approx)):
                    previous = approx[(i - 1) % len(approx)].astype(np.float64)
                    current = approx[i].astype(np.float64)
                    following = approx[(i + 1) % len(approx)].astype(np.float64)

                    vector_a = previous - current
                    vector_b = following - current
                    norm_a = float(np.linalg.norm(vector_a))
                    norm_b = float(np.linalg.norm(vector_b))
                    if norm_a < 1.0 or norm_b < 1.0:
                        continue

                    dot_value = float(np.clip(
                        np.dot(vector_a / norm_a, vector_b / norm_b),
                        -1.0,
                        1.0,
                    ))
                    angle_deg = math.degrees(math.acos(dot_value))
                    if angle_deg < self.map_corner_min_angle_deg:
                        continue
                    if angle_deg > self.map_corner_max_angle_deg:
                        continue

                    column = float(current[0])
                    row = float(current[1])
                    corners.append(self.map_to_world(row, column))

        if not corners:
            self.get_logger().warn("격자 지도에서 유효한 장애물 코너를 찾지 못했습니다.")
            return np.empty((0, 2), dtype=np.float64)

        # Merge nearby detections caused by wall thickness or contour approximation.
        merged: List[np.ndarray] = []
        for corner in corners:
            matched_index = None
            for index, existing in enumerate(merged):
                if np.linalg.norm(corner - existing) <= self.map_corner_merge_distance:
                    matched_index = index
                    break

            if matched_index is None:
                merged.append(corner.copy())
            else:
                merged[matched_index] = 0.5 * (merged[matched_index] + corner)

        self.get_logger().info(
            "격자 지도 코너 추출 완료. "
            f"원본 후보={len(corners)}, 병합 후={len(merged)}"
        )
        return np.asarray(merged, dtype=np.float64)

    def detect_map_corners_on_path(
        self, xy: np.ndarray
    ) -> List[Dict[str, object]]:
        """Match path-turn candidates to nearby obstacle corners on the inside of each turn."""
        if len(xy) < 3 or len(self.map_corners_world) == 0:
            return []

        # Find candidate path turns before matching them to map corners.
        path_candidates = self.detect_corners(xy)
        self.get_logger().info(
            f"경로 형상 기반 회전 후보 검출 완료: {len(path_candidates)}개"
        )

        if not path_candidates:
            self.get_logger().warn(
                "safe_path에서 회전 후보를 찾지 못했습니다. "
                "corner_angle_threshold_deg 또는 corner_lookahead_distance를 확인하세요."
            )
            return []

        cumulative = self.compute_cumulative_distance(xy)
        total_length = float(cumulative[-1])
        matched: List[Dict[str, object]] = []
        used_map_corner_indices = set()

        no_nearby_map_corner = 0
        rejected_by_turn_side = 0

        for path_candidate in path_candidates:
            candidate_s = float(path_candidate["s"])
            p_center = self.interpolate_at_distance(xy, cumulative, candidate_s)
            p_before = self.interpolate_at_distance(
                xy,
                cumulative,
                max(0.0, candidate_s - self.corner_lookahead_distance),
            )
            p_after = self.interpolate_at_distance(
                xy,
                cumulative,
                min(total_length, candidate_s + self.corner_lookahead_distance),
            )
            u_in = np.asarray(path_candidate["u_in"], dtype=np.float64)
            u_out = np.asarray(path_candidate["u_out"], dtype=np.float64)
            turn_sign = float(path_candidate["turn_sign"])
            estimated_corner = np.asarray(
                path_candidate["corner"], dtype=np.float64
            )

            # Measure corner proximity against the local path, not only the turn center.
            local_start_s = max(
                0.0,
                candidate_s - max(
                    self.corner_lookahead_distance,
                    self.approach_shift_length + self.turn_entry_distance,
                ),
            )
            local_end_s = min(
                total_length,
                candidate_s + max(
                    self.corner_lookahead_distance,
                    self.turn_exit_distance
                    + self.exit_hold_length
                    + self.merge_back_length,
                ),
            )
            local_path = self.sample_polyline_range(
                xy,
                cumulative,
                local_start_s,
                local_end_s,
                max(self.output_sample_interval, self.working_resample_interval),
            )

            nearby_candidates: List[Tuple[float, float, float, int, np.ndarray]] = []
            nearby_before_side_check = 0

            for map_index, map_corner in enumerate(self.map_corners_world):
                if map_index in used_map_corner_indices:
                    continue

                if len(local_path) > 0:
                    path_distance = float(
                        np.min(np.linalg.norm(local_path - map_corner, axis=1))
                    )
                else:
                    path_distance = float(np.linalg.norm(p_center - map_corner))

                # Require map corners to lie near the candidate's local path.
                if path_distance > self.map_corner_path_max_distance:
                    continue

                nearby_before_side_check += 1

                # Check that the obstacle corner lies inside both incoming and outgoing turns.
                incoming_inside = (
                    self.cross2d(u_in, map_corner - p_before) * turn_sign
                )
                outgoing_inside = (
                    self.cross2d(u_out, map_corner - p_after) * turn_sign
                )
                if incoming_inside <= self.map_corner_inside_margin:
                    continue
                if outgoing_inside <= self.map_corner_inside_margin:
                    continue

                reference_distance = float(
                    np.linalg.norm(map_corner - estimated_corner)
                )

                # Rank by estimated intersection distance, using local path distance as a
                # secondary score.
                match_score = reference_distance + 0.25 * path_distance
                nearby_candidates.append(
                    (
                        match_score,
                        reference_distance,
                        path_distance,
                        map_index,
                        map_corner.copy(),
                    )
                )

            if not nearby_candidates:
                if nearby_before_side_check == 0:
                    no_nearby_map_corner += 1
                    self.get_logger().warn(
                        "[경로 코너 미매칭: 주변 지도 코너 없음] "
                        f"s={candidate_s:.2f} m, "
                        f"angle={float(path_candidate['angle_deg']):.1f} deg"
                    )
                else:
                    rejected_by_turn_side += 1
                    self.get_logger().warn(
                        "[경로 코너 미매칭: 지도 코너가 회전 안쪽이 아님] "
                        f"s={candidate_s:.2f} m, "
                        f"angle={float(path_candidate['angle_deg']):.1f} deg, "
                        f"nearby_map_corners={nearby_before_side_check}"
                    )
                continue

            nearby_candidates.sort(key=lambda item: item[0])
            (
                match_score,
                reference_distance,
                path_distance,
                map_index,
                matched_map_corner,
            ) = nearby_candidates[0]
            used_map_corner_indices.add(map_index)

            matched_candidate = dict(path_candidate)
            matched_candidate["path_estimated_corner"] = estimated_corner.copy()
            matched_candidate["corner"] = matched_map_corner.copy()
            matched_candidate["map_corner_index"] = map_index
            matched_candidate["map_corner_distance"] = path_distance
            matched_candidate["map_corner_reference_distance"] = reference_distance
            matched_candidate["map_corner_match_score"] = match_score
            matched.append(matched_candidate)

            self.get_logger().info(
                "[경로 코너-지도 코너 매칭] "
                f"s={candidate_s:.2f} m, "
                f"turn_angle={float(path_candidate['angle_deg']):.1f} deg, "
                f"map_corner=({matched_map_corner[0]:.2f}, "
                f"{matched_map_corner[1]:.2f}), "
                f"path_distance={path_distance:.2f} m, "
                f"estimated_corner_distance={reference_distance:.2f} m"
            )

        # Corner suppression is complete; sort the remaining turns by path order.
        matched.sort(key=lambda item: float(item["s"]))

        self.get_logger().info(
            "경로 우선 지도 코너 매칭 완료. "
            f"경로 후보={len(path_candidates)}, "
            f"매칭 성공={len(matched)}, "
            f"주변 지도 코너 없음={no_nearby_map_corner}, "
            f"회전 안쪽 조건 탈락={rejected_by_turn_side}"
        )

        return matched

    def detect_corners(self, xy: np.ndarray) -> List[Dict[str, object]]:
        """Find route bends that need a wide turn."""
        if len(xy) < 3:
            return []

        cumulative = self.compute_cumulative_distance(xy)
        total_length = float(cumulative[-1])
        candidates: List[Dict[str, object]] = []

        for i in range(1, len(xy) - 1):
            s = float(cumulative[i])

            if s <= self.preserve_start_distance:
                continue
            if (total_length - s) <= self.preserve_goal_distance:
                continue
            if s < self.corner_lookahead_distance:
                continue
            if (total_length - s) < self.corner_lookahead_distance:
                continue

            p_before = self.interpolate_at_distance(
                xy, cumulative, s - self.corner_lookahead_distance
            )
            p_center = xy[i]
            p_after = self.interpolate_at_distance(
                xy, cumulative, s + self.corner_lookahead_distance
            )

            incoming = p_center - p_before
            outgoing = p_after - p_center

            incoming_norm = np.linalg.norm(incoming)
            outgoing_norm = np.linalg.norm(outgoing)
            if incoming_norm < 1e-9 or outgoing_norm < 1e-9:
                continue

            u_in = incoming / incoming_norm
            u_out = outgoing / outgoing_norm

            dot_value = float(np.clip(np.dot(u_in, u_out), -1.0, 1.0))
            cross_value = self.cross2d(u_in, u_out)
            angle_deg = math.degrees(math.atan2(abs(cross_value), dot_value))

            if angle_deg < self.corner_angle_threshold_deg:
                continue
            if angle_deg > self.corner_max_angle_deg:
                continue
            if abs(cross_value) < 1e-4:
                continue

            estimated_corner = self.estimate_line_intersection(
                p_before, u_in, p_after, u_out, p_center
            )

            candidates.append(
                {
                    "index": i,
                    "s": s,
                    "angle_deg": angle_deg,
                    "turn_sign": 1.0 if cross_value > 0.0 else -1.0,
                    "u_in": u_in,
                    "u_out": u_out,
                    "corner": estimated_corner,
                }
            )

        # Suppress duplicate candidates around the same corner.
        selected: List[Dict[str, object]] = []
        for candidate in sorted(
            candidates, key=lambda item: float(item["angle_deg"]), reverse=True
        ):
            candidate_s = float(candidate["s"])
            if any(
                abs(candidate_s - float(existing["s"]))
                < self.corner_min_separation
                for existing in selected
            ):
                continue

            maneuver_start_s, maneuver_end_s = self.get_maneuver_bounds(candidate_s)
            if maneuver_start_s <= self.preserve_start_distance:
                continue
            if (total_length - maneuver_end_s) <= self.preserve_goal_distance:
                continue

            selected.append(candidate)

        selected.sort(key=lambda item: float(item["s"]))

        # Discard later corners when complete steering intervals overlap.
        non_overlapping: List[Dict[str, object]] = []
        previous_end_s = -float("inf")
        for corner in selected:
            maneuver_start_s, maneuver_end_s = self.get_maneuver_bounds(
                float(corner["s"])
            )
            if maneuver_start_s <= previous_end_s + self.output_sample_interval:
                continue
            non_overlapping.append(corner)
            previous_end_s = maneuver_end_s

        return non_overlapping

    def get_maneuver_bounds(self, corner_s: float) -> Tuple[float, float]:
        """Return the path-distance interval from pre-shift through merge-back."""
        start_s = corner_s - (
            self.approach_shift_length + self.turn_entry_distance
        )
        end_s = corner_s + (
            self.turn_exit_distance
            + self.exit_hold_length
            + self.merge_back_length
        )
        return start_s, end_s

    def estimate_line_intersection(
        self,
        p1: np.ndarray,
        d1: np.ndarray,
        p2: np.ndarray,
        d2: np.ndarray,
        fallback: np.ndarray,
    ) -> np.ndarray:
        denominator = self.cross2d(d1, d2)
        if abs(denominator) < 1e-6:
            return fallback.copy()

        t = self.cross2d(p2 - p1, d2) / denominator
        intersection = p1 + t * d1

        # Use the actual path point when noise moves the estimated intersection too far.
        max_reasonable_distance = max(1.0, self.corner_lookahead_distance * 2.5)
        if np.linalg.norm(intersection - fallback) > max_reasonable_distance:
            return fallback.copy()

        return intersection

    # Wide-turn path generation
    def generate_corner_turn_path(
        self,
        xy: np.ndarray,
        corners: Sequence[Dict[str, object]],
    ) -> Tuple[np.ndarray, Dict[str, int]]:
        """Replace each matched corner with pre-shift, wide-turn, and merge-back segments."""
        cumulative = self.compute_cumulative_distance(xy)
        total_length = float(cumulative[-1])

        stats = {
            "curve_applied": 0,
            "curve_offset_reduced": 0,
            "curve_unsafe_fallback": 0,
        }

        output_points: List[np.ndarray] = []
        cursor_s = 0.0

        for corner_info in corners:
            corner_s = float(corner_info["s"])
            maneuver_start_s, maneuver_end_s = self.get_maneuver_bounds(corner_s)

            if maneuver_start_s <= cursor_s:
                continue
            if maneuver_end_s >= total_length:
                continue

            original_before = self.sample_polyline_range(
                xy,
                cumulative,
                cursor_s,
                maneuver_start_s,
                self.output_sample_interval,
            )
            self.append_points_without_duplicates(output_points, original_before)

            maneuver, used_shift = self.find_safe_wide_turn_maneuver(
                xy=xy,
                cumulative=cumulative,
                corner_info=corner_info,
            )

            if maneuver is not None:
                self.append_points_without_duplicates(output_points, maneuver)
                stats["curve_applied"] += 1
                if used_shift + 1e-6 < self.approach_outer_shift:
                    stats["curve_offset_reduced"] += 1

                self.get_logger().info(
                    "[pre-shift + wide-turn + merge 적용] "
                    f"s={corner_s:.2f} m, "
                    f"angle={float(corner_info['angle_deg']):.1f} deg, "
                    f"lateral_shift={used_shift:.2f} m"
                )
            else:
                original_corner_section = self.sample_polyline_range(
                    xy,
                    cumulative,
                    maneuver_start_s,
                    maneuver_end_s,
                    self.output_sample_interval,
                )
                self.append_points_without_duplicates(
                    output_points, original_corner_section
                )
                stats["curve_unsafe_fallback"] += 1

                self.get_logger().warn(
                    "[코너 원본 유지] pre-shift/wide-turn 경로가 안전 기준을 "
                    f"만족하지 못했습니다. s={corner_s:.2f} m"
                )

            cursor_s = maneuver_end_s

        remaining = self.sample_polyline_range(
            xy, cumulative, cursor_s, total_length, self.output_sample_interval
        )
        self.append_points_without_duplicates(output_points, remaining)

        if len(output_points) < 2:
            return xy.copy(), stats

        return np.asarray(output_points, dtype=np.float64), stats

    def find_safe_wide_turn_maneuver(
        self,
        xy: np.ndarray,
        cumulative: np.ndarray,
        corner_info: Dict[str, object],
    ) -> Tuple[Optional[np.ndarray], float]:
        """Reduce the requested offset until the widest safe maneuver is found."""
        requested_shift = max(0.0, self.approach_outer_shift)

        shifts: List[float] = [requested_shift]
        if self.reduce_outer_offset_on_collision:
            step = max(0.01, self.outer_offset_reduction_step)
            value = requested_shift - step
            while value > 1e-9:
                shifts.append(max(0.0, value))
                value -= step
            shifts.append(0.0)

        shifts = sorted(set(round(value, 6) for value in shifts), reverse=True)

        for lateral_shift in shifts:
            scale = 1.0
            if requested_shift > 1e-9:
                scale = lateral_shift / requested_shift

            maneuver = self.build_wide_turn_maneuver(
                xy=xy,
                cumulative=cumulative,
                corner_info=corner_info,
                lateral_shift=lateral_shift,
                turn_extra_handle=self.corner_outer_offset * scale,
            )

            if not self.use_map_safety_check or self.is_path_safe(maneuver):
                return maneuver, lateral_shift

        if self.fallback_to_original_on_unsafe:
            return None, 0.0

        maneuver = self.build_wide_turn_maneuver(
            xy=xy,
            cumulative=cumulative,
            corner_info=corner_info,
            lateral_shift=requested_shift,
            turn_extra_handle=self.corner_outer_offset,
        )
        return maneuver, requested_shift

    def build_wide_turn_maneuver(
        self,
        xy: np.ndarray,
        cumulative: np.ndarray,
        corner_info: Dict[str, object],
        lateral_shift: float,
        turn_extra_handle: float,
    ) -> np.ndarray:
        """Build the full pre-shift, wide-turn, and merge-back path."""
        corner_s = float(corner_info["s"])
        u_in = self.normalize_vector(
            np.asarray(corner_info["u_in"], dtype=np.float64)
        )
        u_out = self.normalize_vector(
            np.asarray(corner_info["u_out"], dtype=np.float64)
        )
        turn_sign = float(corner_info["turn_sign"])

        s1 = corner_s - self.turn_entry_distance
        s0 = s1 - self.approach_shift_length
        s2 = corner_s + self.turn_exit_distance
        s3 = s2 + self.exit_hold_length
        s4 = s3 + self.merge_back_length

        p0 = self.interpolate_at_distance(xy, cumulative, s0)
        p1_base = self.interpolate_at_distance(xy, cumulative, s1)
        p2_base = self.interpolate_at_distance(xy, cumulative, s2)
        p3_base = self.interpolate_at_distance(xy, cumulative, s3)
        p4 = self.interpolate_at_distance(xy, cumulative, s4)

        outside_in = self.outside_normal(u_in, turn_sign)
        outside_out = self.outside_normal(u_out, turn_sign)

        shifted_entry = p1_base + outside_in * lateral_shift
        shifted_exit = p2_base + outside_out * lateral_shift
        shifted_hold_end = p3_base + outside_out * lateral_shift

        # Match both pre-shift tangents to the incoming corridor to avoid a sharp kink.
        approach_handle = max(
            self.output_sample_interval,
            self.approach_shift_length * self.shift_handle_ratio,
        )
        approach_curve = self.sample_cubic_bezier_by_control_polygon(
            p0,
            p0 + u_in * approach_handle,
            shifted_entry - u_in * approach_handle,
            shifted_entry,
        )

        # Align the wide-turn tangents with the incoming and outgoing corridors.
        entry_handle = max(
            self.output_sample_interval,
            self.turn_entry_distance * self.turn_handle_ratio
            + max(0.0, turn_extra_handle),
        )
        exit_handle = max(
            self.output_sample_interval,
            self.turn_exit_distance * self.turn_handle_ratio
            + max(0.0, turn_extra_handle),
        )
        turn_curve = self.sample_cubic_bezier_by_control_polygon(
            shifted_entry,
            shifted_entry + u_in * entry_handle,
            shifted_exit - u_out * exit_handle,
            shifted_exit,
        )

        # Follow the local outgoing corridor while holding the offset.
        if self.exit_hold_length > 1e-6:
            hold_base = self.sample_polyline_range(
                xy,
                cumulative,
                s2,
                s3,
                self.output_sample_interval,
            )
            hold_curve = hold_base + outside_out * lateral_shift
        else:
            hold_curve = np.asarray([shifted_exit], dtype=np.float64)

        # Merge gradually from the offset path into the original safe_path.
        merge_handle = max(
            self.output_sample_interval,
            self.merge_back_length * self.shift_handle_ratio,
        )
        merge_curve = self.sample_cubic_bezier_by_control_polygon(
            shifted_hold_end,
            shifted_hold_end + u_out * merge_handle,
            p4 - u_out * merge_handle,
            p4,
        )

        maneuver_points: List[np.ndarray] = []
        self.append_points_without_duplicates(maneuver_points, approach_curve)
        self.append_points_without_duplicates(maneuver_points, turn_curve)
        self.append_points_without_duplicates(maneuver_points, hold_curve)
        self.append_points_without_duplicates(maneuver_points, merge_curve)

        if len(maneuver_points) < 2:
            return np.asarray([p0, p4], dtype=np.float64)

        maneuver = np.asarray(maneuver_points, dtype=np.float64)
        return self.resample_path(maneuver, self.output_sample_interval)

    def sample_cubic_bezier_by_control_polygon(
        self,
        p0: np.ndarray,
        p1: np.ndarray,
        p2: np.ndarray,
        p3: np.ndarray,
    ) -> np.ndarray:
        control_polygon_length = (
            np.linalg.norm(p1 - p0)
            + np.linalg.norm(p2 - p1)
            + np.linalg.norm(p3 - p2)
        )
        sample_count = max(
            12,
            int(math.ceil(control_polygon_length / self.output_sample_interval)),
        )
        t_values = np.linspace(0.0, 1.0, sample_count + 1)
        return np.asarray(
            [self.cubic_bezier(p0, p1, p2, p3, t) for t in t_values],
            dtype=np.float64,
        )

    @staticmethod
    def normalize_vector(vector: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm < 1e-9:
            return np.asarray([1.0, 0.0], dtype=np.float64)
        return vector / norm

    @staticmethod
    def outside_normal(direction: np.ndarray, turn_sign: float) -> np.ndarray:
        """Return the unit normal pointing away from the inside of a turn."""
        left_normal = np.asarray([-direction[1], direction[0]], dtype=np.float64)
        outside = -turn_sign * left_normal
        norm = float(np.linalg.norm(outside))
        if norm < 1e-9:
            return np.asarray([0.0, 0.0], dtype=np.float64)
        return outside / norm

    @staticmethod
    def cubic_bezier(
        p0: np.ndarray,
        p1: np.ndarray,
        p2: np.ndarray,
        p3: np.ndarray,
        t: float,
    ) -> np.ndarray:
        one_minus_t = 1.0 - t
        return (
            (one_minus_t ** 3) * p0
            + 3.0 * (one_minus_t ** 2) * t * p1
            + 3.0 * one_minus_t * (t ** 2) * p2
            + (t ** 3) * p3
        )

    def max_bezier_projection(
        self,
        p0: np.ndarray,
        p1: np.ndarray,
        p2: np.ndarray,
        p3: np.ndarray,
        origin: np.ndarray,
        axis: np.ndarray,
    ) -> float:
        max_projection = -float("inf")
        for t in np.linspace(0.0, 1.0, 81):
            point = self.cubic_bezier(p0, p1, p2, p3, float(t))
            projection = float(np.dot(point - origin, axis))
            max_projection = max(max_projection, projection)
        return max_projection

    # Path utilities
    def remove_near_duplicate_points(
        self, xy: np.ndarray, min_gap: float
    ) -> np.ndarray:
        if len(xy) <= 1:
            return xy.copy()

        filtered = [xy[0]]
        for point in xy[1:]:
            if np.linalg.norm(point - filtered[-1]) >= min_gap:
                filtered.append(point)

        return np.asarray(filtered, dtype=np.float64)

    def resample_path(self, xy: np.ndarray, interval: float) -> np.ndarray:
        if len(xy) < 2 or interval <= 0.0:
            return xy.copy()

        cumulative = self.compute_cumulative_distance(xy)
        total_length = float(cumulative[-1])
        if total_length <= interval:
            return xy.copy()

        sample_distances = np.arange(0.0, total_length, interval)
        if len(sample_distances) == 0 or sample_distances[-1] < total_length:
            sample_distances = np.append(sample_distances, total_length)

        return np.asarray(
            [self.interpolate_at_distance(xy, cumulative, s) for s in sample_distances],
            dtype=np.float64,
        )

    @staticmethod
    def compute_cumulative_distance(xy: np.ndarray) -> np.ndarray:
        if len(xy) == 0:
            return np.array([], dtype=np.float64)
        if len(xy) == 1:
            return np.zeros(1, dtype=np.float64)

        segment_lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        return np.insert(np.cumsum(segment_lengths), 0, 0.0)

    @staticmethod
    def interpolate_at_distance(
        xy: np.ndarray, cumulative: np.ndarray, distance: float
    ) -> np.ndarray:
        if distance <= 0.0:
            return xy[0].copy()
        if distance >= cumulative[-1]:
            return xy[-1].copy()

        index = int(np.searchsorted(cumulative, distance, side="right") - 1)
        index = max(0, min(index, len(xy) - 2))

        s0 = float(cumulative[index])
        s1 = float(cumulative[index + 1])
        if (s1 - s0) < 1e-9:
            return xy[index].copy()

        alpha = (distance - s0) / (s1 - s0)
        return (1.0 - alpha) * xy[index] + alpha * xy[index + 1]

    def sample_polyline_range(
        self,
        xy: np.ndarray,
        cumulative: np.ndarray,
        start_s: float,
        end_s: float,
        interval: float,
    ) -> np.ndarray:
        start_s = max(0.0, float(start_s))
        end_s = min(float(cumulative[-1]), float(end_s))

        if end_s < start_s:
            return np.empty((0, 2), dtype=np.float64)
        if abs(end_s - start_s) < 1e-9:
            return np.asarray(
                [self.interpolate_at_distance(xy, cumulative, start_s)],
                dtype=np.float64,
            )

        distances = np.arange(start_s, end_s, max(interval, 1e-4))
        if len(distances) == 0 or distances[-1] < end_s:
            distances = np.append(distances, end_s)

        return np.asarray(
            [self.interpolate_at_distance(xy, cumulative, s) for s in distances],
            dtype=np.float64,
        )

    @staticmethod
    def append_points_without_duplicates(
        destination: List[np.ndarray], points: np.ndarray
    ) -> None:
        for point in points:
            if destination and np.linalg.norm(point - destination[-1]) < 1e-6:
                continue
            destination.append(np.asarray(point, dtype=np.float64))

    @staticmethod
    def compute_yaw_from_path(xy: np.ndarray) -> np.ndarray:
        yaw_values: List[float] = []

        for i in range(len(xy)):
            if len(xy) == 1:
                yaw_values.append(0.0)
                continue

            if i == 0:
                delta = xy[1] - xy[0]
            elif i == len(xy) - 1:
                delta = xy[-1] - xy[-2]
            else:
                delta = xy[i + 1] - xy[i - 1]

            yaw_values.append(math.atan2(float(delta[1]), float(delta[0])))

        return np.asarray(yaw_values, dtype=np.float64)

    @staticmethod
    def cross2d(a: np.ndarray, b: np.ndarray) -> float:
        return float(a[0] * b[1] - a[1] * b[0])

    # Map loading and safety validation
    def load_map_from_yaml(self, yaml_path: str) -> None:
        """Load occupancy data and map geometry from the configured YAML."""
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"맵 YAML 파일이 없습니다: {yaml_path}")

        with open(yaml_path, "r", encoding="utf-8") as file:
            map_info = yaml.safe_load(file)

        image_path = map_info["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"맵 이미지를 읽지 못했습니다: {image_path}")

        self.resolution = float(map_info["resolution"])
        self.origin = map_info["origin"]
        self.negate = int(map_info.get("negate", 0))
        self.occupied_thresh = float(map_info.get("occupied_thresh", 0.65))
        self.free_thresh = float(map_info.get("free_thresh", 0.196))

        self.height, self.width = image.shape

        if self.negate == 0:
            occupancy_probability = (255.0 - image.astype(np.float32)) / 255.0
        else:
            occupancy_probability = image.astype(np.float32) / 255.0

        occupied = occupancy_probability > self.occupied_thresh
        free = occupancy_probability < self.free_thresh
        unknown = np.logical_not(np.logical_or(occupied, free))

        # Clean only the in-memory occupancy array; leave the source PGM unchanged.
        self.raw_occupied_map = occupied.astype(bool)
        self.unknown_map = unknown.astype(bool)
        self.rebuild_safety_maps(self.raw_occupied_map)
        self.map_ready = True

        self.get_logger().info(
            f"맵 로드 완료: {self.width}x{self.height}, "
            f"resolution={self.resolution:.3f} m/cell"
        )

    def rebuild_safety_maps(self, occupied_map: np.ndarray) -> None:
        """Rebuild corner and clearance maps from cleaned occupied cells."""
        # Use occupied cells for corner geometry and include unknown-space policy in safety checks.
        occupied_geometry = self.apply_morphological_closing(
            occupied_map.astype(bool), self.closing_radius
        )
        self.occupied_geometry_map = occupied_geometry

        if self.treat_unknown_as_obstacle:
            obstacle_map = np.logical_or(occupied_geometry, self.unknown_map)
        else:
            obstacle_map = occupied_geometry.copy()
        self.obstacle_map = obstacle_map

        free_uint8 = np.logical_not(obstacle_map).astype(np.uint8)
        distance_cells = cv2.distanceTransform(
            free_uint8, distanceType=cv2.DIST_L2, maskSize=5
        )
        self.distance_map = distance_cells * self.resolution

    def refine_map_near_input_path(self, path_xy: np.ndarray) -> None:
        """Remove small isolated obstacle remnants near the input path."""
        if not self.use_path_corridor_cleanup:
            self.rebuild_safety_maps(self.raw_occupied_map)
            self.get_logger().info("입력 경로 corridor 기반 맵 정제: 비활성화")
            return

        radius_cells = max(1, int(round(self.path_corridor_radius / self.resolution)))
        corridor = np.zeros((self.height, self.width), dtype=np.uint8)
        previous: Optional[Tuple[int, int]] = None
        valid_points = 0

        for point in path_xy:
            row_column = self.world_to_map(float(point[0]), float(point[1]))
            if row_column is None:
                previous = None
                continue

            row, column = row_column
            valid_points += 1
            cv2.circle(corridor, (column, row), radius_cells, 255, thickness=-1)
            if previous is not None:
                cv2.line(
                    corridor,
                    (previous[1], previous[0]),
                    (column, row),
                    255,
                    thickness=2 * radius_cells + 1,
                )
            previous = (row, column)

        label_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            self.raw_occupied_map.astype(np.uint8), connectivity=8
        )
        cleaned = self.raw_occupied_map.copy()
        max_area_cells = max(
            1,
            int(round(self.dynamic_component_max_area_m2 / (self.resolution ** 2))),
        )
        max_extent_cells = max(
            1, int(round(self.dynamic_component_max_extent_m / self.resolution))
        )
        removed_components = 0
        removed_cells = 0

        for label in range(1, label_count):
            left = int(stats[label, cv2.CC_STAT_LEFT])
            top = int(stats[label, cv2.CC_STAT_TOP])
            width = int(stats[label, cv2.CC_STAT_WIDTH])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            area = int(stats[label, cv2.CC_STAT_AREA])
            component = labels == label

            if not np.any(component & (corridor > 0)):
                continue
            if area > max_area_cells or max(width, height) > max_extent_cells:
                continue
            if max(width, height) / max(1, min(width, height)) > self.dynamic_component_max_aspect_ratio:
                continue

            cleaned[component] = False
            removed_components += 1
            removed_cells += area

        self.rebuild_safety_maps(cleaned)
        self.get_logger().info(
            "입력 경로 corridor 기반 맵 정제 완료. "
            f"유효 경로점={valid_points}, 반경={self.path_corridor_radius:.2f} m, "
            f"제거 component={removed_components}, 제거 cell={removed_cells}"
        )

    def apply_morphological_closing(
        self, obstacle_map: np.ndarray, radius_m: float
    ) -> np.ndarray:
        if radius_m <= 0.0:
            return obstacle_map

        radius_cells = max(1, int(round(radius_m / self.resolution)))
        kernel_size = radius_cells * 2 + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        source = obstacle_map.astype(np.uint8) * 255
        closed = cv2.morphologyEx(source, cv2.MORPH_CLOSE, kernel)
        return closed > 0

    def map_to_world(self, row: float, column: float) -> np.ndarray:
        x = float(self.origin[0]) + (column + 0.5) * self.resolution
        row_from_bottom = (self.height - 1) - row
        y = float(self.origin[1]) + (row_from_bottom + 0.5) * self.resolution
        return np.array([x, y], dtype=np.float64)

    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        column = int(math.floor((x - float(self.origin[0])) / self.resolution))
        row_from_bottom = int(
            math.floor((y - float(self.origin[1])) / self.resolution)
        )
        row = self.height - 1 - row_from_bottom

        if row < 0 or row >= self.height or column < 0 or column >= self.width:
            return None
        return row, column

    def is_pose_safe(self, point: np.ndarray) -> bool:
        if not self.use_map_safety_check:
            return True
        if not self.map_ready:
            return False

        row_column = self.world_to_map(float(point[0]), float(point[1]))
        if row_column is None:
            return False

        row, column = row_column
        if self.obstacle_map[row, column]:
            return False

        clearance = float(self.distance_map[row, column])
        return clearance >= self.min_clearance

    def is_segment_safe(self, p0: np.ndarray, p1: np.ndarray) -> bool:
        if not self.use_map_safety_check:
            return True

        distance = float(np.linalg.norm(p1 - p0))
        if distance < 1e-9:
            return self.is_pose_safe(p0)

        step = max(0.005, self.safety_check_step)
        sample_count = max(2, int(math.ceil(distance / step)))

        for i in range(sample_count + 1):
            alpha = i / sample_count
            point = (1.0 - alpha) * p0 + alpha * p1
            if not self.is_pose_safe(point):
                return False

        return True

    def is_path_safe(self, xy: np.ndarray) -> bool:
        if not self.use_map_safety_check:
            return True

        for point in xy:
            if not self.is_pose_safe(point):
                return False

        for i in range(len(xy) - 1):
            if not self.is_segment_safe(xy[i], xy[i + 1]):
                return False

        return True

    def validate_path(self, xy: np.ndarray) -> int:
        """Reject generated paths that violate map clearance."""
        if not self.use_map_safety_check:
            return 0

        invalid_count = 0
        for point in xy:
            if not self.is_pose_safe(point):
                invalid_count += 1

        for i in range(len(xy) - 1):
            if not self.is_segment_safe(xy[i], xy[i + 1]):
                invalid_count += 1

        return invalid_count

    # ROS publish / CSV
    def publish_path(
        self, xy: np.ndarray, yaw: np.ndarray, frame_id: str
    ) -> None:
        """Publish the generated route for ROS visualization."""
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id

        for point, yaw_value in zip(xy, yaw):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0

            half_yaw = float(yaw_value) * 0.5
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)
            message.poses.append(pose)

        if self.path_pub is None:
            return

        self.path_pub.publish(message)
        self.get_logger().info(
            f"{self.output_path_topic} 발행 완료: pose={len(message.poses)}"
        )

    @staticmethod
    def load_path_csv(csv_path: str) -> np.ndarray:
        """Load supported path CSV layouts into an Nx2 coordinate array."""
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"입력 safe_path.csv가 없습니다: {csv_path}")

        with open(csv_path, "r", encoding="utf-8-sig") as file:
            first_line = file.readline().strip()

        if not first_line:
            raise RuntimeError(f"입력 CSV가 비어 있습니다: {csv_path}")

        has_header = any(character.isalpha() for character in first_line)
        points: List[List[float]] = []

        if has_header:
            with open(csv_path, "r", encoding="utf-8-sig", newline="") as file:
                reader = csv.DictReader(file)
                if reader.fieldnames is None:
                    raise RuntimeError("CSV 헤더를 읽지 못했습니다.")

                normalized = {
                    str(name).strip().lower(): name
                    for name in reader.fieldnames
                    if name is not None
                }

                x_key = next(
                    (
                        normalized[key]
                        for key in ("x", "pose_x", "position_x", "base_link_x")
                        if key in normalized
                    ),
                    None,
                )
                y_key = next(
                    (
                        normalized[key]
                        for key in ("y", "pose_y", "position_y", "base_link_y")
                        if key in normalized
                    ),
                    None,
                )

                if x_key is None or y_key is None:
                    raise RuntimeError(
                        "CSV 헤더에서 x, y 열을 찾지 못했습니다. "
                        f"현재 헤더={reader.fieldnames}"
                    )

                for line_number, row in enumerate(reader, start=2):
                    try:
                        x_text = str(row.get(x_key, "")).strip()
                        y_text = str(row.get(y_key, "")).strip()
                        if not x_text or not y_text:
                            continue
                        points.append([float(x_text), float(y_text)])
                    except (TypeError, ValueError) as exc:
                        raise RuntimeError(
                            f"CSV {line_number}행의 x/y 값을 읽지 못했습니다: {row}"
                        ) from exc
        else:
            try:
                data = np.loadtxt(csv_path, delimiter=",", dtype=np.float64)
            except ValueError as exc:
                raise RuntimeError(f"숫자 CSV를 읽지 못했습니다: {csv_path}") from exc

            if data.ndim == 1:
                data = data.reshape(1, -1)

            for row in data:
                if len(row) >= 4:
                    # Recorded-path CSV format: timestamp,x,y,yaw.
                    x_value, y_value = row[1], row[2]
                elif len(row) >= 2:
                    x_value, y_value = row[0], row[1]
                else:
                    continue
                points.append([float(x_value), float(y_value)])

        if len(points) < 2:
            raise RuntimeError("safe_path.csv에 유효한 경로점이 2개 미만입니다.")

        return np.asarray(points, dtype=np.float64)

    @staticmethod
    def save_path_csv(csv_path: str, xy: np.ndarray, yaw: np.ndarray) -> None:
        """Persist the generated path using the existing CSV layout."""
        output_directory = os.path.dirname(csv_path)
        if output_directory:
            os.makedirs(output_directory, exist_ok=True)

        with open(csv_path, "w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y", "yaw_rad"])
            for point, yaw_value in zip(xy, yaw):
                writer.writerow(
                    [
                        f"{float(point[0]):.15f}",
                        f"{float(point[1]):.15f}",
                        f"{float(yaw_value):.15f}",
                    ]
                )

    def print_statistics(
        self,
        input_count: int,
        working_count: int,
        output_count: int,
        corner_count: int,
        stats: Dict[str, int],
    ) -> None:
        self.get_logger().info("========== Corner Turn 생성 통계 ==========")
        self.get_logger().info(f"입력 safe_path 점 개수        : {input_count}")
        self.get_logger().info(f"작업용 재샘플링 점 개수       : {working_count}")
        self.get_logger().info(f"검출된 독립 코너 개수         : {corner_count}")
        self.get_logger().info(
            f"wide-turn 적용 개수          : {stats.get('curve_applied', 0)}"
        )
        self.get_logger().info(
            f"횡이동량 축소 개수           : {stats.get('curve_offset_reduced', 0)}"
        )
        self.get_logger().info(
            f"원본 safe_path 복구 개수      : {stats.get('curve_unsafe_fallback', 0)}"
        )
        self.get_logger().info(f"최종 경로 점 개수             : {output_count}")
        self.get_logger().info(
            f"최종 안전성 실패 개수         : {stats.get('final_invalid_count', 0)}"
        )
        self.get_logger().info("============================================")

    def _validate_parameters(self) -> None:
        if self.working_resample_interval <= 0.0:
            raise ValueError("working_resample_interval은 0보다 커야 합니다.")
        if self.output_sample_interval <= 0.0:
            raise ValueError("output_sample_interval은 0보다 커야 합니다.")
        if self.corner_lookahead_distance <= 0.0:
            raise ValueError("corner_lookahead_distance는 0보다 커야 합니다.")
        if self.corner_turn_radius <= 0.0:
            raise ValueError("corner_turn_radius는 0보다 커야 합니다.")
        if self.approach_outer_shift < 0.0:
            raise ValueError("approach_outer_shift는 0 이상이어야 합니다.")
        if self.approach_shift_length <= 0.0:
            raise ValueError("approach_shift_length는 0보다 커야 합니다.")
        if self.turn_entry_distance <= 0.0:
            raise ValueError("turn_entry_distance는 0보다 커야 합니다.")
        if self.turn_exit_distance <= 0.0:
            raise ValueError("turn_exit_distance는 0보다 커야 합니다.")
        if self.exit_hold_length < 0.0:
            raise ValueError("exit_hold_length는 0 이상이어야 합니다.")
        if self.merge_back_length <= 0.0:
            raise ValueError("merge_back_length는 0보다 커야 합니다.")
        if not (0.05 <= self.shift_handle_ratio <= 0.95):
            raise ValueError("shift_handle_ratio는 0.05~0.95 범위여야 합니다.")
        if self.turn_handle_ratio <= 0.0:
            raise ValueError("turn_handle_ratio는 0보다 커야 합니다.")
        if self.corner_outer_offset < 0.0:
            raise ValueError("corner_outer_offset은 0 이상이어야 합니다.")
        if self.corner_angle_threshold_deg <= 0.0:
            raise ValueError("corner_angle_threshold_deg는 0보다 커야 합니다.")
        if self.corner_max_angle_deg <= self.corner_angle_threshold_deg:
            raise ValueError(
                "corner_max_angle_deg는 corner_angle_threshold_deg보다 커야 합니다."
            )
        if self.map_corner_approx_epsilon <= 0.0:
            raise ValueError("map_corner_approx_epsilon은 0보다 커야 합니다.")
        if self.map_corner_path_max_distance <= 0.0:
            raise ValueError("map_corner_path_max_distance는 0보다 커야 합니다.")
        if self.map_corner_merge_distance < 0.0:
            raise ValueError("map_corner_merge_distance는 0 이상이어야 합니다.")
        if self.map_corner_min_component_area_m2 < 0.0:
            raise ValueError("map_corner_min_component_area_m2는 0 이상이어야 합니다.")
        if self.map_corner_max_angle_deg <= self.map_corner_min_angle_deg:
            raise ValueError(
                "map_corner_max_angle_deg는 map_corner_min_angle_deg보다 커야 합니다."
            )
        if self.path_corridor_radius <= 0.0:
            raise ValueError("path_corridor_radius는 0보다 커야 합니다.")
        if self.dynamic_component_max_area_m2 < 0.0:
            raise ValueError("dynamic_component_max_area_m2는 0 이상이어야 합니다.")
        if self.dynamic_component_max_extent_m <= 0.0:
            raise ValueError("dynamic_component_max_extent_m은 0보다 커야 합니다.")
        if self.dynamic_component_max_aspect_ratio < 1.0:
            raise ValueError(
                "dynamic_component_max_aspect_ratio는 1 이상이어야 합니다."
            )


def main(args=None) -> None:
    """Run the corner-turn generator as a ROS node."""
    rclpy.init(args=args)
    node: Optional[MapCornerTurnPathGeneratorNode] = None

    try:
        node = MapCornerTurnPathGeneratorNode()

        # Allow DDS time to deliver the transient-local path sample.
        if node.publish_output_path:
            rclpy.spin_once(node, timeout_sec=0.2)

        node.get_logger().info(
            "safe_path.csv 변환 및 corner_turn_path.csv 저장 작업이 완료되었습니다."
        )
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        if node is not None:
            node.get_logger().error(f"corner turn 경로 생성 실패: {exc}")
        else:
            print(f"corner turn 경로 생성 실패: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
