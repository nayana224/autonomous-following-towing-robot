#!/usr/bin/env python3
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

import os
import csv
import math
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class SafePathGeneratorNode(Node):
    """Generate a map-safe path before replay."""

    def __init__(self):
        """Declare path and map parameters before running the generator."""
        super().__init__("safe_path_generator_dynamic_cleanup_node")

        self.declare_parameter("map_yaml_path", "/home/jaebeom/map/mdbot_map.yaml")
        self.declare_parameter("raw_path_csv_path", "/home/jaebeom/recorded_path.csv")
        self.declare_parameter("safe_path_csv_path", "/home/jaebeom/safe_path.csv")

        self.declare_parameter("frame_id", "map")

        self.declare_parameter("resample_interval", 0.20)
        self.declare_parameter("min_point_gap", 0.03)

        self.declare_parameter("robot_radius", 0.35)
        self.declare_parameter("safety_margin", 0.15)

        self.declare_parameter("max_lateral_shift", 0.50)
        self.declare_parameter("lateral_sample_step", 0.05)
        self.declare_parameter("radius_search_step", 0.05)

        self.declare_parameter("treat_unknown_as_obstacle", True)
        self.declare_parameter("closing_radius", 0.15)
        self.declare_parameter("remove_small_components", False)
        self.declare_parameter("min_component_area_cells", 8)

        # Treat only small isolated occupied components near the recorded path as transient
        # obstacles.
        self.declare_parameter("use_path_corridor_cleanup", True)
        self.declare_parameter("path_corridor_radius", 0.50)
        self.declare_parameter("dynamic_component_max_area_m2", 0.80)
        self.declare_parameter("dynamic_component_max_extent_m", 1.50)
        self.declare_parameter("dynamic_component_max_aspect_ratio", 4.0)

        # Keep path endpoints fixed so the drive manager's start-pose check remains valid.
        self.declare_parameter("preserve_start_distance", 1.0)
        self.declare_parameter("preserve_goal_distance", 1.0)

        self.declare_parameter("smooth_window_size", 5)

        self.declare_parameter("w_clearance", 2.0)
        self.declare_parameter("w_deviation", 1.0)
        self.declare_parameter("w_smooth", 0.5)
        self.declare_parameter("clearance_score_cap", 1.0)

        self.map_yaml_path = self.get_parameter("map_yaml_path").value
        self.raw_path_csv_path = self.get_parameter("raw_path_csv_path").value
        self.safe_path_csv_path = self.get_parameter("safe_path_csv_path").value
        self.frame_id = self.get_parameter("frame_id").value

        self.resample_interval = float(self.get_parameter("resample_interval").value)
        self.min_point_gap = float(self.get_parameter("min_point_gap").value)

        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.safety_margin = float(self.get_parameter("safety_margin").value)
        # Require robot radius plus safety margin as the minimum clearance.
        self.min_clearance = self.robot_radius + self.safety_margin

        self.max_lateral_shift = float(self.get_parameter("max_lateral_shift").value)
        self.lateral_sample_step = float(self.get_parameter("lateral_sample_step").value)
        self.radius_search_step = float(self.get_parameter("radius_search_step").value)

        self.treat_unknown_as_obstacle = bool(self.get_parameter("treat_unknown_as_obstacle").value)
        self.closing_radius = float(self.get_parameter("closing_radius").value)
        self.remove_small_components = bool(self.get_parameter("remove_small_components").value)
        self.min_component_area_cells = int(self.get_parameter("min_component_area_cells").value)

        self.use_path_corridor_cleanup = bool(self.get_parameter("use_path_corridor_cleanup").value)
        self.path_corridor_radius = float(self.get_parameter("path_corridor_radius").value)
        self.dynamic_component_max_area_m2 = float(self.get_parameter("dynamic_component_max_area_m2").value)
        self.dynamic_component_max_extent_m = float(self.get_parameter("dynamic_component_max_extent_m").value)
        self.dynamic_component_max_aspect_ratio = float(self.get_parameter("dynamic_component_max_aspect_ratio").value)

        self.preserve_start_distance = float(self.get_parameter("preserve_start_distance").value)
        self.preserve_goal_distance = float(self.get_parameter("preserve_goal_distance").value)

        # Use minimum clearance when the configured corridor radius is non-positive.
        if self.path_corridor_radius <= 0.0:
            self.path_corridor_radius = self.min_clearance

        self.smooth_window_size = int(self.get_parameter("smooth_window_size").value)
        if self.smooth_window_size % 2 == 0: # Keep the smoothing window odd so it has a center sample.
            self.smooth_window_size += 1

        self.w_clearance = float(self.get_parameter("w_clearance").value)
        self.w_deviation = float(self.get_parameter("w_deviation").value)
        self.w_smooth = float(self.get_parameter("w_smooth").value)
        self.clearance_score_cap = float(self.get_parameter("clearance_score_cap").value)

        # Use transient-local durability so late subscribers receive the path.
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, "/safe_path", qos)

        self.safe_path_xy = None
        self.safe_path_yaw = None
        self.failure_stats = {}

        self.get_logger().info("========== Safe Path Generator 파라미터 ==========")
        self.get_logger().info(f"맵 YAML 경로              : {self.map_yaml_path}")
        self.get_logger().info(f"원본 경로 CSV 경로        : {self.raw_path_csv_path}")
        self.get_logger().info(f"저장할 safe path CSV 경로 : {self.safe_path_csv_path}")
        self.get_logger().info(f"로봇 반경                 : {self.robot_radius:.3f} m")
        self.get_logger().info(f"안전 여유 거리            : {self.safety_margin:.3f} m")
        self.get_logger().info(f"최소 장애물 이격 거리     : {self.min_clearance:.3f} m")
        self.get_logger().info(f"경로 재샘플링 간격        : {self.resample_interval:.3f} m")
        self.get_logger().info(f"최대 좌우 보정 거리       : {self.max_lateral_shift:.3f} m")
        self.get_logger().info(f"경로 corridor 정제 사용   : {self.use_path_corridor_cleanup}")
        self.get_logger().info(f"경로 corridor 반경        : {self.path_corridor_radius:.3f} m")
        self.get_logger().info(f"시작/도착 보존 거리       : {self.preserve_start_distance:.3f} / {self.preserve_goal_distance:.3f} m")
        self.get_logger().info("================================================")

        self.generate_safe_path()

    def generate_safe_path(self):
        """Run map cleanup, path correction, smoothing, and final safety checks."""
        if not os.path.exists(self.map_yaml_path):
            raise FileNotFoundError(f"맵 YAML 파일이 존재하지 않습니다: {self.map_yaml_path}")

        if not os.path.exists(self.raw_path_csv_path):
            raise FileNotFoundError(f"원본 경로 CSV 파일이 존재하지 않습니다: {self.raw_path_csv_path}")

        self.get_logger().info(f"맵 파일 로드 시작: {self.map_yaml_path}")
        self.load_map_from_yaml(self.map_yaml_path)

        self.get_logger().info(f"원본 경로 CSV 로드 시작: {self.raw_path_csv_path}")
        raw_xy = self.load_raw_path_csv(self.raw_path_csv_path)
        self.get_logger().info(f"원본 경로 점 개수: {len(raw_xy)}")

        filtered_xy = self.remove_near_duplicate_points(raw_xy, self.min_point_gap)
        self.get_logger().info(f"중복 제거 후 경로 점 개수: {len(filtered_xy)}")

        resampled_xy = self.resample_path(filtered_xy, self.resample_interval)
        self.get_logger().info(f"거리 기준 재샘플링 후 경로 점 개수: {len(resampled_xy)}")

        # Clean only occupied cells; unknown cells must never become traversable.
        # Unknown space must remain subject to the configured safety policy.
        self.get_logger().info("장애물 맵 생성 중...")
        occupied_map = self.occupied_cells.copy()

        if self.remove_small_components:
            occupied_map = self.remove_small_obstacle_components(occupied_map, self.min_component_area_cells)

        if self.use_path_corridor_cleanup:
            occupied_map = self.remove_dynamic_obstacle_artifacts_near_path(occupied_map, resampled_xy)

        obstacle_map = self.compose_obstacle_map(occupied_map)

        # Close map gaps after cleanup to avoid joining transient blobs to walls.
        obstacle_map = self.apply_morphological_closing(obstacle_map, self.closing_radius)
        self.obstacle_map = obstacle_map

        self.get_logger().info("거리 변환 맵 생성 중...")
        self.distance_map = self.compute_distance_map(self.obstacle_map)

        self.get_logger().info("원본 경로를 safe path로 보정 중...")
        corrected_xy, self.failure_stats = self.correct_path_points(resampled_xy)

        self.get_logger().info("safe path smoothing 수행 중...")
        smoothed_xy = self.smooth_path_with_safety_check(corrected_xy)

        self.get_logger().info("safe path yaw 재계산 중...")
        yaw = self.compute_yaw_from_path(smoothed_xy)

        invalid_count = self.validate_final_path(smoothed_xy)

        if invalid_count > 0:
            self.get_logger().warn(
                f"최종 경로에 안전하지 않은 점 또는 구간이 {invalid_count}개 있습니다. "
                "맵, robot_radius, safety_margin, max_lateral_shift 값을 확인하세요."
            )
        else:
            self.get_logger().info("최종 경로 안전성 검사를 통과했습니다.")

        self.safe_path_xy = smoothed_xy
        self.safe_path_yaw = yaw

        self.save_safe_path_csv(self.safe_path_csv_path, self.safe_path_xy, self.safe_path_yaw)
        self.get_logger().info(f"safe_path.csv 저장 완료: {self.safe_path_csv_path}")

        self.print_generation_statistics(
            raw_count=len(raw_xy), filtered_count=len(filtered_xy), resampled_count=len(resampled_xy),
            corrected_count=len(corrected_xy), final_count=len(smoothed_xy), invalid_count=invalid_count,
            stats=self.failure_stats
        )

    def load_map_from_yaml(self, yaml_path):
        """Load ROS map metadata and convert its image to occupancy probabilities."""
        with open(yaml_path, "r") as f: #
            map_info = yaml.safe_load(f)

        image_path = map_info["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        if not os.path.exists(image_path):
            raise FileNotFoundError(f"맵 이미지 파일이 존재하지 않습니다: {image_path}")

        self.resolution = float(map_info["resolution"])
        self.origin = map_info["origin"]
        self.negate = int(map_info.get("negate", 0))
        self.occupied_thresh = float(map_info.get("occupied_thresh", 0.65))
        self.free_thresh = float(map_info.get("free_thresh", 0.196))

        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"맵 이미지를 로드하지 못했습니다: {image_path}")

        self.map_image = image
        self.height, self.width = image.shape

        self.map_min_x = self.origin[0]
        self.map_min_y = self.origin[1]
        self.map_max_x = self.origin[0] + self.width * self.resolution
        self.map_max_y = self.origin[1] + self.height * self.resolution

        if len(self.origin) >= 3 and abs(float(self.origin[2])) > 1e-6:
            self.get_logger().warn(f"맵 origin yaw가 0이 아닙니다. 이 노드는 오직 yaw=0만 가정하고 연산합니다.")

        if self.negate == 0:
            occ_prob = (255.0 - image.astype(np.float32)) / 255.0
        else:
            occ_prob = image.astype(np.float32) / 255.0

        occupied = occ_prob > self.occupied_thresh
        free = occ_prob < self.free_thresh
        unknown = np.logical_not(np.logical_or(occupied, free))

        self.occupied_cells = occupied
        self.free_cells = free
        self.unknown_cells = unknown

        self.get_logger().info(
            f"맵 로드 완료. 크기={self.width}x{self.height}, 해상도={self.resolution:.3f} m/cell, "
            f"origin=({self.origin[0]:.3f}, {self.origin[1]:.3f})"
        )
        self.get_logger().info(f"맵 월드 좌표 범위: x=[{self.map_min_x:.3f}, {self.map_max_x:.3f}], y=[{self.map_min_y:.3f}, {self.map_max_y:.3f}]")

    def build_obstacle_map(self):
        """Compose an obstacle map using the configured unknown-space policy."""
        return self.compose_obstacle_map(self.occupied_cells)

    def compose_obstacle_map(self, occupied_map):
        """Merge cleaned occupied cells with unknown cells according to safety policy."""
        if self.treat_unknown_as_obstacle:
            obstacle = np.logical_or(occupied_map, self.unknown_cells)
        else:
            obstacle = occupied_map.copy()
        return obstacle.astype(bool)

    def make_path_corridor_mask(self, path_xy, radius_m):
        """Mark occupancy cells within the configured radius of the recorded path."""
        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        radius_cells = max(1, int(round(radius_m / self.resolution)))
        thickness = max(1, 2 * radius_cells + 1)

        prev_rc = None
        valid_points = 0

        for p in path_xy:
            rc = self.world_to_map(p[0], p[1])
            if rc is None:
                prev_rc = None
                continue

            row, col = rc
            valid_points += 1

            cv2.circle(mask, (col, row), radius_cells, 255, thickness=-1)

            if prev_rc is not None:
                prev_row, prev_col = prev_rc
                cv2.line(mask, (prev_col, prev_row), (col, row), 255, thickness=thickness)

            prev_rc = rc

        self.get_logger().info(
            f"경로 corridor mask 생성 완료. 유효 경로점={valid_points}, "
            f"반경={radius_m:.3f} m, 반경 cell={radius_cells}"
        )
        return mask > 0

    def remove_dynamic_obstacle_artifacts_near_path(self, occupied_map, path_xy):
        """Remove only small isolated occupied components near the path; preserve walls and unknown cells."""
        if occupied_map is None or occupied_map.size == 0:
            return occupied_map

        corridor_mask = self.make_path_corridor_mask(path_xy, self.path_corridor_radius)

        src = occupied_map.astype(np.uint8)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(src, connectivity=8)

        cleaned = occupied_map.copy()

        max_area_cells = max(1, int(round(self.dynamic_component_max_area_m2 / (self.resolution ** 2))))
        max_extent_cells = max(1, int(round(self.dynamic_component_max_extent_m / self.resolution)))

        removed_components = 0
        removed_cells = 0
        corridor_components = 0

        for label in range(1, num_labels):
            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP])
            w = int(stats[label, cv2.CC_STAT_WIDTH])
            h = int(stats[label, cv2.CC_STAT_HEIGHT])
            area = int(stats[label, cv2.CC_STAT_AREA])

            component_mask = labels == label

            # Preserve components outside the recorded-path corridor.
            if not np.any(component_mask & corridor_mask):
                continue

            corridor_components += 1

            # Keep large structures and wall components.
            if area > max_area_cells:
                continue

            if max(w, h) > max_extent_cells:
                continue

            aspect_ratio = max(w, h) / max(1, min(w, h))
            if aspect_ratio > self.dynamic_component_max_aspect_ratio:
                continue

            cleaned[component_mask] = False
            removed_components += 1
            removed_cells += area

        removed_area_m2 = removed_cells * (self.resolution ** 2)
        self.get_logger().info(
            "경로 주변 동적 장애물 잔상 제거 완료. "
            f"corridor와 겹친 component={corridor_components}, "
            f"제거 component={removed_components}, 제거 cell={removed_cells}, "
            f"제거 면적={removed_area_m2:.3f} m^2"
        )

        return cleaned.astype(bool)

    def apply_morphological_closing(self, obstacle_map, radius_m):
        """Close small gaps in the obstacle map after transient-obstacle cleanup."""
        if radius_m <= 0.0:
            return obstacle_map

        radius_cells = max(1, int(round(radius_m / self.resolution)))
        kernel_size = 2 * radius_cells + 1

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

        src = obstacle_map.astype(np.uint8) * 255
        closed = cv2.morphologyEx(src, cv2.MORPH_CLOSE, kernel)

        self.get_logger().info(f"장애물 closing 적용 완료. 반경={radius_m:.2f} m, 커널 크기={kernel_size}x{kernel_size}")
        return closed > 0

    def remove_small_obstacle_components(self, obstacle_map, min_area):
        """Remove isolated occupied components below the configured area limit."""
        src = obstacle_map.astype(np.uint8)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(src, connectivity=8)

        cleaned = np.zeros_like(src)
        for label in range(1, num_labels):
            area = stats[label, cv2.CC_STAT_AREA]
            if area >= min_area:
                cleaned[labels == label] = 1

        removed = int(np.sum(src) - np.sum(cleaned))
        self.get_logger().info(f"작은 장애물 성분 제거 완료. 최소 면적={min_area}, 제거된 cell 수={removed}")
        return cleaned.astype(bool)

    def compute_distance_map(self, obstacle_map):
        """Measure metric clearance from each free cell to the nearest obstacle."""
        free_uint8 = np.logical_not(obstacle_map).astype(np.uint8)

        dist_cells = cv2.distanceTransform(free_uint8, distanceType=cv2.DIST_L2, maskSize=5)

        dist_m = dist_cells * self.resolution
        return dist_m

    def load_raw_path_csv(self, csv_path):
        """Read supported recorded-path CSV layouts into a NumPy array."""
        with open(csv_path, "r") as f:
            first_line = f.readline().strip()

        has_header = any(ch.isalpha() for ch in first_line)
        points = []

        if has_header:
            with open(csv_path, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    x = self.get_first_existing_float(row, ["x", "pose_x", "position_x", "base_link_x"])
                    y = self.get_first_existing_float(row, ["y", "pose_y", "position_y", "base_link_y"])
                    if x is not None and y is not None:
                        points.append([x, y])
        else:
            data = np.loadtxt(csv_path, delimiter=",")
            if data.ndim == 1:
                data = data.reshape(1, -1)
            for row in data:
                if len(row) >= 4:
                    x, y = row[1], row[2]
                elif len(row) >= 2:
                    x, y = row[0], row[1]
                else:
                    continue
                points.append([float(x), float(y)])

        if len(points) < 2:
            raise RuntimeError("raw_path.csv에는 최소 2개 이상의 유효한 점이 필요합니다.")

        return np.array(points, dtype=np.float64)

    def get_first_existing_float(self, row, keys):
        """Return the first valid numeric value among the candidate row keys."""
        for key in keys:
            if key in row and row[key] != "":
                return float(row[key])
        return None

    def save_safe_path_csv(self, csv_path, xy, yaw):
        """Write the corrected path in the existing safe-path CSV format."""
        output_dir = os.path.dirname(csv_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "yaw_rad"])
            for p, yaw_value in zip(xy, yaw):
                writer.writerow([f"{p[0]:.15f}", f"{p[1]:.15f}", f"{yaw_value:.15f}"])

    def remove_near_duplicate_points(self, xy, min_gap):
        """Drop recorded points closer than the minimum distance."""
        filtered = [xy[0]]
        for p in xy[1:]:
            if np.linalg.norm(p - filtered[-1]) >= min_gap:
                filtered.append(p)
        return np.array(filtered, dtype=np.float64)

    def resample_path(self, xy, interval):
        """Interpolate path points at the configured metric interval."""
        if len(xy) < 2:
            return xy

        segment_lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        cumulative = np.insert(np.cumsum(segment_lengths), 0, 0.0)
        total_length = cumulative[-1]

        if total_length < interval:
            return xy

        sample_s = np.arange(0.0, total_length, interval)
        if sample_s[-1] < total_length:
            sample_s = np.append(sample_s, total_length)

        resampled = []
        for s in sample_s:
            # Stop at or inside the target distance.
            idx = np.searchsorted(cumulative, s) - 1
            idx = max(0, min(idx, len(xy) - 2))

            s0 = cumulative[idx]
            s1 = cumulative[idx + 1]

            alpha = 0.0 if (s1 - s0) < 1e-9 else (s - s0) / (s1 - s0)

            p = (1.0 - alpha) * xy[idx] + alpha * xy[idx + 1]
            resampled.append(p)

        return np.array(resampled, dtype=np.float64)

    def compute_cumulative_distance(self, xy):
        """Measure cumulative distance along the path."""
        if len(xy) == 0:
            return np.array([], dtype=np.float64)
        if len(xy) == 1:
            return np.zeros(1, dtype=np.float64)

        segment_lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        return np.insert(np.cumsum(segment_lengths), 0, 0.0)

    def correct_path_points(self, raw_xy):
        """Choose collision-free lateral candidates while preserving path continuity."""
        safe_points = []
        stats = {
            "raw_out_of_map": 0, "raw_obstacle_or_unknown": 0, "raw_low_clearance": 0, "raw_safe": 0,
            "no_candidate": 0, "skipped": 0, "used_raw_fallback": 0, "selected_candidate": 0,
            "preserved_start": 0, "preserved_goal": 0,
        }

        cumulative_s = self.compute_cumulative_distance(raw_xy)
        total_s = float(cumulative_s[-1]) if len(cumulative_s) > 0 else 0.0

        for i, raw_p in enumerate(raw_xy):
            raw_status, raw_clearance = self.classify_point(raw_p)
            stats[f"raw_{raw_status.lower()}"] = stats.get(f"raw_{raw_status.lower()}", 0) + 1

            # Do not shift endpoints used for start and goal pose checks.
            # Clean LiDAR remnants before endpoint clearance checks.
            if cumulative_s[i] <= self.preserve_start_distance:
                safe_points.append(raw_p)
                stats["preserved_start"] += 1
                continue

            if (total_s - cumulative_s[i]) <= self.preserve_goal_distance:
                safe_points.append(raw_p)
                stats["preserved_goal"] += 1
                continue

            tangent = self.get_tangent(raw_xy, i)
            normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)

            candidates = self.generate_lateral_candidates(raw_p, normal)
            valid_candidates = []

            for c in candidates:
                if not self.is_pose_safe(c): continue
                if len(safe_points) > 0 and not self.is_segment_safe(safe_points[-1], c): continue

                score = self.score_candidate(candidate=c, raw_point=raw_p, previous_safe=safe_points[-1] if safe_points else None)
                valid_candidates.append((score, c))

            if not valid_candidates:
                fallback_candidates = self.generate_radius_candidates(raw_p)
                for c in fallback_candidates:
                    if not self.is_pose_safe(c): continue
                    if len(safe_points) > 0 and not self.is_segment_safe(safe_points[-1], c): continue
                    score = self.score_candidate(candidate=c, raw_point=raw_p, previous_safe=safe_points[-1] if safe_points else None)
                    valid_candidates.append((score, c))

            if valid_candidates:
                valid_candidates.sort(key=lambda x: x[0], reverse=True)
                best = valid_candidates[0][1]
                safe_points.append(best)
                stats["selected_candidate"] += 1
            else:
                stats["no_candidate"] += 1
                can_use_raw = False

                # Retain the original point only if it and its incoming segment remain
                # collision-free.
                if raw_status == "SAFE":
                    can_use_raw = True if len(safe_points) == 0 else self.is_segment_safe(safe_points[-1], raw_p)

                if can_use_raw:
                    safe_points.append(raw_p)
                    stats["used_raw_fallback"] += 1
                    self.get_logger().warn(f"[후보 없음 - 원본 점 사용] index={i}, raw=({raw_p[0]:.3f}, {raw_p[1]:.3f}), 원본 상태={self.korean_status(raw_status)}, 이격 거리={raw_clearance:.3f} m")
                else:
                    # Skip a point when no collision-free candidate remains.
                    stats["skipped"] += 1
                    self.get_logger().warn(f"[점 제거] index={i}, raw=({raw_p[0]:.3f}, {raw_p[1]:.3f}), 원본 상태={self.korean_status(raw_status)}, 사유=유효한 후보점 없음")

        if len(safe_points) < 2:
            raise RuntimeError("safe path 생성 실패: 유효한 safe point가 2개 미만입니다.")

        return np.array(safe_points, dtype=np.float64), stats

    def get_tangent(self, xy, i):
        """Estimate the forward tangent with a central difference."""
        if i == 0:
            v = xy[1] - xy[0]
        elif i == len(xy) - 1:
            v = xy[-1] - xy[-2]
        else:
            v = xy[i + 1] - xy[i - 1]

        norm = np.linalg.norm(v)
        if norm < 1e-9:
            return np.array([1.0, 0.0], dtype=np.float64)
        return v / norm

    def generate_lateral_candidates(self, raw_p, normal):
        """Sample candidate points along the path normal."""
        offsets = np.arange(-self.max_lateral_shift, self.max_lateral_shift + self.lateral_sample_step * 0.5, self.lateral_sample_step)
        offsets = np.unique(np.append(offsets, 0.0))

        candidates = []
        for offset in offsets:
            c = raw_p + offset * normal
            candidates.append(c)
        return candidates

    def generate_radius_candidates(self, raw_p):
        """Sample fallback candidates within the search radius."""
        candidates = []
        radius = self.max_lateral_shift
        step = self.radius_search_step

        xs = np.arange(-radius, radius + step * 0.5, step)
        ys = np.arange(-radius, radius + step * 0.5, step)

        for dx in xs:
            for dy in ys:
                if dx * dx + dy * dy > radius * radius: continue
                c = raw_p + np.array([dx, dy], dtype=np.float64)
                candidates.append(c)
        return candidates

    def score_candidate(self, candidate, raw_point, previous_safe):
        """Weight clearance, original-path deviation, and step smoothness."""
        clearance = self.get_clearance(candidate)
        clearance_score = min(clearance, self.clearance_score_cap)

        deviation = np.linalg.norm(candidate - raw_point)

        if previous_safe is None:
            smooth_penalty = 0.0
        else:
            expected_step = self.resample_interval
            actual_step = np.linalg.norm(candidate - previous_safe)
            smooth_penalty = abs(actual_step - expected_step)

        score = (self.w_clearance * clearance_score) - (self.w_deviation * deviation) - (self.w_smooth * smooth_penalty)
        return score

    def classify_point(self, p):
        """Classify a world point by map bounds, obstacles, and clearance."""
        row_col = self.world_to_map(p[0], p[1])
        if row_col is None:
            return "OUT_OF_MAP", -1.0

        row, col = row_col
        clearance = float(self.distance_map[row, col])

        if self.obstacle_map[row, col]:
            return "OBSTACLE_OR_UNKNOWN", clearance

        if clearance < self.min_clearance:
            return "LOW_CLEARANCE", clearance

        return "SAFE", clearance

    def korean_status(self, status):
        """Translate internal status codes for Korean user-facing logs."""
        mapping = {"OUT_OF_MAP": "맵 범위 밖", "OBSTACLE_OR_UNKNOWN": "장애물 또는 unknown 영역", "LOW_CLEARANCE": "이격 거리 부족", "SAFE": "안전"}
        return mapping.get(status, status)

    def is_pose_safe(self, p):
        """Check whether a point meets the clearance requirement."""
        status, _ = self.classify_point(p)
        return status == "SAFE"

    def is_segment_safe(self, p0, p1):
        """Sample an entire segment densely enough to detect map collisions."""
        dist = np.linalg.norm(p1 - p0)
        if dist < 1e-9:
            return self.is_pose_safe(p0)

        step = max(self.resolution * 0.5, 0.02)
        num = max(2, int(math.ceil(dist / step)))

        for i in range(num + 1):
            alpha = i / num
            p = (1.0 - alpha) * p0 + alpha * p1
            if not self.is_pose_safe(p):
                return False
        return True

    def get_clearance(self, p):
        """Return metric clearance for a world-coordinate point."""
        row_col = self.world_to_map(p[0], p[1])
        if row_col is None: return -1.0
        return float(self.distance_map[row_col[0], row_col[1]])

    def validate_final_path(self, xy):
        """Check all corrected points and segments for collisions."""
        invalid_count = 0
        for p in xy:
            if not self.is_pose_safe(p): invalid_count += 1
        for i in range(len(xy) - 1):
            if not self.is_segment_safe(xy[i], xy[i + 1]): invalid_count += 1
        return invalid_count

    def smooth_path_with_safety_check(self, xy):
        """Smooth interior points only when the result remains collision-free."""
        window = self.smooth_window_size
        if window < 3 or len(xy) < window:
            return xy

        half = window // 2
        smoothed = xy.copy()
        cumulative_s = self.compute_cumulative_distance(xy)
        total_s = float(cumulative_s[-1]) if len(cumulative_s) > 0 else 0.0

        for i in range(half, len(xy) - half):
            # Smoothing must not move protected start and goal segments.
            if cumulative_s[i] <= self.preserve_start_distance:
                continue
            if (total_s - cumulative_s[i]) <= self.preserve_goal_distance:
                continue

            q = np.mean(xy[i - half : i + half + 1], axis=0)

            # Accept a smoothed point only after clearance checks.
            if not self.is_pose_safe(q): continue
            if i > 0 and not self.is_segment_safe(smoothed[i - 1], q): continue
            if i < len(xy) - 1 and not self.is_segment_safe(q, xy[i + 1]): continue

            smoothed[i] = q

        return smoothed

    def compute_yaw_from_path(self, xy):
        """Compute heading angles from consecutive 2D path points."""
        yaw = []
        for i in range(len(xy)):
            if len(xy) == 1:
                yaw.append(0.0)
                continue

            if i == len(xy) - 1:
                dx = xy[i][0] - xy[i - 1][0]
                dy = xy[i][1] - xy[i - 1][1]
            else:
                dx = xy[i + 1][0] - xy[i][0]
                dy = xy[i + 1][1] - xy[i][1]

            yaw.append(math.atan2(dy, dx))
        return np.array(yaw, dtype=np.float64)

    def world_to_map(self, x, y):
        """Convert world coordinates to image rows and columns, reversing the image Y axis."""
        origin_x = self.origin[0]
        origin_y = self.origin[1]

        col = int(math.floor((x - origin_x) / self.resolution))
        row_from_bottom = int(math.floor((y - origin_y) / self.resolution))

        # Image rows grow downward while ROS map coordinates grow upward.
        # Flip the row index when mapping world coordinates to image cells.
        row = self.height - 1 - row_from_bottom

        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return None

        return row, col

    def publish_safe_path_once(self):
        """Publish the generated path as a ROS navigation message."""
        if self.safe_path_xy is None or self.safe_path_yaw is None:
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for p, yaw in zip(self.safe_path_xy, self.safe_path_yaw):
            pose = PoseStamped()
            pose.header = msg.header

            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            pose.pose.position.z = 0.0

            qx, qy, qz, qw = self.yaw_to_quaternion(float(yaw))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info("/safe_path 토픽을 1회 발행했습니다.")

    def yaw_to_quaternion(self, yaw):
        """Convert a planar yaw angle to a ROS quaternion."""
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def print_generation_statistics(self, raw_count, filtered_count, resampled_count, corrected_count, final_count, invalid_count, stats):
        """Report correction and safety-check statistics."""
        self.get_logger().info("========== Safe Path 생성 통계 ==========")
        self.get_logger().info(f"원본 경로 점 개수              : {raw_count}")
        self.get_logger().info(f"중복 제거 후 점 개수           : {filtered_count}")
        self.get_logger().info(f"재샘플링 후 점 개수            : {resampled_count}")
        self.get_logger().info(f"보정된 safe point 개수         : {corrected_count}")
        self.get_logger().info(f"최종 safe_path 점 개수         : {final_count}")
        self.get_logger().info("----------------------------------------")
        self.get_logger().info(f"맵 범위 밖 원본 점 개수        : {stats.get('raw_out_of_map', 0)}")
        self.get_logger().info(f"장애물/unknown 위 원본 점 개수 : {stats.get('raw_obstacle_or_unknown', 0)}")
        self.get_logger().info(f"이격 거리 부족 원본 점 개수    : {stats.get('raw_low_clearance', 0)}")
        self.get_logger().info(f"안전한 원본 점 개수            : {stats.get('raw_safe', 0)}")
        self.get_logger().info("----------------------------------------")
        self.get_logger().info(f"시작 구간 보존 점 개수         : {stats.get('preserved_start', 0)}")
        self.get_logger().info(f"도착 구간 보존 점 개수         : {stats.get('preserved_goal', 0)}")
        self.get_logger().info(f"후보점 선택 성공 개수          : {stats.get('selected_candidate', 0)}")
        self.get_logger().info(f"후보점 없음 개수               : {stats.get('no_candidate', 0)}")
        self.get_logger().info(f"원본 점 fallback 사용 개수     : {stats.get('used_raw_fallback', 0)}")
        self.get_logger().info(f"제거된 점 개수                 : {stats.get('skipped', 0)}")
        self.get_logger().info(f"최종 안전성 실패 개수          : {invalid_count}")
        self.get_logger().info("========================================")

        if stats.get("raw_out_of_map", 0) > 0:
            self.get_logger().warn("일부 원본 경로점이 맵 범위 밖에 있습니다. 원본 데이터의 map 프레임 일치 여부를 파악하십시오.")
        if stats.get("raw_low_clearance", 0) > 0:
            self.get_logger().warn("일부 원본 경로점의 장애물 이격 거리가 협소합니다. robot_radius 및 safety_margin 설정을 검토하십시오.")
        if stats.get("raw_obstacle_or_unknown", 0) > 0:
            self.get_logger().warn("일부 원본 경로점이 장애물 내부 혹은 미지 영역에 침범해 있습니다. 지도 매핑 상태 및 잔상을 확인하십시오.")


def main(args=None):
    """Initialize ROS, generate a safe path, and release the node."""
    rclpy.init(args=args)
    node = None

    try:
        node = SafePathGeneratorNode()

        node.publish_safe_path_once()

        # Spin briefly so DDS can publish the transient-local path.
        rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().info("safe path 생성 작업이 완료되었습니다. 노드를 자동 종료합니다.")

    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()