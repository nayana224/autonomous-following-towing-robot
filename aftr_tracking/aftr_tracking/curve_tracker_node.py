# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Run conservative target-locking on top of the proven follower controls."""

import json
import math
import traceback

import numpy as np
import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from aftr_tracking.target_lock import RobustTargetLock
from aftr_tracking.target_lock import TargetLockConfig
from aftr_tracking.tracker_node import NearestPersonFollower


class CurvedMotionPersonFollower(NearestPersonFollower):
    """Preserve existing detection and control while hardening target identity."""

    def __init__(self):
        """Initialize robust association, diagnostics, and tracking-frame TF."""
        super().__init__()

        # Keep the established following distance used by the current robot.
        self.d_ref = 0.47
        self.d_stop_margin = 0.04
        self.follow_fov_deg = float(
            self.declare_parameter("robust_follow_fov_deg", 105.0).value
        )
        self.follow_fov_rad = math.radians(self.follow_fov_deg)
        self.single_leg_fallback_enabled = bool(
            self.declare_parameter("single_leg_fallback_enabled", True).value
        )
        self.single_leg_measurement_weight = float(
            self.declare_parameter("single_leg_measurement_weight", 0.35).value
        )
        self.merged_leg_spread_max = float(
            self.declare_parameter("merged_leg_spread_max", 0.35).value
        )
        self.detection_source = "PAIRED_LEGS"
        self.tracking_linear_scale = 1.0

        self.tracking_frame = str(
            self.declare_parameter("tracking_frame", "odom").value
        )
        config = TargetLockConfig(
            search_confirm_frames=int(
                self.declare_parameter(
                    "search_confirm_frames",
                    self.required_frames,
                ).value
            ),
            search_match_distance_m=float(
                self.declare_parameter("search_match_distance_m", 0.45).value
            ),
            search_max_missed_frames=int(
                self.declare_parameter("search_max_missed_frames", 3).value
            ),
            match_gate_m=float(
                self.declare_parameter(
                    "target_match_gate_m",
                    self.follow_match_threshold,
                ).value
            ),
            ambiguity_proximity_m=float(
                self.declare_parameter("ambiguity_proximity_m", 0.65).value
            ),
            ambiguity_score_margin_m=float(
                self.declare_parameter("ambiguity_score_margin_m", 0.12).value
            ),
            reacquire_confirm_frames=int(
                self.declare_parameter("reacquire_confirm_frames", 2).value
            ),
            prediction_hold_frames=int(
                self.declare_parameter("prediction_hold_frames", 10).value
            ),
            ambiguous_prediction_hold_frames=int(
                self.declare_parameter(
                    "ambiguous_prediction_hold_frames",
                    2,
                ).value
            ),
            max_missed_frames=int(
                self.declare_parameter(
                    "target_max_missed_frames",
                    self.max_lost_frames,
                ).value
            ),
            max_ambiguous_frames=int(
                self.declare_parameter("target_max_ambiguous_frames", 50).value
            ),
            position_alpha=float(
                self.declare_parameter("target_position_alpha", 0.65).value
            ),
            velocity_alpha=float(
                self.declare_parameter("target_velocity_alpha", 0.20).value
            ),
            max_target_speed_mps=float(
                self.declare_parameter("max_target_speed_mps", 2.0).value
            ),
        )
        self.target_lock = RobustTargetLock(config)
        self.tracking_tf_buffer = Buffer()
        self.tracking_tf_listener = TransformListener(
            self.tracking_tf_buffer,
            self,
        )
        self.tracking_detail_pub = self.create_publisher(
            String,
            "/tracking_detail",
            10,
        )
        self.last_tracking_state = "SEARCH"
        self.last_tf_warning_sec = -10.0

        self.get_logger().info(
            "강건한 작업자 타겟 잠금이 활성화되었습니다. "
            f"tracking_frame={self.tracking_frame}, FOLLOW FOV=±{self.follow_fov_deg:.0f}°"
        )

    def detect_humans(self, clusters):
        """Use paired legs normally and one leg only during an active lock."""
        leg_candidates = self.detect_leg_candidates(clusters)
        paired_humans = self.pair_leg_candidates(leg_candidates)
        if paired_humans:
            self.detection_source = "PAIRED_LEGS"
            return paired_humans
        if (
            self.mode == "FOLLOW"
            and self.single_leg_fallback_enabled
        ):
            if not leg_candidates:
                leg_candidates = self.detect_leg_candidates(
                    clusters,
                    spread_max=self.merged_leg_spread_max,
                )
            if not leg_candidates:
                self.detection_source = "NONE"
                return []
            self.detection_source = "SINGLE_LEG"
            return leg_candidates
        self.detection_source = "NONE"
        return []

    def search_target(self, humans):
        """Lock only a spatially continuous worker candidate across frames."""
        local_candidates = [
            self._point_tuple(human)
            for human in humans
            if 0.3 <= math.hypot(float(human[0]), float(human[1]))
            <= self.detection_dist_limit
        ]
        transform = self._tracking_transform()
        if transform is None:
            self.candidate_count = 0
            self._publish_tracking_detail("TF_UNAVAILABLE", None, 0.0)
            return None

        tracking_candidates = [
            self._to_tracking_frame(point, transform)
            for point in local_candidates
        ]
        result = self.target_lock.search(
            tracking_candidates,
            self._now_sec(),
        )
        self.candidate_count = self.target_lock.candidate_frames
        self._publish_tracking_result(result)
        if result.position is None:
            return None

        local_target = self._to_laser_frame(result.position, transform)
        self.locked_target = np.asarray(local_target, dtype=np.float32)
        self.lost_frames = 0
        self.candidate_count = 0
        self.mode = "FOLLOW"
        return self.locked_target

    def update_locked_target(self, humans):
        """Update only the locked worker; stop while identity is ambiguous."""
        transform = self._tracking_transform()
        if transform is None:
            self.locked_target = None
            self._publish_tracking_detail("TF_UNAVAILABLE", None, 0.0)
            return None

        tracking_candidates = [
            self._to_tracking_frame(self._point_tuple(human), transform)
            for human in humans
        ]
        now_sec = self._now_sec()
        if self.detection_source == "SINGLE_LEG":
            predicted = self.target_lock.predicted_position(now_sec)
            if predicted is not None:
                weight = min(
                    1.0,
                    max(0.0, self.single_leg_measurement_weight),
                )
                tracking_candidates = [
                    (
                        weight * point[0] + (1.0 - weight) * predicted[0],
                        weight * point[1] + (1.0 - weight) * predicted[1],
                    )
                    for point in tracking_candidates
                ]
        result = self.target_lock.update(
            tracking_candidates,
            now_sec,
        )
        self._publish_tracking_result(result)

        if result.state == "LOST":
            self.locked_target = None
            self.candidate_count = 0
            self.lost_frames = self.max_lost_frames + 1
            self.mode = "SEARCH"
            return None

        self.lost_frames = self.target_lock.missed_frames
        if result.state == "LOCKED":
            self.tracking_linear_scale = (
                0.55 if self.detection_source == "SINGLE_LEG" else 1.0
            )
        elif result.state == "OCCLUDED":
            self.tracking_linear_scale = max(0.1, result.confidence)
        elif result.state == "AMBIGUOUS":
            self.tracking_linear_scale = 0.0
        else:
            self.tracking_linear_scale = 0.0
        if result.position is None:
            # A longer uncertain interval deliberately removes the control
            # target. Short gaps already returned a prediction above so normal
            # scan noise does not create a stop/reacquire cycle.
            self.locked_target = None
            return None

        local_target = self._to_laser_frame(result.position, transform)
        self.locked_target = np.asarray(local_target, dtype=np.float32)
        return self.locked_target

    def apply_slew(self, v_des, w_des, dt):
        """Keep angular correction active when linear speed becomes very small."""
        dt = max(float(dt), 1e-3)
        v_des = float(v_des) * min(
            1.0,
            max(0.0, self.tracking_linear_scale),
        )

        dv = float(v_des) - self.last_cmd_v
        dv_max = self.a_lin_follow * dt if dv > 0.0 else self.a_stop * dt
        v_cmd = self.last_cmd_v + max(-dv_max, min(dv_max, dv))

        dw = float(w_des) - self.last_cmd_w
        dw_max = self.a_ang_follow * dt if dw > 0.0 else self.a_ang_stop * dt
        w_cmd = self.last_cmd_w + max(-dw_max, min(dw_max, dw))

        v_cmd = max(0.0, min(self.v_max_follow, v_cmd))
        w_cmd = max(-self.w_max_follow, min(self.w_max_follow, w_cmd))

        # The legacy implementation zeroed angular velocity whenever v < 0.05.
        # That interrupted turns near the worker. Use only a small angular
        # deadband so a locked worker can still be followed through a curve.
        if abs(w_cmd) < 0.04:
            w_cmd = 0.0

        return float(v_cmd), float(w_cmd)

    def _tracking_transform(self):
        """Return the latest 2D transform from laser into the tracking frame."""
        try:
            transform = self.tracking_tf_buffer.lookup_transform(
                self.tracking_frame,
                self.frame_id,
                Time(),
            )
        except TransformException as exc:
            now_sec = self._now_sec()
            if now_sec - self.last_tf_warning_sec >= 2.0:
                self.get_logger().warning(
                    "작업자 추적 TF를 기다리는 중입니다: "
                    f"{self.tracking_frame} <- {self.frame_id}: {exc}"
                )
                self.last_tf_warning_sec = now_sec
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return float(translation.x), float(translation.y), yaw

    @staticmethod
    def _to_tracking_frame(point, transform):
        """Apply a planar laser-to-tracking-frame transform."""
        tx, ty, yaw = transform
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            tx + cos_yaw * point[0] - sin_yaw * point[1],
            ty + sin_yaw * point[0] + cos_yaw * point[1],
        )

    @staticmethod
    def _to_laser_frame(point, transform):
        """Apply the inverse planar transform for the existing controller."""
        tx, ty, yaw = transform
        dx = point[0] - tx
        dy = point[1] - ty
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
        )

    def _publish_tracking_result(self, result):
        """Publish a target-lock result without changing the public state topic."""
        self._publish_tracking_detail(
            result.state,
            result.target_id,
            result.confidence,
        )

    def _publish_tracking_detail(self, state, target_id, confidence):
        """Publish compact JSON diagnostics for tests and field tuning."""
        message = String()
        message.data = json.dumps(
            {
                "state": str(state),
                "target_id": target_id,
                "confidence": round(float(confidence), 3),
                "missed_frames": int(self.target_lock.missed_frames),
                "ambiguous_frames": int(self.target_lock.ambiguous_frames),
                "tracking_frame": self.tracking_frame,
                "detection_source": self.detection_source,
                "linear_scale": round(float(self.tracking_linear_scale), 3),
            },
            separators=(",", ":"),
        )
        self.tracking_detail_pub.publish(message)
        if state != self.last_tracking_state:
            self.get_logger().info(
                f"작업자 추적 상태 변경: {self.last_tracking_state} -> {state}"
            )
            self.last_tracking_state = str(state)

    def _now_sec(self):
        """Return current ROS time as floating-point seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _point_tuple(point):
        """Convert a NumPy-like point into a typed two-dimensional tuple."""
        return float(point[0]), float(point[1])


def main():
    """Run the robust worker follower."""
    node = None
    try:
        rclpy.init()
        node = CurvedMotionPersonFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("person_follower interrupted by SIGINT")
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"person_follower crashed: {exc}")
        else:
            print(f"person_follower crashed before startup completed: {exc}")
        traceback.print_exc()
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
            except Exception as exc:
                node.get_logger().warning(
                    f"failed to destroy person_follower node cleanly: {exc}"
                )

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
            except Exception as exc:
                if node is not None:
                    node.get_logger().warning(
                        f"failed to shut down rclpy cleanly: {exc}"
                    )
                else:
                    print(f"failed to shut down rclpy cleanly: {exc}")


if __name__ == "__main__":
    main()
