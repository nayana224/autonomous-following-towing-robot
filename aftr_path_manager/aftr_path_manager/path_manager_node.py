# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Record odometry paths and save the last robot pose."""

import csv
import json
import math
import os
import subprocess
import sys
import time
import traceback

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
import rclpy
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from aftr_path_manager.bidirectional_corner_fillet import (
    apply_bidirectional_corner_fillets,
)
from aftr_path_manager.follow_runtime import build_remaining_follow_poses
from aftr_path_manager.follow_runtime import compute_retry_delay_seconds
from aftr_path_manager.follow_runtime import estimate_remaining_follow_distance
from aftr_path_manager.follow_runtime import FollowPathRuntimeState
from aftr_path_manager.path_geometry import yaw_from_quaternion
from aftr_path_manager.path_geometry import yaw_to_quaternion
from aftr_path_manager.path_geometry import normalize_angle
from aftr_path_manager.recording import filter_global_frame_poses
from aftr_path_manager.recording import should_record_pose
from aftr_path_manager.recording import simplify_recorded_path
from aftr_path_manager.path_safety_cleanup import build_grid_snapshot
from aftr_path_manager.path_safety_cleanup import cleanup_path_points
from aftr_path_manager.path_safety_cleanup import is_pose_safe
from aftr_path_manager.path_storage import load_path_csv
from aftr_path_manager.path_storage import load_pose_yaml
from aftr_path_manager.path_storage import save_path_csv
from aftr_path_manager.path_storage import save_pose_yaml
from aftr_path_manager.path_types import RecordedPose


class PathManagerNode(Node):
    """Manage path recording, saved-pose handling, and Nav2 path replay."""

    def __init__(self):
        """Initialize parameters, runtime state, and ROS interfaces."""
        super().__init__("path_manager")
        self._declare_parameters()
        self._initialize_runtime_state()
        self._create_ros_interfaces()
        self._register_services()

        if self.auto_start:
            self.start_recording(reset=True)

        self.get_logger().info(
            "path_manager ready "
            f"(csv_file={self.csv_file}, "
            f"safe_csv_file={self.safe_csv_file}, "
            f"pose_file={self.pose_file})"
        )

    # Initialization

    def _declare_parameters(self):
        """Declare ROS parameters used by recording and replay workflows."""
        self.csv_file = self.declare_parameter(
            "csv_file",
            "/data/paths/recorded_path.csv",
        ).value
        self.safe_csv_file = self.declare_parameter(
            "safe_csv_file",
            "/data/paths/safe_path.csv",
        ).value
        self.safe_path_generator_timeout_sec = self.declare_parameter(
            "safe_path_generator_timeout_sec",
            60.0,
        ).value
        self.corner_turn_output_csv = self.declare_parameter(
            "corner_turn_output_csv",
            "/data/paths/corner_turn_path.csv",
        ).value
        self.corner_turn_map_yaml = self.declare_parameter(
            "corner_turn_map_yaml",
            "/data/maps/mdbot_map.yaml",
        ).value
        self.corner_turn_generator_timeout_sec = self.declare_parameter(
            "corner_turn_generator_timeout_sec",
            30.0,
        ).value
        self.pose_file = self.declare_parameter(
            "pose_file",
            "/data/poses/last_pose.yaml",
        ).value
        self.global_frame = self.declare_parameter("global_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_footprint").value
        self.allow_odom_fallback = self.declare_parameter(
            "allow_odom_fallback",
            True,
        ).value
        self.record_global_frame_only = self.declare_parameter(
            "record_global_frame_only",
            True,
        ).value
        self.map_pose_ready_timeout_sec = self.declare_parameter(
            "map_pose_ready_timeout_sec",
            3.0,
        ).value

        # The default point spacing is intentionally small so the saved path
        # preserves corridor and corner geometry before simplification.
        self.min_distance_m = self.declare_parameter("min_distance_m", 0.05).value
        self.min_yaw_delta_rad = self.declare_parameter(
            "min_yaw_delta_rad",
            math.radians(5.0),
        ).value
        self.min_time_delta_s = self.declare_parameter("min_time_delta_s", 0.05).value
        self.simplify_tolerance_m = self.declare_parameter(
            "simplify_tolerance_m",
            0.03,
        ).value
        self.initial_pose_publish_count = self.declare_parameter(
            "initial_pose_publish_count",
            5,
        ).value
        self.initial_pose_publish_interval_s = self.declare_parameter(
            "initial_pose_publish_interval_s",
            0.2,
        ).value
        # FollowPath may abort on temporary obstacles or progress failures.
        # The retry settings keep replay moving toward the final stored pose.
        self.follow_retry_enabled = self.declare_parameter(
            "follow_retry_enabled",
            True,
        ).value
        self.follow_retry_delay_s = self.declare_parameter(
            "follow_retry_delay_s",
            2.0,
        ).value
        self.follow_max_retries = self.declare_parameter(
            "follow_max_retries",
            0,
        ).value
        self.follow_retry_max_delay_s = self.declare_parameter(
            "follow_retry_max_delay_s",
            8.0,
        ).value
        self.follow_resume_lookback_points = self.declare_parameter(
            "follow_resume_lookback_points",
            1,
        ).value
        self.follow_final_tolerance_m = self.declare_parameter(
            "follow_final_tolerance_m",
            0.25,
        ).value
        # Start alignment is diagnostic only. The node logs offset details
        # but lets Nav2 decide how to connect to the path.
        self.max_start_heading_error_rad = self.declare_parameter(
            "max_start_heading_error_rad",
            math.radians(35.0),
        ).value
        self.max_start_path_distance_m = self.declare_parameter(
            "max_start_path_distance_m",
            0.45,
        ).value
        self.min_start_segment_length_m = self.declare_parameter(
            "min_start_segment_length_m",
            0.20,
        ).value
        self.follow_path_interpolation_step_m = self.declare_parameter(
            "follow_path_interpolation_step_m",
            0.08,
        ).value
        self.follow_path_safety_cleanup_enabled = self.declare_parameter(
            "follow_path_safety_cleanup_enabled",
            True,
        ).value
        self.follow_path_safety_clearance_m = self.declare_parameter(
            "follow_path_safety_clearance_m",
            0.25,
        ).value
        self.follow_path_max_lateral_shift_m = self.declare_parameter(
            "follow_path_max_lateral_shift_m",
            0.35,
        ).value
        self.follow_path_preferred_clearance_m = self.declare_parameter(
            "follow_path_preferred_clearance_m",
            0.75,
        ).value
        self.follow_path_preferred_max_lateral_shift_m = self.declare_parameter(
            "follow_path_preferred_max_lateral_shift_m",
            0.35,
        ).value
        self.follow_path_lateral_sample_step_m = self.declare_parameter(
            "follow_path_lateral_sample_step_m",
            0.05,
        ).value
        self.follow_path_preserve_start_distance_m = self.declare_parameter(
            "follow_path_preserve_start_distance_m",
            0.25,
        ).value
        self.follow_path_preserve_goal_distance_m = self.declare_parameter(
            "follow_path_preserve_goal_distance_m",
            0.25,
        ).value
        self.follow_path_unknown_is_occupied = self.declare_parameter(
            "follow_path_unknown_is_occupied",
            True,
        ).value
        self.follow_path_correction_mode = self.declare_parameter(
            "follow_path_correction_mode",
            "corner_only",
        ).value
        self.follow_path_centering_enabled = self.declare_parameter(
            "follow_path_centering_enabled",
            True,
        ).value
        self.follow_path_centering_corridor_max_width_m = self.declare_parameter(
            "follow_path_centering_corridor_max_width_m",
            2.20,
        ).value
        self.follow_path_centering_corner_heading_deg = self.declare_parameter(
            "follow_path_centering_corner_heading_deg",
            18.0,
        ).value
        self.follow_path_centering_corner_shift_scale = self.declare_parameter(
            "follow_path_centering_corner_shift_scale",
            0.30,
        ).value
        self.follow_path_centering_smoothing_window = self.declare_parameter(
            "follow_path_centering_smoothing_window",
            3,
        ).value
        self.follow_path_centering_transition_distance_m = self.declare_parameter(
            "follow_path_centering_transition_distance_m",
            0.45,
        ).value
        self.follow_path_centering_max_shift_delta_m = self.declare_parameter(
            "follow_path_centering_max_shift_delta_m",
            0.08,
        ).value
        self.follow_path_centering_shift_smoothing_window = self.declare_parameter(
            "follow_path_centering_shift_smoothing_window",
            2,
        ).value
        self.follow_path_centering_outlier_threshold_m = self.declare_parameter(
            "follow_path_centering_outlier_threshold_m",
            0.10,
        ).value
        self.follow_path_pre_simplify_enabled = self.declare_parameter(
            "follow_path_pre_simplify_enabled",
            True,
        ).value
        self.follow_path_pre_simplify_min_spacing_m = self.declare_parameter(
            "follow_path_pre_simplify_min_spacing_m",
            0.10,
        ).value
        self.follow_path_pre_simplify_heading_deg = self.declare_parameter(
            "follow_path_pre_simplify_heading_deg",
            8.0,
        ).value
        self.follow_path_corner_clearance_delta_threshold_m = self.declare_parameter(
            "follow_path_corner_clearance_delta_threshold_m",
            0.18,
        ).value
        self.follow_path_corner_shift_delta_threshold_m = self.declare_parameter(
            "follow_path_corner_shift_delta_threshold_m",
            0.07,
        ).value
        self.follow_path_corner_segment_padding_points = self.declare_parameter(
            "follow_path_corner_segment_padding_points",
            2,
        ).value
        self.follow_path_corner_curve_resample_step_m = self.declare_parameter(
            "follow_path_corner_curve_resample_step_m",
            0.06,
        ).value
        self.follow_path_corner_entry_transition_distance_m = self.declare_parameter(
            "follow_path_corner_entry_transition_distance_m",
            1.80,
        ).value
        self.follow_path_corner_exit_transition_distance_m = self.declare_parameter(
            "follow_path_corner_exit_transition_distance_m",
            1.80,
        ).value
        self.follow_path_corner_smoothing_strength = self.declare_parameter(
            "follow_path_corner_smoothing_strength",
            0.65,
        ).value
        self.follow_path_corner_max_deviation_m = self.declare_parameter(
            "follow_path_corner_max_deviation_m",
            0.20,
        ).value
        self.follow_path_bidirectional_corner_enabled = self.declare_parameter(
            "follow_path_bidirectional_corner_enabled",
            True,
        ).value
        self.follow_path_bidirectional_corner_heading_deg = self.declare_parameter(
            "follow_path_bidirectional_corner_heading_deg",
            20.0,
        ).value
        self.follow_path_bidirectional_corner_radius_m = self.declare_parameter(
            "follow_path_bidirectional_corner_radius_m",
            1.80,
        ).value
        self.follow_path_bidirectional_corner_max_deviation_m = (
            self.declare_parameter(
                "follow_path_bidirectional_corner_max_deviation_m",
                0.55,
            ).value
        )
        self.follow_path_bidirectional_corner_resample_step_m = (
            self.declare_parameter(
                "follow_path_bidirectional_corner_resample_step_m",
                0.05,
            ).value
        )
        self.follow_path_bidirectional_corner_minimum_radius_m = (
            self.declare_parameter(
                "follow_path_bidirectional_corner_minimum_radius_m",
                0.80,
            ).value
        )
        self.follow_path_bidirectional_corner_outward_overshoot_m = (
            self.declare_parameter(
                "follow_path_bidirectional_corner_outward_overshoot_m",
                0.0,
            ).value
        )
        self.auto_start = self.declare_parameter("auto_start", False).value

    def _initialize_runtime_state(self):
        """Initialize non-ROS runtime state used by services and callbacks."""
        self.recording = False
        self.saved_once = False
        self.follow_state = FollowPathRuntimeState()
        self.poses: list[RecordedPose] = []
        self.last_recorded_pose: RecordedPose | None = None
        self.latest_pose: RecordedPose | None = None
        self.latest_odom_pose: RecordedPose | None = None
        self.latest_map: OccupancyGrid | None = None
        self.record_skipped_no_pose = 0
        self.record_skipped_non_global = 0
        self.last_record_skip_reason = "idle"
        self.shutting_down = False

    def _create_ros_interfaces(self):
        """Create publishers, subscribers, timers, TF helpers, and actions."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            20,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10,
        )

    def _register_services(self):
        """Register Trigger services exposed by the path manager node."""
        self.create_service(Trigger, "~/start_record", self.start_record_callback)
        self.create_service(Trigger, "~/stop_record", self.stop_record_callback)
        self.create_service(
            Trigger,
            "~/check_record_ready",
            self.check_record_ready_callback,
        )
        self.create_service(Trigger, "~/save_pose", self.save_pose_callback)
        self.create_service(
            Trigger,
            "~/publish_initial_pose",
            self.publish_initial_pose_callback,
        )
        self.create_service(
            Trigger,
            "~/follow_saved_path",
            self.follow_saved_path_callback,
        )
        self.create_service(
            Trigger,
            "~/follow_saved_path_reverse",
            self.follow_saved_path_reverse_callback,
        )
        self.create_service(
            Trigger,
            "~/cancel_follow_path",
            self.cancel_follow_path_callback,
        )

    # Status publishing

    def publish_status(self):
        """Publish path manager status as JSON text."""
        msg = String()
        status = {
            "recording": self.recording,
            "following_path": self.follow_state.active,
            "follow_retry_count": self.follow_state.retry_count,
            "last_follow_event": self.follow_state.last_event,
            "remaining_distance_m": round(self.remaining_follow_distance(), 2),
            "points": len(self.poses),
            "recorded_points": len(self.poses),
            "map_points": len(self.valid_global_poses(self.poses)),
            "pose_frame": self.latest_pose.frame_id if self.latest_pose else "none",
            "record_skip_no_pose": self.record_skipped_no_pose,
            "record_skip_non_global": self.record_skipped_non_global,
            "record_event": self.last_record_skip_reason,
            "min_distance_m": round(float(self.min_distance_m), 3),
            "min_yaw_delta_deg": round(math.degrees(self.min_yaw_delta_rad), 1),
            "csv_file": self.csv_file,
            "safe_csv_file": self.safe_csv_file,
            "pose_file": self.pose_file,
        }
        msg.data = json.dumps(
            status,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_pub.publish(msg)

    # Service callbacks

    def build_trigger_response(self, response, success, message):
        """Fill a Trigger response object and return it."""
        response.success = bool(success)
        response.message = message
        return response

    def start_record_callback(self, request, response):
        """Start path recording and clear previous in-memory points."""
        started = self.start_recording(reset=True)
        message = (
            "path recording started"
            if started
            else "path recording could not start: waiting for map-frame pose"
        )
        return self.build_trigger_response(response, started, message)

    def stop_record_callback(self, request, response):
        """Stop path recording and save current data."""
        success, message = self.stop_recording(save=True)
        return self.build_trigger_response(response, success, message)

    def check_record_ready_callback(self, request, response):
        """Report whether map-frame robot pose is available for recording."""
        pose = self.wait_for_map_pose(float(self.map_pose_ready_timeout_sec))
        if pose is None:
            return self.build_trigger_response(
                response,
                False,
                "recording is not ready: waiting for map-frame pose",
            )

        self.latest_pose = pose
        return self.build_trigger_response(
            response,
            True,
            "recording is ready with map-frame pose",
        )

    def save_pose_callback(self, request, response):
        """Save the current map-frame robot pose."""
        pose, error_message = self.resolve_latest_global_pose()
        if pose is None:
            return self.build_trigger_response(response, False, error_message)

        self.save_pose(pose)
        return self.build_trigger_response(
            response,
            True,
            f"pose saved to {self.pose_file}",
        )

    def publish_initial_pose_callback(self, request, response):
        """Publish the saved latest pose to Nav2/AMCL as /initialpose."""
        self.reset_follow_status("idle")

        # Prefer the aligned saved pose because a later /odom callback may change the pose frame.
        pose = self.load_saved_pose()
        if pose is None:
            pose = self.latest_pose

        if pose is None:
            return self.build_trigger_response(
                response,
                False,
                "no pose has been received yet",
            )

        if pose.frame_id != self.global_frame:
            return self.build_trigger_response(
                response,
                False,
                "latest pose is not in the global frame: "
                f"{pose.frame_id}",
            )

        self.publish_initial_pose(pose)
        return self.build_trigger_response(response, True, "initial pose published")

    def follow_saved_path_callback(self, request, response):
        """Send the saved CSV path to Nav2 FollowPath."""
        return self.send_saved_path_to_nav2(response, reverse=False)

    def follow_saved_path_reverse_callback(self, request, response):
        """Send the saved CSV path to Nav2 FollowPath in reverse order."""
        return self.send_saved_path_to_nav2(response, reverse=True)

    def send_saved_path_to_nav2(self, response, reverse=False):
        """Send a saved path to Nav2 after choosing the path direction."""
        success, message = self.start_saved_path_following(reverse=reverse)
        self.publish_status()
        return self.build_trigger_response(response, success, message)

    def start_saved_path_following(self, reverse=False):
        """Start Nav2 FollowPath with the saved path in the requested direction."""
        if self.recording:
            message = "cannot follow path while recording"
            self.follow_state.last_event = message
            return False, message

        if self.follow_state.active:
            message = "a saved path is already being followed"
            self.follow_state.last_event = message
            return False, message

        generated, generation_message = self.generate_attached_safe_path()
        if not generated:
            self.follow_state.last_event = generation_message
            return False, generation_message

        poses = self.load_saved_path_poses(reverse=reverse)
        if len(poses) < 2:
            message = f"no valid saved path: {self.csv_file}"
            self.follow_state.last_event = message
            return False, message

        is_aligned, alignment_message = self.check_start_alignment(poses)
        if not is_aligned:
            self.follow_state.last_event = alignment_message
            self.get_logger().warn(alignment_message)
            return False, alignment_message

        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            message = "FollowPath action server is not available"
            self.follow_state.last_event = message
            return False, message

        # The two supplied generators are the authoritative replay pipeline:
        # safe-path cleanup first, then direction-specific wide turns. Do not
        # run the former cleanup or generic fillet over their output.
        poses = self.generate_attached_wide_turn_path(poses, reverse=reverse)
        if len(poses) < 2:
            message = "supplied wide-turn path generation failed"
            self.follow_state.last_event = message
            return False, message
        # Keep the short current-pose connector out of corner detection. It is
        # only an alignment bridge, not part of the recorded route geometry.
        poses = self.prepend_current_pose_to_follow_path(poses)
        poses = self.interpolate_follow_path(poses)
        poses = self.with_path_direction_yaw(poses)
        path_msg = self.poses_to_nav_path(poses)
        self.follow_state.active = True
        self.follow_state.cancel_requested = False
        self.follow_state.retry_count = 0
        self.follow_state.reverse = reverse
        self.follow_state.poses = poses
        self.follow_state.last_event = (
            "reverse path following started"
            if reverse
            else "forward path following started"
        )
        self.path_pub.publish(path_msg)

        # Send the navigation goal asynchronously so GUI and mode-manager callbacks remain
        # responsive.
        self.send_follow_path_goal(path_msg)

        direction = "reverse" if reverse else "forward"
        message = (
            f"sent {len(path_msg.poses)} {direction} poses to FollowPath"
        )
        return True, message

    def check_start_alignment(self, poses):
        """Log start offset diagnostics without blocking saved-path driving."""
        current_pose = self.current_robot_pose()
        if current_pose is None:
            return False, "current robot pose is unavailable; cannot start saved path"
        if current_pose.frame_id != self.global_frame:
            return (
                False,
                "current robot pose is not in the global frame: "
                f"{current_pose.frame_id}",
            )

        first_pose = poses[0]
        dx_to_start = first_pose.x - current_pose.x
        dy_to_start = first_pose.y - current_pose.y
        distance_to_path_start = math.hypot(dx_to_start, dy_to_start)
        if distance_to_path_start > float(self.max_start_path_distance_m):
            cos_yaw = math.cos(current_pose.yaw)
            sin_yaw = math.sin(current_pose.yaw)
            forward_offset = cos_yaw * dx_to_start + sin_yaw * dy_to_start
            left_offset = -sin_yaw * dx_to_start + cos_yaw * dy_to_start
            motion = "forward" if forward_offset >= 0.0 else "backward"
            joystick = "down" if forward_offset >= 0.0 else "up"
            self.get_logger().warn(
                "saved path start is far from current pose, but continuing: "
                f"(distance={distance_to_path_start:.2f} m, "
                f"forward={forward_offset:.2f} m, "
                f"left={left_offset:.2f} m, "
                f"move={motion}, joystick={joystick})"
            )

        start_segment = self.first_meaningful_path_segment(poses)
        if start_segment is None:
            self.get_logger().warn(
                "saved path start segment is short, but continuing"
            )
            return True, "path start alignment check skipped"

        path_yaw = math.atan2(
            start_segment[1].y - start_segment[0].y,
            start_segment[1].x - start_segment[0].x,
        )
        signed_heading_error = normalize_angle(path_yaw - current_pose.yaw)
        heading_error = abs(signed_heading_error)
        if heading_error > float(self.max_start_heading_error_rad):
            turn_direction = "left" if signed_heading_error > 0.0 else "right"
            self.get_logger().warn(
                "saved path start heading is large, but continuing: "
                f"{math.degrees(heading_error):.1f} deg, "
                f"turn={turn_direction}, "
                f"signed={math.degrees(signed_heading_error):.1f} deg"
            )

        return True, "path start alignment is acceptable"

    def first_meaningful_path_segment(self, poses):
        """Return the first segment long enough to define a safe start heading."""
        if len(poses) < 2:
            return None

        anchor = poses[0]
        min_length = max(0.03, float(self.min_start_segment_length_m))
        for next_pose in poses[1:]:
            segment_length = math.hypot(
                next_pose.x - anchor.x,
                next_pose.y - anchor.y,
            )
            if segment_length >= min_length:
                return anchor, next_pose
        return None

    def prepend_current_pose_to_follow_path(self, poses):
        """Start the FollowPath goal at the current robot pose when possible."""
        current_pose = self.current_robot_pose()
        if current_pose is None or current_pose.frame_id != self.global_frame:
            return poses

        first_pose = poses[0]
        distance_to_first = math.hypot(
            first_pose.x - current_pose.x,
            first_pose.y - current_pose.y,
        )
        if distance_to_first < 0.03:
            return poses

        # Start from the current aligned pose to avoid a large initial arc toward the wall.
        adjusted_poses = [
            RecordedPose(
                stamp_sec=current_pose.stamp_sec,
                frame_id=self.global_frame,
                x=current_pose.x,
                y=current_pose.y,
                yaw=current_pose.yaw,
            )
        ]
        adjusted_poses.extend(poses)
        return adjusted_poses

    def cancel_follow_path_callback(self, request, response):
        """Cancel the active Nav2 FollowPath goal if one exists."""
        success, message = self.cancel_active_follow_path()
        self.publish_status()
        return self.build_trigger_response(response, success, message)

    def cancel_active_follow_path(self):
        """Cancel the active Nav2 FollowPath goal if one is running."""
        if self.follow_state.goal_handle is None:
            self.reset_follow_status("idle")
            return True, "no active FollowPath goal"

        self.follow_state.cancel_requested = True
        self.destroy_follow_retry_timer()
        cancel_future = self.follow_state.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.follow_path_cancel_done_callback)
        self.follow_state.active = False
        message = "FollowPath cancel requested"
        self.follow_state.last_event = message
        return True, message

    # Recording and map callbacks

    def odom_callback(self, msg):
        """Store odometry while recording is enabled."""
        self.latest_odom_pose = self.pose_from_odom(msg)
        pose = self.current_map_pose()
        if pose is None:
            if self.recording:
                self.record_skipped_no_pose += 1
                self.last_record_skip_reason = "waiting for robot pose"
            return

        self.latest_pose = pose

        if not self.recording:
            return
        if not self.should_record(pose):
            return

        self.poses.append(pose)
        self.last_recorded_pose = pose

    def map_callback(self, msg):
        """Cache the latest occupancy grid used for replay path cleanup."""
        self.latest_map = msg
        self.last_record_skip_reason = "recording map-frame poses"

    # Pose lookup helpers

    def pose_from_odom(self, msg):
        """Convert an Odometry message to a compact 2D pose."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        return RecordedPose(
            stamp_sec=stamp_sec,
            frame_id=self.odom_frame,
            x=float(position.x),
            y=float(position.y),
            yaw=yaw,
        )

    def current_robot_pose(self):
        """Return the current robot pose in the global frame when available."""
        return self.current_map_pose()

    def current_map_pose(self):
        """Return only the map-frame robot pose."""
        return self.lookup_robot_pose_in_global_frame()

    def resolve_latest_global_pose(self):
        """Return the best available map-frame pose or an error message."""
        pose = self.wait_for_map_pose(float(self.map_pose_ready_timeout_sec))
        if pose is not None:
            self.latest_pose = pose
        else:
            pose = self.latest_pose

        if pose is None:
            return None, "no pose has been received yet"
        if pose.frame_id != self.global_frame:
            return (
                None,
                "latest pose is not in the global frame: "
                f"{pose.frame_id}",
            )
        return pose, ""

    def wait_for_map_pose(self, timeout_sec):
        """Wait briefly for a map-frame pose to become available."""
        timeout_sec = max(0.0, float(timeout_sec))
        deadline = time.monotonic() + timeout_sec
        while True:
            pose = self.current_map_pose()
            if pose is not None:
                return pose
            if timeout_sec <= 0.0 or time.monotonic() >= deadline:
                return None
            time.sleep(0.05)

    def lookup_robot_pose_in_global_frame(self):
        """Look up the current robot pose from TF in the global frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return RecordedPose(
            stamp_sec=stamp_sec,
            frame_id=self.global_frame,
            x=float(translation.x),
            y=float(translation.y),
            yaw=yaw_from_quaternion(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
        )

    def should_record(self, pose):
        """Return True when the new pose is far enough from the last point."""
        return should_record_pose(
            pose=pose,
            last_recorded_pose=self.last_recorded_pose,
            min_distance_m=float(self.min_distance_m),
            min_yaw_delta_rad=float(self.min_yaw_delta_rad),
            min_time_delta_s=float(self.min_time_delta_s),
        )

    # Recording workflow

    def start_recording(self, reset):
        """Enable recording, optionally clearing previous points."""
        if reset:
            self.poses.clear()
            self.last_recorded_pose = None
            self.saved_once = False
            self.record_skipped_no_pose = 0
            self.record_skipped_non_global = 0

        self.recording = True
        self.last_record_skip_reason = "recording started"
        if self.seed_recording_with_current_pose(wait=False):
            self.get_logger().info("path recording started")
        else:
            self.get_logger().warn(
                "path recording started; waiting for map-frame pose"
            )
        self.publish_status()
        return True

    def stop_recording(self, save):
        """Disable recording and optionally save path and pose."""
        if save:
            self.seed_recording_with_current_pose()
        self.recording = False

        if save and not self.saved_once:
            path_to_save = self.valid_global_poses(self.poses)
            if len(path_to_save) < 2:
                message = (
                    "recording stopped without saving: not enough map-frame path points "
                    f"(map_points={len(path_to_save)}, raw_points={len(self.poses)}, "
                    f"last_record_event={self.last_record_skip_reason})"
                )
                self.get_logger().warn(message)
                self.publish_status()
                return True, message

            path_to_save = self.simplify_path(path_to_save)
            self.save_path(path_to_save)
            self.save_safe_path(path_to_save)
            pose = self.wait_for_map_pose(float(self.map_pose_ready_timeout_sec))
            if pose is not None:
                self.latest_pose = pose
            if (
                self.latest_pose is not None
                and self.latest_pose.frame_id == self.global_frame
            ):
                self.save_pose(self.latest_pose)
            self.saved_once = True

        self.publish_status()
        return True, f"saved map-frame path points to {self.csv_file}"

    def seed_recording_with_current_pose(self, wait=True):
        """Record the current map-frame pose immediately when available."""
        if wait:
            pose = self.wait_for_map_pose(float(self.map_pose_ready_timeout_sec))
        else:
            pose = self.current_map_pose()
        if pose is None:
            self.record_skipped_no_pose += 1
            self.last_record_skip_reason = "waiting for robot pose"
            return False

        self.latest_pose = pose
        if self.should_record(pose):
            self.poses.append(pose)
            self.last_recorded_pose = pose
        self.last_record_skip_reason = "recording map-frame poses"
        return True

    def valid_global_poses(self, poses):
        """Return only poses that can safely be used as a Nav2 map-frame path."""
        return filter_global_frame_poses(
            poses=poses,
            global_frame=self.global_frame,
        )

    def save_path(self, poses):
        """Save recorded path points as CSV."""
        save_path_csv(self.csv_file, poses)

        self.get_logger().info(
            f"saved {len(poses)} points to {self.csv_file} "
            f"(recorded_points={len(self.poses)}, "
            f"map_points={len(self.valid_global_poses(self.poses))})"
        )

    def save_safe_path(self, raw_poses):
        """Defer safe-path generation until the newly recorded map is saved."""
        self.remove_safe_path_file()
        if len(raw_poses) >= 2:
            self.get_logger().info(
                "safe-path generation deferred until replay start so the "
                "supplied generator uses the newly saved map"
            )

    def remove_safe_path_file(self):
        """Remove a stale safe path file so replay cannot use outdated data."""
        if not os.path.exists(self.safe_csv_file):
            return

        os.remove(self.safe_csv_file)
        self.get_logger().warn(
            f"removed stale safe replay path file: {self.safe_csv_file}"
        )

    def simplify_path(self, poses):
        """Remove nearly collinear middle points before writing the CSV."""
        return simplify_recorded_path(
            poses=poses,
            simplify_tolerance_m=float(self.simplify_tolerance_m),
            min_yaw_delta_rad=float(self.min_yaw_delta_rad),
        )

    def save_pose(self, pose):
        """Save the latest pose as a small YAML file."""
        save_pose_yaml(self.pose_file, pose)

        if not self.shutting_down:
            self.get_logger().info(f"pose saved to {self.pose_file}")

    def load_saved_pose(self):
        """Load the last saved pose from YAML when the node was restarted."""
        pose, error = load_pose_yaml(self.pose_file)
        if error:
            self.get_logger().warn(error)
        return pose

    def load_saved_path_poses(self, reverse=False):
        """Load replay path poses in the requested driving direction."""
        poses, path_source = self.load_saved_path()
        if len(poses) < 2:
            return []
        if reverse:
            poses = list(reversed(poses))
            poses = self.with_path_direction_yaw(poses)

        for pose in poses:
            if pose.frame_id != self.global_frame:
                self.get_logger().warn(
                    "saved path contains a non-global-frame pose: "
                    f"{pose.frame_id}"
                )
                return []

        self.get_logger().info(
            f"loaded replay path from {path_source} ({len(poses)} poses)"
        )
        return poses

    def load_saved_path_as_nav_path(self, reverse=False):
        """Load the saved CSV as a Nav2 FollowPath goal path."""
        poses = self.load_saved_path_poses(reverse=reverse)
        if len(poses) < 2:
            return None
        return self.poses_to_nav_path(poses)

    # Nav2 path replay

    def poses_to_nav_path(self, poses):
        """Convert compact recorded poses to a nav_msgs/Path for Nav2."""
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for pose in poses:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.global_frame
            pose_msg.header.stamp = path_msg.header.stamp
            pose_msg.pose.position.x = pose.x
            pose_msg.pose.position.y = pose.y
            pose_msg.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            path_msg.poses.append(pose_msg)

        return path_msg

    def send_follow_path_goal(self, path_msg):
        """Send a FollowPath action goal and attach result callbacks."""
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = "FollowPath"
        goal_msg.goal_checker_id = "goal_checker"

        future = self.follow_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self.follow_path_goal_response_callback)

    def with_path_direction_yaw(self, poses):
        """Return poses whose yaw follows the direction between path points."""
        directed_poses = []

        for index, pose in enumerate(poses):
            if index < len(poses) - 1:
                next_pose = poses[index + 1]
                yaw = math.atan2(next_pose.y - pose.y, next_pose.x - pose.x)
            elif directed_poses:
                yaw = directed_poses[-1].yaw
            else:
                yaw = pose.yaw

            directed_poses.append(
                RecordedPose(
                    stamp_sec=pose.stamp_sec,
                    frame_id=pose.frame_id,
                    x=pose.x,
                    y=pose.y,
                    yaw=yaw,
                )
            )

        return directed_poses

    def generate_attached_safe_path(self):
        """Run the supplied dynamic-cleanup generator against the latest map."""
        if not os.path.isfile(self.csv_file):
            return False, f"recorded path does not exist: {self.csv_file}"
        if not os.path.isfile(self.corner_turn_map_yaml):
            return False, (
                "saved map does not exist for safe-path generation: "
                f"{self.corner_turn_map_yaml}"
            )

        try:
            if os.path.exists(self.safe_csv_file):
                os.remove(self.safe_csv_file)
            completed = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "aftr_path_manager.safe_path_generator_dynamic_cleanup_node",
                    "--ros-args",
                    "-p",
                    f"raw_path_csv_path:={self.csv_file}",
                    "-p",
                    f"safe_path_csv_path:={self.safe_csv_file}",
                    "-p",
                    f"map_yaml_path:={self.corner_turn_map_yaml}",
                    "-p",
                    f"frame_id:={self.global_frame}",
                ],
                capture_output=True,
                check=False,
                text=True,
                timeout=float(self.safe_path_generator_timeout_sec),
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            message = f"supplied safe-path generator failed to run: {exc}"
            self.get_logger().error(message)
            return False, message

        if completed.returncode != 0:
            diagnostic = (completed.stderr or completed.stdout).strip()
            message = (
                "supplied safe-path generator exited unsuccessfully "
                f"(returncode={completed.returncode}): {diagnostic[-1200:]}"
            )
            self.get_logger().error(message)
            return False, message

        generated, error = self.load_xyyaw_path_csv(self.safe_csv_file)
        if error or len(generated) < 2:
            message = error or "supplied safe-path output has fewer than two poses"
            self.get_logger().error(message)
            return False, message

        self.get_logger().info(
            "supplied dynamic-cleanup safe path applied "
            f"(output={len(generated)}, file={self.safe_csv_file})"
        )
        return True, "supplied safe path generated"

    def load_xyyaw_path_csv(self, csv_path):
        """Load the x,y,yaw_rad schema emitted by the supplied generators."""
        poses = []
        try:
            with open(csv_path, newline="", encoding="utf-8") as csv_handle:
                reader = csv.DictReader(csv_handle)
                for index, row in enumerate(reader):
                    yaw_text = row.get("yaw_rad", row.get("yaw", "0.0"))
                    poses.append(
                        RecordedPose(
                            stamp_sec=index * 0.05,
                            frame_id=self.global_frame,
                            x=float(row["x"]),
                            y=float(row["y"]),
                            yaw=float(yaw_text),
                        )
                    )
        except (OSError, KeyError, TypeError, ValueError) as exc:
            return [], f"generated path CSV could not be loaded: {csv_path}: {exc}"
        return poses, None

    def generate_attached_wide_turn_path(self, poses, reverse=False):
        """Run the supplied wide_turn_v2 CSV generator for this direction."""
        if len(poses) < 2:
            return list(poses)
        if not os.path.isfile(self.corner_turn_map_yaml):
            self.get_logger().warn(
                "attached wide-turn skipped: map YAML does not exist: "
                f"{self.corner_turn_map_yaml}"
            )
            return []

        suffix = "_reverse" if reverse else ""
        output_root, output_extension = os.path.splitext(
            self.corner_turn_output_csv
        )
        output_path = f"{output_root}{suffix}{output_extension or '.csv'}"
        input_path = f"{output_root}{suffix}_input.csv"
        save_path_csv(input_path, poses)
        try:
            if os.path.exists(output_path):
                os.remove(output_path)
            completed = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "aftr_path_manager.map_corner_turn_path_csv_generator_node",
                    "--ros-args",
                    "-p",
                    f"input_csv_path:={input_path}",
                    "-p",
                    f"output_csv_path:={output_path}",
                    "-p",
                    f"map_yaml_path:={self.corner_turn_map_yaml}",
                    "-p",
                    "publish_output_path:=false",
                ],
                capture_output=True,
                check=False,
                text=True,
                timeout=float(self.corner_turn_generator_timeout_sec),
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(
                f"attached wide-turn generator failed to run: {exc}"
            )
            return []

        if completed.returncode != 0:
            diagnostic = (completed.stderr or completed.stdout).strip()
            self.get_logger().error(
                "attached wide-turn generator exited unsuccessfully "
                f"(returncode={completed.returncode}): {diagnostic[-1200:]}"
            )
            return []

        generated, error = self.load_xyyaw_path_csv(output_path)
        if error:
            self.get_logger().error(error)
            return []

        if len(generated) < 2:
            self.get_logger().warn(
                "attached wide-turn output had fewer than two poses; "
                "saved-path replay will not start"
            )
            return []

        direction = "reverse" if reverse else "forward"
        self.get_logger().info(
            "attached wide_turn_v2 path applied "
            f"(direction={direction}, input={len(poses)}, "
            f"output={len(generated)}, file={output_path})"
        )
        return generated

    def load_saved_path(self):
        """Load replay path points, preferring the generated safe path file."""
        safe_poses, safe_error = load_path_csv(self.safe_csv_file)
        if safe_error or len(safe_poses) < 2:
            safe_poses, safe_error = self.load_xyyaw_path_csv(self.safe_csv_file)
        if safe_error:
            self.get_logger().warn(safe_error)
        if len(safe_poses) >= 2:
            return safe_poses, self.safe_csv_file

        poses, error = load_path_csv(self.csv_file)
        if error:
            self.get_logger().warn(error)
        return poses, self.csv_file

    def follow_path_goal_response_callback(self, future):
        """Store the FollowPath goal handle after Nav2 accepts the goal."""
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.follow_state.active = False
            self.follow_state.goal_handle = None
            self.follow_state.last_event = "goal_rejected"
            self.get_logger().error("FollowPath goal was rejected")
            self.publish_status()
            return

        self.follow_state.goal_handle = goal_handle
        self.get_logger().info("FollowPath goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.follow_path_result_callback)

    def follow_path_result_callback(self, future):
        """Log the final FollowPath result status."""
        result = future.result()
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("FollowPath completed successfully")
            self.follow_state.last_event = "completed"
            self.clear_follow_path_state()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("FollowPath was canceled")
            self.follow_state.last_event = "canceled"
            self.clear_follow_path_state()
        else:
            self.get_logger().error(f"FollowPath finished with status={status}")
            self.follow_state.last_event = f"aborted_status_{status}"
            if not self.schedule_follow_path_retry(status):
                self.clear_follow_path_state()

        self.publish_status()

    def follow_path_cancel_done_callback(self, future):
        """Clear local FollowPath state after a cancel request completes."""
        self.clear_follow_path_state()
        self.get_logger().info("FollowPath cancel completed")
        self.publish_status()

    def schedule_follow_path_retry(self, status):
        """Schedule a retry if a FollowPath failure was not an operator cancel."""
        if self.follow_state.cancel_requested:
            return False
        if not self.follow_retry_enabled:
            return False
        if self.has_reached_active_path_goal():
            self.get_logger().info(
                "FollowPath reported failure, but final saved pose is already reached"
            )
            return False

        max_retries = int(self.follow_max_retries)
        if max_retries > 0 and self.follow_state.retry_count >= max_retries:
            self.follow_state.last_event = (
                "blocked: obstacle did not clear before retry limit "
                f"({self.follow_state.retry_count}/{max_retries})"
            )
            self.get_logger().error(self.follow_state.last_event)
            return False

        self.follow_state.retry_count += 1
        delay_s = compute_retry_delay_seconds(
            retry_count=self.follow_state.retry_count,
            base_delay_s=float(self.follow_retry_delay_s),
            max_delay_s=float(self.follow_retry_max_delay_s),
        )
        self.follow_state.goal_handle = None
        self.follow_state.active = True
        self.destroy_follow_retry_timer()
        self.follow_state.retry_timer = self.create_timer(
            delay_s,
            self.retry_follow_path_once,
        )
        self.follow_state.last_event = (
            f"blocked by obstacle; retry {self.follow_state.retry_count} scheduled"
        )
        self.get_logger().warn(
            "FollowPath will retry after obstacle/progress failure "
            f"(status={status}, retry={self.follow_state.retry_count}, delay={delay_s:.1f}s)"
        )
        return True

    def retry_follow_path_once(self):
        """Re-send the remaining saved path after a retry delay."""
        self.destroy_follow_retry_timer()

        if self.follow_state.cancel_requested:
            self.clear_follow_path_state()
            self.publish_status()
            return

        remaining_poses = self.remaining_follow_poses_from_current_pose()
        if len(remaining_poses) < 2:
            if self.has_reached_active_path_goal():
                self.get_logger().info("saved path final pose reached after retry wait")
                self.follow_state.last_event = "completed"
            else:
                self.get_logger().error("not enough remaining path points to retry")
                self.follow_state.last_event = (
                    "failed: not enough remaining path points to retry"
                )
            self.clear_follow_path_state()
            self.publish_status()
            return

        if not self.follow_path_client.server_is_ready():
            if not self.follow_path_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(
                    "FollowPath action server is unavailable; scheduling another retry"
                )
                self.follow_state.last_event = "FollowPath action server unavailable"
                self.schedule_follow_path_retry(GoalStatus.STATUS_ABORTED)
                self.publish_status()
                return

        # This is already the exact generated path. Never run a second cleanup
        # or corner pass over a shortened retry path.
        path_msg = self.poses_to_nav_path(remaining_poses)
        self.path_pub.publish(path_msg)
        self.send_follow_path_goal(path_msg)
        self.follow_state.last_event = (
            f"retry {self.follow_state.retry_count} sent "
            f"({len(remaining_poses)} poses)"
        )
        self.get_logger().info(
            "FollowPath retry sent remaining path "
            f"({len(remaining_poses)} poses, reverse={self.follow_state.reverse})"
        )
        self.publish_status()

    def remaining_follow_poses_from_current_pose(self):
        """Return the active path from the closest stored point to the final pose."""
        pose = self.current_robot_pose()
        if pose is None:
            self.get_logger().warn("current robot pose is unavailable; retrying full path")
        return build_remaining_follow_poses(
            active_poses=self.follow_state.poses,
            current_pose=pose,
            global_frame=self.global_frame,
            lookback_points=int(self.follow_resume_lookback_points),
        )

    def interpolate_follow_path(self, poses):
        """Add intermediate poses so Nav2 receives a dense, stable path."""
        if len(poses) < 2:
            return list(poses)

        step = float(self.follow_path_interpolation_step_m)
        if step <= 0.0:
            return list(poses)

        dense_poses = [poses[0]]
        for start_pose, end_pose in zip(poses[:-1], poses[1:]):
            dx = end_pose.x - start_pose.x
            dy = end_pose.y - start_pose.y
            distance = math.hypot(dx, dy)
            if distance < 1e-6:
                continue

            yaw = math.atan2(dy, dx)
            segment_count = max(1, int(math.ceil(distance / step)))
            for index in range(1, segment_count + 1):
                ratio = index / segment_count
                dense_poses.append(
                    RecordedPose(
                        stamp_sec=end_pose.stamp_sec,
                        frame_id=self.global_frame,
                        x=start_pose.x + dx * ratio,
                        y=start_pose.y + dy * ratio,
                        yaw=yaw if index < segment_count else end_pose.yaw,
                    )
                )

        return dense_poses

    def preprocess_follow_path_for_replay(self, poses, apply_corner_fillets=True):
        """Apply replay-time cleanup before sending a path to Nav2."""
        input_pose_count = len(poses)
        prepared_poses = list(poses)
        prepared_poses = self.cleanup_follow_path_if_needed(prepared_poses)
        cleanup_pose_count = len(prepared_poses)
        prepared_poses = self.interpolate_follow_path(prepared_poses)
        interpolated_pose_count = len(prepared_poses)
        fillet_summary = None
        if (
            apply_corner_fillets
            and bool(self.follow_path_bidirectional_corner_enabled)
        ):
            fillet_grid = self.build_follow_path_cleanup_grid()
            fillet_clearance_m = float(self.follow_path_safety_clearance_m)

            def fillet_sample_is_safe(sample):
                """Check a turn sample against the configured map clearance."""
                if fillet_grid is None:
                    return True
                return is_pose_safe(
                    sample,
                    fillet_grid,
                    fillet_clearance_m,
                )

            fillet_summary = apply_bidirectional_corner_fillets(
                prepared_poses,
                target_radius_m=float(
                    self.follow_path_bidirectional_corner_radius_m
                ),
                heading_threshold_rad=math.radians(
                    float(self.follow_path_bidirectional_corner_heading_deg)
                ),
                max_deviation_m=float(
                    self.follow_path_bidirectional_corner_max_deviation_m
                ),
                resample_step_m=float(
                    self.follow_path_bidirectional_corner_resample_step_m
                ),
                minimum_radius_m=float(
                    self.follow_path_bidirectional_corner_minimum_radius_m
                ),
                sample_is_safe=fillet_sample_is_safe,
                outward_overshoot_m=float(
                    self.follow_path_bidirectional_corner_outward_overshoot_m
                ),
            )
            prepared_poses = fillet_summary.poses
            # Do not run point-wise clearance cleanup after curve generation.
            # Every accepted sample was already checked against the same map
            # clearance above; moving individual samples now would recreate
            # curvature spikes at the corner entrance and exit.
        prepared_poses = self.with_path_direction_yaw(prepared_poses)

        if (
            cleanup_pose_count != input_pose_count
            or interpolated_pose_count != cleanup_pose_count
        ):
            self.get_logger().info(
                "follow path replay preprocessing complete "
                f"(input={input_pose_count}, "
                f"after_cleanup={cleanup_pose_count}, "
                f"after_interpolation={interpolated_pose_count})"
            )
        if fillet_summary is not None and fillet_summary.detected_corners > 0:
            applied_radii = ",".join(
                f"{radius_m:.2f}"
                for radius_m in fillet_summary.applied_radii_m
            )
            self.get_logger().info(
                "bidirectional corner fillet complete "
                f"(detected={fillet_summary.detected_corners}, "
                f"modified={fillet_summary.modified_corners}, "
                f"minimum_radius={fillet_summary.minimum_applied_radius_m:.2f}m, "
                f"degraded={fillet_summary.degraded_corners}, "
                "maximum_curvature_rate="
                f"{fillet_summary.maximum_curvature_rate_per_m2:.2f}/m2, "
                f"radii={applied_radii or '-'}m, "
                f"output={len(prepared_poses)})"
            )
        return prepared_poses

    def build_follow_path_cleanup_grid(self):
        """Build the occupancy-grid snapshot used by replay-path cleanup."""
        if self.latest_map is None:
            self.get_logger().warn(
                "follow path safety cleanup skipped: no /map has been received yet"
            )
            return None

        grid = build_grid_snapshot(
            self.latest_map,
            unknown_is_occupied=bool(self.follow_path_unknown_is_occupied),
        )
        if grid is None:
            self.get_logger().warn(
                "follow path safety cleanup skipped: invalid occupancy grid"
            )
            return None

        return grid

    def build_follow_path_cleanup_options(self):
        """Collect replay cleanup parameters in one place for readability."""
        return {
            "clearance_m": float(self.follow_path_safety_clearance_m),
            "max_lateral_shift_m": float(self.follow_path_max_lateral_shift_m),
            "preferred_clearance_m": float(
                self.follow_path_preferred_clearance_m
            ),
            "preferred_max_lateral_shift_m": float(
                self.follow_path_preferred_max_lateral_shift_m
            ),
            "lateral_sample_step_m": float(self.follow_path_lateral_sample_step_m),
            "preserve_start_distance_m": float(
                self.follow_path_preserve_start_distance_m
            ),
            "preserve_goal_distance_m": float(
                self.follow_path_preserve_goal_distance_m
            ),
            "correction_mode": str(self.follow_path_correction_mode),
            "centering_enabled": bool(self.follow_path_centering_enabled),
            "corridor_max_width_m": float(
                self.follow_path_centering_corridor_max_width_m
            ),
            "corner_heading_threshold_rad": math.radians(
                float(self.follow_path_centering_corner_heading_deg)
            ),
            "corner_shift_scale": float(
                self.follow_path_centering_corner_shift_scale
            ),
            "smoothing_window": int(self.follow_path_centering_smoothing_window),
            "transition_distance_m": float(
                self.follow_path_centering_transition_distance_m
            ),
            "max_shift_delta_m": float(
                self.follow_path_centering_max_shift_delta_m
            ),
            "shift_smoothing_window": int(
                self.follow_path_centering_shift_smoothing_window
            ),
            "outlier_threshold_m": float(
                self.follow_path_centering_outlier_threshold_m
            ),
            "pre_simplify_enabled": bool(self.follow_path_pre_simplify_enabled),
            "pre_simplify_min_spacing_m": float(
                self.follow_path_pre_simplify_min_spacing_m
            ),
            "pre_simplify_heading_threshold_rad": math.radians(
                float(self.follow_path_pre_simplify_heading_deg)
            ),
            "corner_clearance_delta_threshold_m": float(
                self.follow_path_corner_clearance_delta_threshold_m
            ),
            "corner_shift_delta_threshold_m": float(
                self.follow_path_corner_shift_delta_threshold_m
            ),
            "corner_segment_padding_points": int(
                self.follow_path_corner_segment_padding_points
            ),
            "corner_curve_resample_step_m": float(
                self.follow_path_corner_curve_resample_step_m
            ),
            "corner_entry_transition_distance_m": float(
                self.follow_path_corner_entry_transition_distance_m
            ),
            "corner_exit_transition_distance_m": float(
                self.follow_path_corner_exit_transition_distance_m
            ),
            "corner_smoothing_strength": float(
                self.follow_path_corner_smoothing_strength
            ),
            "corner_max_deviation_m": float(
                self.follow_path_corner_max_deviation_m
            ),
        }

    def log_follow_path_cleanup_result(self, cleanup_result):
        """Log replay cleanup counters using the current parameter snapshot."""
        summary = (
            "follow path safety cleanup summary: "
            f"input={cleanup_result.input_points}, "
            f"output={cleanup_result.output_points}, "
            f"corner_mask_count={cleanup_result.corner_mask_count}, "
            f"preserved={cleanup_result.preserved_safe_points}, "
            f"preserved_outside_corner={cleanup_result.preserved_outside_corner_points}, "
            f"repaired={cleanup_result.repaired_unsafe_points}, "
            f"adjusted_inside_corner={cleanup_result.adjusted_inside_corner_points}, "
            f"corner_adjusted={cleanup_result.corner_adjusted_points}, "
            f"adjusted={cleanup_result.adjusted_points}, "
            f"unresolved_inside_corner={cleanup_result.unresolved_inside_corner_points}, "
            f"unresolved={cleanup_result.unresolved_points}, "
            f"unsafe_outside_corner_ignored={cleanup_result.unsafe_outside_corner_ignored_points}, "
            f"mode={cleanup_result.correction_mode}"
        )
        context_summary = (
            "contexts: "
            f"open_space={cleanup_result.open_space_points}, "
            f"corridor={cleanup_result.corridor_points}, "
            f"cornering_to_corridor={cleanup_result.cornering_points}"
        )

        if cleanup_result.unresolved_points > 0:
            self.get_logger().warn(summary)
            self.get_logger().warn(context_summary)
            return

        if cleanup_result.adjusted_points > 0:
            self.get_logger().info(summary)
            self.get_logger().info(context_summary)
            self.get_logger().info(
                "follow path safety cleanup adjusted indices: "
                + ", ".join(str(index) for index in cleanup_result.adjusted_indices)
            )
            return

        self.get_logger().info(summary)
        self.get_logger().info(context_summary)

    def cleanup_follow_path_if_needed(self, poses):
        """Repair unsafe saved path points by sampling nearby lateral candidates."""
        if not self.follow_path_safety_cleanup_enabled:
            return list(poses)

        if len(poses) < 3:
            return list(poses)

        grid = self.build_follow_path_cleanup_grid()
        if grid is None:
            return list(poses)

        cleanup_options = self.build_follow_path_cleanup_options()
        cleanup_result = cleanup_path_points(
            poses=poses,
            grid=grid,
            **cleanup_options,
        )
        self.log_follow_path_cleanup_result(cleanup_result)
        return cleanup_result.poses

    def has_reached_active_path_goal(self):
        """Return True if the robot is already close to the final saved pose."""
        if not self.follow_state.poses:
            return False

        pose = self.current_robot_pose()
        if pose is None:
            return False

        final_pose = self.follow_state.poses[-1]
        distance = math.hypot(final_pose.x - pose.x, final_pose.y - pose.y)
        return distance <= float(self.follow_final_tolerance_m)

    def remaining_follow_distance(self):
        """Estimate remaining distance to the active FollowPath goal."""
        if not self.follow_state.active or not self.follow_state.poses:
            return 0.0

        pose = self.current_robot_pose()
        return estimate_remaining_follow_distance(
            active_poses=self.follow_state.poses,
            current_pose=pose,
        )

    # FollowPath state helpers

    def destroy_follow_retry_timer(self):
        """Destroy a pending retry timer, if one exists."""
        if self.follow_state.retry_timer is None:
            return

        self.follow_state.retry_timer.cancel()
        self.destroy_timer(self.follow_state.retry_timer)
        self.follow_state.retry_timer = None

    def clear_follow_path_state(self):
        """Reset local FollowPath bookkeeping after completion or cancellation."""
        last_event = self.follow_state.last_event
        self.destroy_follow_retry_timer()
        self.follow_state.reset()
        self.follow_state.last_event = last_event

    def reset_follow_status(self, event):
        """Clear stale FollowPath state before a new operator workflow."""
        self.clear_follow_path_state()
        self.follow_state.last_event = event

    # Localization support and shutdown

    def publish_initial_pose(self, pose):
        """Publish a pose in map frame to AMCL/Nav2."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = pose.frame_id
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        # AMCL may need a brief moment to attach its /initialpose subscription
        # after lifecycle activation. Repeating the publish reduces lost poses.
        publish_count = max(1, int(self.initial_pose_publish_count))
        interval_s = max(0.0, float(self.initial_pose_publish_interval_s))
        for _index in range(publish_count):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            if interval_s > 0.0:
                time.sleep(interval_s)

    def cleanup(self):
        """Save data once when the node is shutting down."""
        self.shutting_down = True
        if self.recording:
            try:
                self.stop_recording(save=True)
            except KeyboardInterrupt:
                pass


def main(args=None):
    """Run the path manager node."""
    node = None
    try:
        rclpy.init(args=args)
        node = PathManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("path_manager interrupted by SIGINT")
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"path_manager crashed: {exc}")
        else:
            print(f"path_manager crashed before startup completed: {exc}")
        traceback.print_exc()
    finally:
        if node is not None:
            try:
                node.cleanup()
            except KeyboardInterrupt:
                node.get_logger().info(
                    "path_manager cleanup interrupted; continuing shutdown"
                )
            except Exception as exc:
                node.get_logger().warn(
                    f"failed to run path_manager cleanup cleanly: {exc}"
                )

            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
            except Exception as exc:
                node.get_logger().warn(
                    f"failed to destroy path_manager node cleanly: {exc}"
                )

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
            except Exception as exc:
                if node is not None:
                    node.get_logger().warn(
                        f"failed to shut down rclpy cleanly: {exc}"
                    )
                else:
                    print(f"failed to shut down rclpy cleanly: {exc}")


if __name__ == "__main__":
    main()
