# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Launch the AFTR path manager."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create the path manager launch description."""
    path_manager = Node(
        package="aftr_path_manager",
        executable="path_manager",
        name="path_manager",
        output="screen",
        parameters=[
            {
                # Start recording only after the mode manager calls /path_manager/start_record.
                # Localization bringup must not restart CSV recording.
                "auto_start": False,
                "csv_file": os.path.expanduser("~/recorded_path.csv"),
                "safe_csv_file": os.path.expanduser("~/safe_path.csv"),
                "safe_path_generator_timeout_sec": 60.0,
                "corner_turn_output_csv": os.path.expanduser(
                    "~/corner_turn_path.csv"
                ),
                "corner_turn_map_yaml": os.path.expanduser(
                    "~/map/mdbot_map.yaml"
                ),
                "corner_turn_generator_timeout_sec": 30.0,
                "global_frame": "map",
                "odom_frame": "odom",
                "robot_frame": "base_footprint",
                "allow_odom_fallback": True,
                "record_global_frame_only": True,
                "min_distance_m": 0.05,
                "min_yaw_delta_rad": 0.087266,
                "min_time_delta_s": 0.05,
                "simplify_tolerance_m": 0.02,
                "max_start_heading_error_rad": 0.610865,
                "max_start_path_distance_m": 0.45,
                "min_start_segment_length_m": 0.20,
                "follow_path_interpolation_step_m": 0.05,
                "follow_path_safety_cleanup_enabled": True,
                # The robot footprint is 0.50 m wide. A 0.50 m center-point
                # clearance leaves roughly one additional half-width outside
                # the physical body without modelling trailer articulation.
                "follow_path_safety_clearance_m": 0.65,
                "follow_path_max_lateral_shift_m": 0.65,
                # Absorb localization error and the 0.58 m cart width by
                # moving wall-adjacent sections farther into free space. This
                # remains a correction request, not a route rejection rule.
                "follow_path_preferred_clearance_m": 0.90,
                "follow_path_preferred_max_lateral_shift_m": 0.55,
                "follow_path_lateral_sample_step_m": 0.05,
                "follow_path_preserve_start_distance_m": 0.25,
                "follow_path_preserve_goal_distance_m": 0.25,
                "follow_path_unknown_is_occupied": True,
                # Prevent one replay point from moving far away from its
                # neighbours when a wall-clearance correction is applied.
                "follow_path_centering_max_shift_delta_m": 0.03,
                "follow_path_centering_outlier_threshold_m": 0.08,
                # Use equal transition distances so forward and reverse replay
                # receive the same geometry around a saved corner.
                "follow_path_centering_corner_heading_deg": 15.0,
                "follow_path_corner_entry_transition_distance_m": 2.40,
                "follow_path_corner_exit_transition_distance_m": 2.40,
                "follow_path_corner_smoothing_strength": 0.80,
                "follow_path_corner_max_deviation_m": 0.35,
                # Replace major turns with a direction-symmetric, zero-endpoint-
                # curvature transition. Short segments use the best available
                # smooth radius instead of falling back to the original kink or
                # rejecting an otherwise traversable saved route.
                "follow_path_bidirectional_corner_enabled": True,
                # Ignore small recorded-path heading jitter so it cannot steal
                # transition distance from the nearby load-bearing cart corner.
                "follow_path_bidirectional_corner_heading_deg": 25.0,
                "follow_path_bidirectional_corner_radius_m": 2.40,
                "follow_path_bidirectional_corner_max_deviation_m": 0.80,
                "follow_path_bidirectional_corner_resample_step_m": 0.05,
                # This is a curve-candidate preference, not a route rejection:
                # prefer an arc of at least 1.50 m and reduce the outside offset
                # or fall back to the ordinary smooth fillet when space is short.
                "follow_path_bidirectional_corner_minimum_radius_m": 1.50,
                # Continue the tangent controls beyond the saved corner so the
                # robot draws one broad outside arc instead of cutting inside.
                # The 0.80 m deviation and map-clearance checks still cap it.
                "follow_path_bidirectional_corner_outward_overshoot_m": 1.00,
            },
        ],
    )

    return LaunchDescription([path_manager])
