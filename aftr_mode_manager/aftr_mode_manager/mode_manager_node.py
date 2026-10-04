# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Main ROS node for the AFTR mode manager package.

This node is the assembly point of the package. It declares runtime parameters,
creates managed processes and readiness helpers, registers ROS interfaces, and
publishes the consolidated operator-facing status used by the GUI.
"""

import os
import threading
import time
import traceback

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer
from tf2_ros import TransformListener

from aftr_mode_manager.feedback.audio_events import AudioEventMixin
from aftr_mode_manager.feedback.led_events import LedEventMixin
from aftr_mode_manager.mode_state import ModeStatus
from aftr_mode_manager.mode_state import RobotMode
from aftr_mode_manager.runtime.bringup_manager import BringupManagerMixin
from aftr_mode_manager.operator_status import OperatorStatusMixin
from aftr_mode_manager.runtime.process_supervisor import ManagedProcess
from aftr_mode_manager.runtime.readiness import MessageReadiness
from aftr_mode_manager.runtime.readiness import NodeReadiness
from aftr_mode_manager.runtime.readiness import TopicReadiness
from aftr_mode_manager.runtime.service_clients import ServiceClientMixin
from aftr_mode_manager.workflow.manager import WorkflowManagerMixin


class ModeManagerNode(
    AudioEventMixin,
    LedEventMixin,
    OperatorStatusMixin,
    WorkflowManagerMixin,
    BringupManagerMixin,
    ServiceClientMixin,
    Node,
):
    """Own the ROS-facing assembly of the AFTR workflow manager."""

    RUNTIME_PARAMETER_DEFAULTS = (
        ("auto_start_base", False),
        ("shutdown_timeout_sec", 10.0),
        ("base_ready_timeout_sec", 45.0),
        ("base_min_ready_delay_sec", 8.0),
        ("scan_message_timeout_sec", 10.0),
        ("controller_ready_timeout_sec", 20.0),
        ("base_restart_attempts", 1),
        ("base_restart_delay_sec", 5.0),
        ("slam_ready_timeout_sec", 20.0),
        ("path_ready_timeout_sec", 10.0),
        ("follower_ready_timeout_sec", 10.0),
        ("nav2_ready_timeout_sec", 60.0),
        ("amcl_pose_ready_timeout_sec", 10.0),
        ("map_save_timeout_sec", 15.0),
        ("nav2_initial_pose_delay_sec", 1.0),
        ("nav2_lifecycle_ready_timeout_sec", 60.0),
        ("nav2_stable_ready_duration_sec", 2.0),
        ("nav2_stable_ready_timeout_sec", 10.0),
        # path_manager may spend up to 60 s on supplied safe-path generation
        # and 30 s on supplied corner generation before dispatching FollowPath.
        ("path_replay_service_timeout_sec", 120.0),
        ("amcl_pose_freshness_sec", 5.0),
        ("alignment_confirmation_delay_sec", 1.5),
        ("follow_reacquire_audio_min_search_sec", 1.2),
        ("robust_tracking", False),
        ("localizing_promote_timeout_sec", 30.0),
        ("enable_rviz_view", False),
        ("reject_stale_runtime_nodes", True),
        ("stale_runtime_audit_attempts", 3),
        ("stale_runtime_audit_interval_sec", 1.0),
        ("cleanup_stale_runtime_processes", True),
        ("stale_runtime_cleanup_timeout_sec", 2.0),
    )

    def __init__(self):
        """Initialize parameters, runtime helpers, and ROS interfaces."""
        super().__init__("mode_manager")
        self.callback_group = ReentrantCallbackGroup()
        self.status = ModeStatus()
        self._initialize_runtime_state()
        self._declare_runtime_parameters()
        self.map_save_file = os.path.expanduser(
            self.declare_parameter(
                "map_save_file",
                "/data/maps/mdbot_map",
            ).value
        )
        default_rviz_config_file = os.path.join(
            get_package_share_directory("aftr_navigation"),
            "rviz",
            "nav2_default_view.rviz",
        )
        self.rviz_config_file = self.declare_parameter(
            "rviz_config_file",
            default_rviz_config_file,
        ).value
        self.base_started_at = None

        # Keep the assembly order explicit here so this file stays the single
        # place where the full mode manager wiring can be understood quickly.
        self._create_managed_processes()
        self._create_readiness_helpers()
        self._create_publishers_and_subscribers()
        self._create_external_service_clients()
        self._create_nav2_lifecycle_clients()
        self._register_mode_manager_services()

        self.auto_start_base_timer = None
        self.stale_runtime_audit_timer = None
        if self.reject_stale_runtime_nodes:
            # Confirm stale graph entries before starting any managed process.
            self.stale_runtime_audit_timer = self.create_timer(
                max(0.2, float(self.stale_runtime_audit_interval_sec)),
                self.audit_stale_runtime_nodes_once,
                callback_group=self.callback_group,
            )
        elif self.auto_start_base:
            self.schedule_auto_start_base(delay_sec=0.5)

        self.get_logger().info("mode_manager started in IDLE mode")

    def _initialize_runtime_state(self):
        """Initialize in-memory workflow fields before ROS wiring starts."""
        self.command_lock = threading.Lock()
        self.command_active = False
        self.active_command_name = ""
        self.last_service_error = ""
        self.last_path_follow_event = "idle"
        self.alignment_control_active = False
        self.next_drive_direction = "reverse"
        self.active_drive_direction = ""
        self.pending_drive_direction = ""
        self.latest_path_status = {}
        self.latest_follow_state = ""
        self.previous_follow_state = ""
        self.follow_search_started_at = None
        self.follow_detected_announced = False
        self.latest_linear_speed = 0.0
        self.localizing_wait_log_emitted = False
        self.localizing_started_at = None
        self.nav2_ready_stable_since = None
        self.reported_runtime_mismatches = set()
        self.process_running_snapshot = {}
        self.pending_managed_process_exits = set()
        self.runtime_monitor_lock = threading.Lock()
        self.stale_runtime_audit_count = 0
        self.stale_runtime_cleanup_attempted = False
        self.stale_runtime_audit_lock = threading.Lock()
        self.autonomous_blocked_audio_active = False
        self.autonomous_blocked_led_active = False
        self.system_ready_audio_announced = False
        self.alignment_confirmation_audio_timer = None
        self.alignment_confirmation_expected_commands = ()

    def _declare_runtime_parameters(self):
        """Declare scalar runtime parameters and store them as attributes."""
        for parameter_name, default_value in self.RUNTIME_PARAMETER_DEFAULTS:
            setattr(
                self,
                parameter_name,
                self.declare_parameter(parameter_name, default_value).value,
            )

        self.enable_rviz_view = bool(self.enable_rviz_view)
        if isinstance(self.robust_tracking, str):
            self.robust_tracking = self.robust_tracking.strip().lower() in {
                "1",
                "true",
                "yes",
                "on",
            }
        else:
            self.robust_tracking = bool(self.robust_tracking)

        self.reject_stale_runtime_nodes = bool(self.reject_stale_runtime_nodes)
        self.cleanup_stale_runtime_processes = bool(
            self.cleanup_stale_runtime_processes
        )

    def _create_managed_processes(self):
        """Create all external processes controlled by the mode manager."""
        self.managed_process_registry_dir = os.path.join(
            "/tmp",
            f"mdbot_mode_manager_{os.getuid()}",
            "processes",
        )

        def registry_path(process_name):
            """Return a per-process registry file within this manager instance."""
            return os.path.join(
                self.managed_process_registry_dir,
                f"{process_name}.json",
            )

        self.base_process = ManagedProcess(
            name="base_bringup",
            command=["ros2", "launch", "aftr_bringup", "mdbot.launch.py"],
            registry_path=registry_path("base_bringup"),
        )
        self.slam_process = ManagedProcess(
            name="mapping_slam",
            command=["ros2", "launch", "aftr_slam", "slam.launch.py"],
            registry_path=registry_path("mapping_slam"),
        )
        self.path_process = ManagedProcess(
            name="path_manager",
            command=["ros2", "launch", "aftr_path_manager", "path_manager.launch.py"],
            registry_path=registry_path("path_manager"),
        )
        self.follower_node = ManagedProcess(
            name="person_follow",
            command=[
                "ros2",
                "launch",
                "aftr_tracking",
                "follow.launch.py",
                "robust_tracking:="
                + ("true" if self.robust_tracking else "false"),
            ],
            registry_path=registry_path("person_follow"),
        )
        self.nav2_process = ManagedProcess(
            name="nav2_localization",
            command=["ros2", "launch", "aftr_navigation", "navigation.launch.py"],
            registry_path=registry_path("nav2_localization"),
        )
        self.rviz_process = ManagedProcess(
            name="rviz2_navigation_view",
            command=["rviz2", "-d", self.rviz_config_file],
            registry_path=registry_path("rviz2_navigation_view"),
        )

    def _create_readiness_helpers(self):
        """Create topic, node, and message readiness helpers."""
        self.base_readiness = TopicReadiness(
            self,
            required_topics=[
                "/cmd_vel",
                "/joint_states",
                "/odom",
                "/scan",
                "/tf",
                "/tf_static",
            ],
        )
        self.slam_readiness = TopicReadiness(self, required_topics=["/map"])
        self.path_readiness = TopicReadiness(
            self,
            required_topics=["/path_manager/status"],
        )
        self.follower_readiness = TopicReadiness(
            self,
            required_topics=["/follow_state"],
        )
        self.nav2_readiness = TopicReadiness(self, required_topics=["/map"])
        self.nav2_node_readiness = NodeReadiness(
            self,
            required_nodes=[
                "/amcl",
                "/map_server",
                "/controller_server",
                "/planner_server",
                "/bt_navigator",
                "/lifecycle_manager_localization",
                "/lifecycle_manager_navigation",
                "/local_costmap/local_costmap",
                "/global_costmap/global_costmap",
            ],
        )
        self.amcl_pose_readiness = TopicReadiness(
            self,
            required_topics=["/amcl_pose"],
        )
        self.amcl_pose_message_readiness = MessageReadiness(
            self,
            PoseWithCovarianceStamped,
            "/amcl_pose",
            10,
            callback_group=self.callback_group,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.scan_message_readiness = MessageReadiness(
            self,
            LaserScan,
            "/scan",
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

    def localization_transform_ready(self, timeout_sec=0.0):
        """Return whether the live map-to-base transform chain is available."""
        return self.tf_buffer.can_transform(
            "map",
            "base_footprint",
            Time(),
            timeout=Duration(seconds=max(0.0, float(timeout_sec))),
        )

    def _create_publishers_and_subscribers(self):
        """Create publishers, timers, and subscriptions owned by the node."""
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.audio_event_pub = self.create_publisher(
            String,
            "/mode_manager/audio_event",
            10,
        )
        self.led_event_pub = self.create_publisher(
            String,
            "/mode_manager/led_event",
            10,
        )
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.path_status_sub = self.create_subscription(
            String,
            "/path_manager/status",
            self.path_status_callback,
            10,
            callback_group=self.callback_group,
        )
        self.follow_state_sub = self.create_subscription(
            String,
            "/follow_state",
            self.follow_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )

    def _create_external_service_clients(self):
        """Create service clients used to talk to other robot packages."""
        trigger_clients = (
            ("path_stop_record_client", "/path_manager/stop_record"),
            ("path_start_record_client", "/path_manager/start_record"),
            ("path_check_record_ready_client", "/path_manager/check_record_ready"),
            ("path_save_pose_client", "/path_manager/save_pose"),
            ("path_publish_initial_pose_client", "/path_manager/publish_initial_pose"),
            ("path_follow_saved_path_client", "/path_manager/follow_saved_path"),
            (
                "path_follow_saved_path_reverse_client",
                "/path_manager/follow_saved_path_reverse",
            ),
            ("path_cancel_follow_path_client", "/path_manager/cancel_follow_path"),
        )
        for attr_name, service_name in trigger_clients:
            self._create_trigger_client(attr_name, service_name)

        self.controller_list_client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
            callback_group=self.callback_group,
        )
        self.local_costmap_clear_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=self.callback_group,
        )

    def _create_trigger_client(self, attr_name, service_name):
        """Create one Trigger client and store it under an attribute name.

        Args:
            attr_name: Attribute name used to store the created client.
            service_name: Fully qualified ROS service name.
        """
        setattr(
            self,
            attr_name,
            self.create_client(
                Trigger,
                service_name,
                callback_group=self.callback_group,
            ),
        )

    def _create_nav2_lifecycle_clients(self):
        """Create lifecycle state clients used for Nav2 readiness checks."""
        self.nav2_localization_lifecycle_service_names = [
            "/amcl/get_state",
            "/map_server/get_state",
        ]
        self.nav2_navigation_lifecycle_service_names = [
            "/controller_server/get_state",
            "/smoother_server/get_state",
            "/planner_server/get_state",
            "/behavior_server/get_state",
            "/bt_navigator/get_state",
            "/waypoint_follower/get_state",
            "/velocity_smoother/get_state",
        ]
        self.nav2_lifecycle_service_names = [
            *self.nav2_localization_lifecycle_service_names,
            *self.nav2_navigation_lifecycle_service_names,
        ]
        self.nav2_lifecycle_clients = {
            service_name: self.create_client(
                GetState,
                service_name,
                callback_group=self.callback_group,
            )
            for service_name in self.nav2_lifecycle_service_names
        }

    def _register_internal_services(self):
        """Register low-level runtime start/check/stop services."""
        self._register_trigger_services(
            (
                ("~/internal/start_base", self.start_base_callback),
                ("~/internal/check_base_ready", self.check_base_ready_callback),
                ("~/internal/stop_base", self.stop_base_callback),
                ("~/internal/start_slam", self.start_slam_callback),
                ("~/internal/check_slam_ready", self.check_slam_ready_callback),
                ("~/internal/stop_slam", self.stop_slam_callback),
                ("~/internal/start_path_manager", self.start_path_manager_callback),
                ("~/internal/check_path_ready", self.check_path_ready_callback),
                ("~/internal/stop_path_manager", self.stop_path_manager_callback),
                ("~/internal/start_follower_node", self.start_follower_node_callback),
                ("~/internal/check_follower_ready", self.check_follower_ready_callback),
                ("~/internal/stop_follower_node", self.stop_follower_node_callback),
                ("~/internal/start_nav2", self.start_nav2_callback),
                ("~/internal/check_nav2_ready", self.check_nav2_ready_callback),
                ("~/internal/stop_nav2", self.stop_nav2_callback),
                ("~/internal/start_rviz", self.start_rviz_callback),
                ("~/internal/stop_rviz", self.stop_rviz_callback),
            )
        )

    def _register_command_services(self):
        """Register public workflow command services."""
        self._register_trigger_services(
            (
                ("~/command/start_follow", self.start_follow_callback),
                (
                    "~/command/start_recording_follow",
                    self.start_recording_follow_callback,
                ),
                (
                    "~/command/finish_recording_for_alignment",
                    self.finish_recording_for_alignment_callback,
                ),
                ("~/command/start_alignment", self.start_alignment_callback),
                ("~/command/skip_alignment", self.skip_alignment_callback),
                ("~/command/finish_alignment", self.finish_alignment_callback),
                ("~/command/save_pose", self.save_pose_callback),
                ("~/command/start_localizing", self.start_localizing_callback),
                (
                    "~/command/set_autonomous_ready",
                    self.set_autonomous_ready_callback,
                ),
                ("~/command/prepare_autonomous", self.prepare_autonomous_callback),
                ("~/command/path_forward", self.path_forward_callback),
                ("~/command/path_reverse", self.path_reverse_callback),
                ("~/command/path_forward_auto", self.path_forward_auto_callback),
                ("~/command/path_reverse_auto", self.path_reverse_auto_callback),
                ("~/command/stop", self.stop_callback),
                ("~/command/clear_error", self.clear_error_callback),
            )
        )

    def _register_shutdown_service(self):
        """Register the mode manager shutdown service."""
        self._register_trigger_services(
            (
                ("~/shutdown", self.shutdown_callback),
            )
        )

    def _register_trigger_services(self, service_definitions):
        """Register multiple Trigger services from a compact table definition.

        Args:
            service_definitions: Iterable of ``(service_name, callback)`` pairs.
        """
        for service_name, callback in service_definitions:
            self.create_service(Trigger, service_name, callback)

    def _register_mode_manager_services(self):
        """Register every service exposed by the mode manager node."""
        self._register_internal_services()
        self._register_command_services()
        self._register_shutdown_service()

    def auto_start_base_once(self):
        """Start base once when the node is launched with auto-start enabled."""
        if self.auto_start_base_timer is not None:
            self.auto_start_base_timer.cancel()
            self.auto_start_base_timer = None
        response = Trigger.Response()
        if not self.begin_command(response, "auto_start_base"):
            self.get_logger().warn("auto_start_base skipped: mode_manager is busy")
            return

        try:
            self.start_base(response)
            if not response.success:
                self.get_logger().error(
                    "auto_start_base failed: " + response.message
                )
        finally:
            self.end_command()

    def publish_status(self):
        """Refresh cached runtime flags and publish the current status payload."""
        if self.command_active:
            self.status.busy = True
        self.status.base_running = self.base_process.is_running()
        if not self.status.base_running:
            self.status.base_ready = False
        elif self.status.base_ready and not self.scan_message_readiness.has_recent_message(
            max(self.scan_message_timeout_sec * 2.0, 2.0),
        ):
            self.status.base_ready = False
            self.status.last_error = "base scan messages are stale"
        elif self.status.base_ready and self.base_readiness.missing_topics():
            self.status.base_ready = False
        self.status.slam_running = self.slam_process.is_running()
        if not self.status.slam_running:
            self.status.slam_ready = False
        elif self.status.slam_ready and self.slam_readiness.missing_topics():
            self.status.slam_ready = False
        visible_path_managers = self.count_visible_node("/path_manager")
        self.status.path_running = (
            self.path_process.is_running() or visible_path_managers > 0
        )
        if not self.status.path_running:
            self.status.path_ready = False
        elif self.status.path_ready and self.path_readiness.missing_topics():
            self.status.path_ready = False
        self.status.follower_running = self.follower_node.is_running()
        if not self.status.follower_running:
            self.status.follower_ready = False
        elif self.status.follower_ready and self.follower_readiness.missing_topics():
            self.status.follower_ready = False
        self.status.nav2_running = self.nav2_process.is_running()
        if not self.status.nav2_running:
            self.status.nav2_ready = False
            self.status.amcl_pose_ready = False
        elif self.status.nav2_ready and self.nav2_missing_dependencies():
            self.status.nav2_ready = False
            self.status.amcl_pose_ready = False
        elif (
            self.status.amcl_pose_ready
            and self.amcl_pose_readiness.missing_topics()
        ):
            self.status.amcl_pose_ready = False

        self.log_managed_process_state_changes()
        self.log_runtime_mode_mismatches()

        self.promote_localizing_mode_if_nav2_ready()

        self.update_operator_status()
        msg = String()
        msg.data = self.status.to_status_text()
        self.status_pub.publish(msg)

    def log_managed_process_state_changes(self):
        """Serialize managed-process monitoring in the reentrant executor."""
        if not self.runtime_monitor_lock.acquire(blocking=False):
            return
        try:
            self._log_managed_process_state_changes()
        finally:
            self.runtime_monitor_lock.release()

    def _log_managed_process_state_changes(self):
        """Log each exit once and reconcile required runtime failures."""
        processes = {
            "base": self.base_process,
            "slam": self.slam_process,
            "path_manager": self.path_process,
            "follower": self.follower_node,
            "nav2": self.nav2_process,
            "rviz": self.rviz_process,
        }
        for name, process in processes.items():
            running = process.is_running()
            previous = self.process_running_snapshot.get(name)
            self.process_running_snapshot[name] = running
            if previous is True and not running:
                self.get_logger().warning(
                    f"managed process exited: process={name}, "
                    f"mode={self._mode_name()}, "
                    f"active_command={self.active_command_name or 'none'}; "
                    + process.describe()
                )
                if self.command_active:
                    self.pending_managed_process_exits.add(name)
                else:
                    self.handle_unexpected_managed_process_exit(name)

        if not self.command_active and self.pending_managed_process_exits:
            pending_exits = sorted(self.pending_managed_process_exits)
            self.pending_managed_process_exits.clear()
            for process_name in pending_exits:
                self.handle_unexpected_managed_process_exit(process_name)

    def handle_unexpected_managed_process_exit(self, process_name):
        """Stop motion and enter ERROR when a required process disappears."""
        mode = self.status.mode
        required_modes = {
            "base": {
                RobotMode.FOLLOW,
                RobotMode.RECORDING_FOLLOW,
                RobotMode.ALIGNMENT,
                RobotMode.LOCALIZING,
                RobotMode.AUTONOMOUS_READY,
                RobotMode.AUTONOMOUS_DRIVING,
            },
            "slam": {RobotMode.RECORDING_FOLLOW},
            "path_manager": {
                RobotMode.RECORDING_FOLLOW,
                RobotMode.ALIGNMENT,
                RobotMode.LOCALIZING,
                RobotMode.AUTONOMOUS_READY,
                RobotMode.AUTONOMOUS_DRIVING,
            },
            "follower": {RobotMode.FOLLOW, RobotMode.RECORDING_FOLLOW},
            "nav2": {
                RobotMode.ALIGNMENT,
                RobotMode.LOCALIZING,
                RobotMode.AUTONOMOUS_READY,
                RobotMode.AUTONOMOUS_DRIVING,
            },
        }
        if mode not in required_modes.get(process_name, set()):
            return

        failure_message = (
            f"required managed process exited unexpectedly: {process_name}; "
            "motion was stopped and the saved map/path files were preserved"
        )
        self.active_drive_direction = ""
        self.pending_drive_direction = ""
        self.cancel_alignment_confirmation_request()

        if mode == RobotMode.AUTONOMOUS_DRIVING:
            self.notify_autonomous_failed()
            self.notify_autonomous_failed_led()
        self.stop_autonomous_audio()

        # Preserve the base process and saved files, but stop every remaining
        # command producer so recovery never resumes motion automatically.
        if self.follower_node.is_running():
            self.follower_node.stop(self.shutdown_timeout_sec)
        if self.nav2_process.is_running():
            self.log_nav2_stop_request(
                f"required process exited unexpectedly: {process_name}"
            )
            self.nav2_process.stop(self.shutdown_timeout_sec)

        self.status.follower_running = self.follower_node.is_running()
        self.status.follower_ready = False
        self.status.nav2_running = self.nav2_process.is_running()
        self.status.nav2_ready = False
        self.status.amcl_pose_ready = False
        self.set_error(failure_message)

    def runtime_mode_mismatch_messages(self):
        """Return mode/process combinations that should not persist normally."""
        mode = self.status.mode
        messages = set()

        if mode not in {RobotMode.IDLE, RobotMode.ERROR} and not self.status.base_running:
            messages.add(f"mode={mode.value} but base process is not running")
        if mode == RobotMode.FOLLOW and not self.status.follower_running:
            messages.add("mode=FOLLOW but follower process is not running")
        if mode == RobotMode.RECORDING_FOLLOW:
            if not self.status.follower_running:
                messages.add(
                    "mode=RECORDING_FOLLOW but follower process is not running"
                )
            if not self.status.slam_running:
                messages.add("mode=RECORDING_FOLLOW but SLAM process is not running")
            if not self.status.path_running:
                messages.add(
                    "mode=RECORDING_FOLLOW but path manager is not running"
                )
        if mode in {
            RobotMode.ALIGNMENT,
            RobotMode.LOCALIZING,
            RobotMode.AUTONOMOUS_READY,
            RobotMode.AUTONOMOUS_DRIVING,
        }:
            if not self.status.nav2_running:
                messages.add(f"mode={mode.value} but Nav2 process is not running")
            if not self.status.path_running:
                messages.add(
                    f"mode={mode.value} but path manager is not running"
                )
        if mode == RobotMode.IDLE and self.status.nav2_running:
            messages.add("mode=IDLE while Nav2 process is still running")
        return messages

    def log_runtime_mode_mismatches(self):
        """Emit one warning per persistent mode/process mismatch episode."""
        if self.command_active:
            return

        current = self.runtime_mode_mismatch_messages()
        resolved = self.reported_runtime_mismatches - current
        for message in sorted(resolved):
            self.get_logger().info("runtime mismatch resolved: " + message)
        for message in sorted(current - self.reported_runtime_mismatches):
            self.get_logger().warning("runtime mismatch detected: " + message)
        self.reported_runtime_mismatches = current

    def promote_localizing_mode_if_nav2_ready(self):
        """Promote ``LOCALIZING`` to ``AUTONOMOUS_READY`` when Nav2 catches up.

        This keeps ``finish_alignment`` responsive. The operator can finish
        manual alignment immediately, and the periodic status loop upgrades the
        workflow once the full Nav2 stack reaches a ready snapshot.
        """
        if self.command_active:
            return
        if self.status.mode != RobotMode.LOCALIZING:
            self.localizing_wait_log_emitted = False
            self.localizing_started_at = None
            self.nav2_ready_stable_since = None
            return
        if not self.nav2_ready_now():
            self.nav2_ready_stable_since = None
            if self._localizing_wait_timed_out():
                return
            if (
                not self.localizing_wait_log_emitted
                and (self.status.nav2_ready or self.status.amcl_pose_ready)
            ):
                self.get_logger().info(
                    "LOCALIZING is waiting for full Nav2 readiness "
                    f"(nav2_ready={self.status.nav2_ready}, "
                    f"amcl_pose_ready={self.status.amcl_pose_ready})"
                )
                self.localizing_wait_log_emitted = True
            return

        now = time.monotonic()
        if self.nav2_ready_stable_since is None:
            self.nav2_ready_stable_since = now
            return
        if (
            now - self.nav2_ready_stable_since
            < float(self.nav2_stable_ready_duration_sec)
        ):
            return

        self.status.mode = RobotMode.AUTONOMOUS_READY
        self.status.last_command = "set_autonomous_ready"
        self.status.transition_count += 1
        self.status.nav2_ready = True
        self.status.amcl_pose_ready = True
        self.localizing_wait_log_emitted = False
        self.localizing_started_at = None
        self.nav2_ready_stable_since = None
        self.notify_autonomous_ready()
        self.get_logger().info(
            "Nav2 became ready during LOCALIZING; "
            "mode set to AUTONOMOUS_READY"
        )

    def _localizing_wait_timed_out(self) -> bool:
        """Return whether LOCALIZING has waited too long for Nav2 promotion."""
        started_at = self.localizing_started_at
        if started_at is None:
            self.localizing_started_at = time.monotonic()
            return False

        timeout_sec = max(0.0, float(self.localizing_promote_timeout_sec))
        if timeout_sec <= 0.0:
            return False
        if (time.monotonic() - started_at) < timeout_sec:
            return False

        message = (
            "LOCALIZING timed out before Nav2 became fully ready; "
            "clear the error and try autonomous preparation again"
        )
        self.cancel_alignment_confirmation_request()
        self.localizing_wait_log_emitted = False
        self.set_error(message)
        return True


def main(args=None):
    """Run the mode manager node inside a multithreaded executor."""
    node = None
    executor = None
    try:
        rclpy.init(args=args)
        node = ModeManagerNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("mode_manager interrupted by SIGINT")
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"mode_manager crashed: {exc}")
        else:
            print(f"mode_manager crashed before startup completed: {exc}")
        traceback.print_exc()
    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except KeyboardInterrupt:
                pass
            except Exception as exc:
                node.get_logger().warn(
                    f"failed to remove mode_manager from executor during shutdown: {exc}"
                )

        if node is not None:
            try:
                node.shutdown_processes()
            except KeyboardInterrupt:
                node.get_logger().info(
                    "mode_manager process shutdown interrupted; continuing cleanup"
                )
            except Exception as exc:
                node.get_logger().warn(
                    f"failed to stop managed processes cleanly: {exc}"
                )

            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
            except Exception as exc:
                node.get_logger().warn(
                    f"failed to destroy mode_manager node cleanly: {exc}"
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
