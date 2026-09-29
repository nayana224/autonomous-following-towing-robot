# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""ROS bridge layer for AFTR GUI commands and status subscriptions."""

from dataclasses import dataclass
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from PyQt5.QtCore import QObject
from PyQt5.QtCore import QThread
from PyQt5.QtCore import pyqtSignal
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class GuiCommand:
    """Describe one GUI command and the service it calls.

    Attributes:
        key: Stable GUI-side command key.
        label: Human-readable command label shown in the UI.
        service_name: Fully qualified Trigger service name.
        timeout_sec: Maximum wait time for the service response.
    """

    key: str
    label: str
    service_name: str
    timeout_sec: float = 90.0


COMMANDS = {
    # GUI command keys must stay aligned with mode_manager allowed_commands.
    # When a service name changes, update both mode_manager and operator_gui.py.
    "start_base": GuiCommand(
        "start_base",
        "기본 시스템 시작",
        "/mode_manager/internal/start_base",
        timeout_sec=30.0,
    ),
    "follow": GuiCommand(
        "follow",
        "추종 모드 시작",
        "/mode_manager/command/start_follow",
    ),
    "recording_follow": GuiCommand(
        "recording_follow",
        "경로 저장 추종 시작",
        "/mode_manager/command/start_recording_follow",
    ),
    "finish_recording": GuiCommand(
        "finish_recording",
        "경로 저장 종료 및 정렬",
        "/mode_manager/command/finish_recording_for_alignment",
        timeout_sec=180.0,
    ),
    "prepare_autonomous": GuiCommand(
        "prepare_autonomous",
        "자율주행 준비",
        "/mode_manager/command/prepare_autonomous",
        timeout_sec=120.0,
    ),
    "alignment": GuiCommand(
        "alignment",
        "정렬 시작",
        "/mode_manager/command/start_alignment",
        timeout_sec=15.0,
    ),
    "skip_alignment": GuiCommand(
        "skip_alignment",
        "정렬 없이 진행",
        "/mode_manager/command/skip_alignment",
        timeout_sec=60.0,
    ),
    "finish_alignment": GuiCommand(
        "finish_alignment",
        "정렬 완료",
        "/mode_manager/command/finish_alignment",
        timeout_sec=60.0,
    ),
    "path_reverse_auto": GuiCommand(
        "path_reverse_auto",
        "복귀 주행",
        "/mode_manager/command/path_reverse_auto",
        timeout_sec=150.0,
    ),
    "path_forward_auto": GuiCommand(
        "path_forward_auto",
        "정방향 주행",
        "/mode_manager/command/path_forward_auto",
        timeout_sec=150.0,
    ),
    "stop": GuiCommand(
        "stop",
        "주행 중단",
        "/mode_manager/command/stop",
    ),
    "clear_error": GuiCommand(
        "clear_error",
        "오류 해제",
        "/mode_manager/command/clear_error",
        timeout_sec=10.0,
    ),
}


class ModeManagerClient(QObject):
    """ROS 2 bridge used by the PyQt operator GUI.

    ROS callbacks run in a dedicated executor thread and forward data to the
    GUI thread through Qt signals. This keeps widget updates out of ROS
    callback threads.
    """

    mode_status = pyqtSignal(str)
    odom_velocity = pyqtSignal(float, float)
    map_received = pyqtSignal(object)
    amcl_pose_received = pyqtSignal(object)
    planned_path_received = pyqtSignal(object)

    def __init__(self):
        """Create the ROS bridge node, subscriptions, and service clients."""
        super().__init__()
        rclpy.init(args=None)
        self.node = Node("mdbot_operator_gui")

        # Keep ROS subscription and service callbacks off the GUI thread.
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.command_clients = {
            key: self.node.create_client(Trigger, command.service_name)
            for key, command in COMMANDS.items()
        }
        self.manual_cmd_publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)

        # /mode_manager/status is the single status input for the GUI.
        self.node.create_subscription(
            String,
            "/mode_manager/status",
            self.handle_mode_status_message,
            10,
        )
        self.node.create_subscription(
            Odometry,
            "/odom",
            self.handle_odom_message,
            10,
        )

        # These topics feed the embedded map widget used during autonomous mode.
        self.node.create_subscription(
            OccupancyGrid,
            "/map",
            self.handle_map_message,
            1,
        )
        self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.handle_amcl_pose_message,
            10,
        )
        self.node.create_subscription(
            Path,
            "/planned_path",
            self.handle_planned_path_message,
            10,
        )
        self._running = True
        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.executor_thread.start()

    def handle_mode_status_message(self, msg):
        """Forward the raw mode-manager status string to the GUI thread."""
        self.mode_status.emit(msg.data)

    def handle_odom_message(self, msg):
        """Extract linear and angular velocity from odometry for the GUI."""
        self.odom_velocity.emit(
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z),
        )

    def handle_map_message(self, msg):
        """Forward a map message to the embedded map widget."""
        self.map_received.emit(msg)

    def handle_amcl_pose_message(self, msg):
        """Forward the latest AMCL pose to the embedded map widget."""
        self.amcl_pose_received.emit(msg)

    def handle_planned_path_message(self, msg):
        """Forward the current planned path to the embedded map widget."""
        self.planned_path_received.emit(msg)

    def call_command(self, command_key):
        """Call one configured command service.

        Args:
            command_key: GUI command key from ``COMMANDS``.

        Returns:
            Tuple of ``(success, message)`` returned by the Trigger service.
        """
        command = COMMANDS[command_key]
        client = self.command_clients[command_key]
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"{command.service_name} service is not available"

        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + float(command.timeout_sec)
        while time.monotonic() < deadline:
            if future.done():
                response = future.result()
                if response is None:
                    return False, f"{command.service_name} returned no response"
                return bool(response.success), response.message
            time.sleep(0.05)
        return False, f"{command.service_name} service call timed out"

    def publish_cmd(self, linear_x, angular_z):
        """Publish one manual velocity command.

        Args:
            linear_x: Linear x velocity in meters per second.
            angular_z: Angular z velocity in radians per second.
        """
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.manual_cmd_publisher.publish(msg)

    def stop(self):
        """Stop ROS executor and destroy the GUI node."""
        if not self._running:
            return
        self._running = False
        self.executor.shutdown()
        if self.executor_thread.is_alive():
            self.executor_thread.join(timeout=1.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class CommandWorker(QThread):
    """Run one mode-manager service call outside the GUI thread."""

    finished = pyqtSignal(str, bool, str)

    def __init__(self, client, key):
        """Store the bridge client and command key for one async call."""
        super().__init__()
        self.client = client
        self.key = key
        self.label = COMMANDS[key].label

    def run(self):
        """Execute one command call and emit the result back to the GUI."""
        success, message = self.client.call_command(self.key)
        self.finished.emit(self.label, success, message)
