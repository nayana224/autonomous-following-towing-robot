# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Sequential startup and recoverable map-save handling for AFTR."""

import hashlib
import os
import subprocess
import time
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor

from aftr_mode_manager.hardened_fall_mode_manager_node import (
    HardenedFallModeManagerNode,
)
from aftr_mode_manager.mode_state import RobotMode


class SequentialFallModeManagerNode(HardenedFallModeManagerNode):
    """Serialize heavy process transitions and make map saving recoverable."""

    def _declare_runtime_parameters(self):
        super()._declare_runtime_parameters()
        self.heavy_process_settle_sec = float(
            self.declare_parameter("heavy_process_settle_sec", 2.0).value
        )
        self.map_message_freshness_sec = float(
            self.declare_parameter("map_message_freshness_sec", 3.0).value
        )
        self.map_save_attempts = int(
            self.declare_parameter("map_save_attempts", 2).value
        )
        self.map_saver_internal_timeout_sec = float(
            self.declare_parameter("map_saver_internal_timeout_sec", 10.0).value
        )
        self.initial_pose_service_attempts = int(
            self.declare_parameter("initial_pose_service_attempts", 3).value
        )

    def _wait_for_recent_map(self, timeout_sec):
        """Wait for a map message that is recent enough for a save request."""
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            received_at = self.last_map_message_at
            if (
                received_at is not None
                and time.monotonic() - received_at <= self.map_message_freshness_sec
            ):
                return True
            time.sleep(0.1)
        return False

    def _saved_map_output_signature(self):
        """Return metadata for the saved map pair, or ``None`` if incomplete."""
        map_yaml_path = self.normalized_map_save_file() + ".yaml"
        if not os.path.isfile(map_yaml_path):
            return None

        try:
            image_value = ""
            with open(map_yaml_path, "r", encoding="utf-8") as map_file:
                for raw_line in map_file:
                    stripped_line = raw_line.strip()
                    if not stripped_line or stripped_line.startswith("#"):
                        continue
                    key, separator, value = stripped_line.partition(":")
                    if separator and key.strip() == "image":
                        image_value = value.strip().strip("\"'")
                        break
            if not image_value:
                return None
            image_path = os.path.expanduser(image_value)
            if not os.path.isabs(image_path):
                image_path = os.path.join(
                    os.path.dirname(map_yaml_path),
                    image_path,
                )
            image_path = os.path.normpath(image_path)
            yaml_stat = os.stat(map_yaml_path)
            image_stat = os.stat(image_path)
            with open(map_yaml_path, "rb") as map_yaml_file:
                yaml_digest = hashlib.sha256(map_yaml_file.read()).hexdigest()
            with open(image_path, "rb") as map_image_file:
                image_digest = hashlib.sha256(map_image_file.read()).hexdigest()
        except OSError:
            return None

        return (
            map_yaml_path,
            yaml_stat.st_mtime_ns,
            yaml_stat.st_size,
            yaml_digest,
            image_path,
            image_stat.st_mtime_ns,
            image_stat.st_size,
            image_digest,
        )

    def save_slam_map(self):
        """Save the map after fresh-data validation, with one bounded retry."""
        if not self.slam_process.is_running():
            self.get_logger().warning("map save skipped: SLAM process is not running")
            return False

        attempts = max(1, self.map_save_attempts)
        map_prefix = self.normalized_map_save_file()
        map_dir = os.path.dirname(map_prefix)
        if map_dir:
            os.makedirs(map_dir, exist_ok=True)

        for attempt in range(1, attempts + 1):
            self.set_operator_message(
                f"지도를 저장하고 있습니다. ({attempt}/{attempts})",
                publish=True,
            )
            if not self._wait_for_recent_map(self.slam_ready_timeout_sec):
                self.get_logger().warning(
                    f"map save attempt {attempt}: no recent /map message"
                )
                continue

            signature_before_save = self._saved_map_output_signature()
            command = [
                "ros2",
                "run",
                "nav2_map_server",
                "map_saver_cli",
                "-f",
                map_prefix,
                "--ros-args",
                "-p",
                f"save_map_timeout:={self.map_saver_internal_timeout_sec}",
            ]
            self.get_logger().info(
                f"saving SLAM map to {map_prefix} (attempt {attempt}/{attempts})"
            )
            try:
                env = os.environ.copy()
                env["ROS_LOG_DIR"] = "/tmp"
                result = subprocess.run(
                    command,
                    capture_output=True,
                    check=False,
                    text=True,
                    timeout=max(
                        float(self.map_save_timeout_sec),
                        self.map_saver_internal_timeout_sec + 5.0,
                    ),
                    env=env,
                )
            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    f"map saver attempt {attempt} timed out"
                )
                result = None
            except OSError as exc:
                self.get_logger().warning(
                    f"map saver attempt {attempt} could not start: {exc}"
                )
                result = None

            if result is not None:
                if result.stdout:
                    self.get_logger().info(result.stdout.strip())
                if result.stderr:
                    self.get_logger().warning(result.stderr.strip())
                if result.returncode == 0:
                    signature_after_save = self._saved_map_output_signature()
                    if (
                        signature_after_save is not None
                        and signature_after_save != signature_before_save
                    ):
                        self.get_logger().info(f"SLAM map saved to {map_prefix}")
                        return True
                    self.get_logger().warning(
                        "map saver returned success without producing a new "
                        "complete map pair"
                    )
                self.get_logger().warning(
                    f"map saver attempt {attempt} failed with exit code "
                    f"{result.returncode}"
                )

            if attempt < attempts:
                self.get_logger().info("waiting for a fresh map before retry")
                time.sleep(self.heavy_process_settle_sec)

        return False

    def _save_recording_outputs_for_alignment(self, response):
        """Save path, a fresh map, and pose without accepting stale map files."""
        self.set_operator_message(
            "경로 저장을 마무리하고 있습니다.",
            publish=True,
        )
        if not self.call_trigger_service_with_retry(
            self.path_stop_record_client,
            "/path_manager/stop_record",
            timeout_sec=15.0,
            attempts=2,
            success_probe=lambda: (
                bool(self.latest_path_status)
                and not bool(self.latest_path_status.get("recording", True))
            ),
        ):
            self.fail_response(response, "failed to stop path recording")
            return False

        # Give path-manager callbacks and SLAM publishing time to settle before
        # launching the separate map_saver process.
        time.sleep(self.heavy_process_settle_sec)
        map_saved = self.save_slam_map()
        if not map_saved:
            self.fail_response(
                response,
                "새 SLAM 지도 저장을 확인하지 못했습니다. 이전 지도와 새 "
                "경로가 섞이지 않도록 자율주행 준비를 중단합니다.",
            )
            return False

        self.set_operator_message(
            "현재 위치를 저장하고 있습니다.",
            publish=True,
        )
        if not self.call_trigger_service_with_retry(
            self.path_save_pose_client,
            "/path_manager/save_pose",
            timeout_sec=8.0,
            attempts=2,
        ):
            self.fail_response(response, "failed to save alignment pose")
            return False

        response.success = True
        response.message = "recording outputs and fresh map saved"
        return True

    def _wait_for_process_exit(self, process, label, timeout_sec):
        """Wait until a managed process is fully gone before starting another."""
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if not process.is_running():
                time.sleep(self.heavy_process_settle_sec)
                return True
            time.sleep(0.1)
        self.get_logger().error(f"{label} did not fully exit before timeout")
        return False

    def _initial_pose_runtime_ready(self):
        """Return whether localization is already usable after an ambiguous call."""
        return (
            self.amcl_pose_message_readiness.has_recent_message(
                self.amcl_pose_freshness_sec,
            )
            and self.localization_transform_ready(timeout_sec=0.05)
        )

    def start_localizing(self, response):
        """Stop SLAM completely, then start Nav2 and publish initial pose."""
        if not self.ensure_base_running(response):
            return response
        if not self.ensure_path_manager_running(response):
            return response

        self._cancel_active_path_follow(timeout_sec=2.0)
        if not self._stop_slam_before_localization(response):
            return response

        self.set_operator_message("위치 추정 시스템을 시작하고 있습니다.", publish=True)
        if not self.start_nav2_for_localization(response):
            # Keep a live process intact so a delayed lifecycle/service startup
            # can be checked again by the next operator request.
            self.status.nav2_running = self.nav2_process.is_running()
            self.status.nav2_ready = False
            self.status.amcl_pose_ready = False
            self.get_logger().warning(
                "localization is not ready yet; Nav2 was left running for a "
                "bounded retry: " + self.nav2_process.describe()
            )
            return response

        if not self._publish_initial_pose_for_localization(response):
            return response

        return self.request_mode(
            response,
            RobotMode.LOCALIZING,
            "start_localizing",
        )

    def _stop_slam_before_localization(self, response):
        if not self.slam_process.is_running():
            return True

        self.set_operator_message("SLAM을 종료하고 있습니다.", publish=True)
        if not self.slam_process.stop(self.shutdown_timeout_sec):
            self.fail_response(response, "SLAM shutdown timed out")
            return False

        self.status.slam_running = False
        self.status.slam_ready = False
        if not self._wait_for_process_exit(
            self.slam_process,
            "SLAM",
            self.shutdown_timeout_sec,
        ):
            self.fail_response(
                response,
                "SLAM did not fully exit before Nav2 startup",
            )
            return False
        return True

    def _publish_initial_pose_for_localization(self, response):
        time.sleep(max(
            float(self.nav2_initial_pose_delay_sec),
            self.heavy_process_settle_sec,
        ))
        self.amcl_pose_message_readiness.reset()
        if not self.call_trigger_service_with_retry(
            self.path_publish_initial_pose_client,
            "/path_manager/publish_initial_pose",
            timeout_sec=6.0,
            attempts=self.initial_pose_service_attempts,
            success_probe=self._initial_pose_runtime_ready,
        ):
            self.status.nav2_running = self.nav2_process.is_running()
            self.status.nav2_ready = False
            self.status.amcl_pose_ready = False
            self.fail_response(response, "failed to publish initial pose")
            return False

        is_ready, missing_topics = self.amcl_pose_readiness.wait_until_ready(
            self.amcl_pose_ready_timeout_sec,
        )
        self.status.amcl_pose_ready = is_ready
        if is_ready:
            self.get_logger().info("AMCL pose is visible after initial pose")
        else:
            self.get_logger().warning(
                "AMCL pose is not visible yet after initial pose; missing topics: "
                + ", ".join(missing_topics)
            )
        return True


def main(args=None):
    """Run the sequential fall-aware mode manager."""
    node = None
    executor = None
    try:
        rclpy.init(args=args)
        node = SequentialFallModeManagerNode()
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
            except Exception as exc:
                node.get_logger().warning(
                    f"failed to remove mode_manager during shutdown: {exc}"
                )
        if node is not None:
            try:
                node.shutdown_processes()
            except Exception as exc:
                node.get_logger().warning(
                    f"failed to stop managed processes cleanly: {exc}"
                )
            try:
                node.destroy_node()
            except Exception as exc:
                node.get_logger().warning(
                    f"failed to destroy mode_manager node cleanly: {exc}"
                )
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
