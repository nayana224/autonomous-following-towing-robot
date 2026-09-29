#!/usr/bin/env python3
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

"""Mode manager with reusable Nav2 startup and saved-map validation."""

import os
import time
import traceback

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

from aftr_mode_manager.sequential_fall_mode_manager_node import (
    SequentialFallModeManagerNode,
)
from aftr_mode_manager.runtime.instance_lock import ModeManagerInstanceLock


class ValidatedMapModeManagerNode(SequentialFallModeManagerNode):
    """Prevent Nav2 from starting without a complete saved map."""

    @staticmethod
    def _read_map_image_entry(map_yaml_path: str) -> str:
        """Read the standard Nav2 ``image:`` entry without another dependency."""
        with open(map_yaml_path, "r", encoding="utf-8") as map_file:
            for raw_line in map_file:
                stripped_line = raw_line.strip()
                if not stripped_line or stripped_line.startswith("#"):
                    continue
                key, separator, value = stripped_line.partition(":")
                if separator and key.strip() == "image":
                    return value.strip().strip("\"'")
        return ""

    def _saved_map_validation_error(self) -> str:
        """Return an error message when the YAML or referenced image is missing."""
        map_yaml_path = self.normalized_map_save_file() + ".yaml"
        if not os.path.isfile(map_yaml_path):
            return f"saved map YAML does not exist: {map_yaml_path}"

        try:
            image_value = self._read_map_image_entry(map_yaml_path)
        except OSError as exc:
            return f"failed to read saved map YAML: {exc}"

        if not image_value:
            return f"saved map YAML has no image entry: {map_yaml_path}"

        image_path = os.path.expanduser(image_value)
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)
        image_path = os.path.normpath(image_path)

        if not os.path.isfile(image_path):
            return f"saved map image does not exist: {image_path}"
        return ""

    def _saved_map_signature(self):
        """Return file metadata used to distinguish a newly saved map."""
        map_yaml_path = self.normalized_map_save_file() + ".yaml"
        image_value = self._read_map_image_entry(map_yaml_path)
        image_path = os.path.expanduser(image_value)
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)
        image_path = os.path.normpath(image_path)

        yaml_stat = os.stat(map_yaml_path)
        image_stat = os.stat(image_path)
        return (
            map_yaml_path,
            yaml_stat.st_mtime_ns,
            yaml_stat.st_size,
            image_path,
            image_stat.st_mtime_ns,
            image_stat.st_size,
        )

    def _prepare_existing_nav2_before_localization(
        self,
        response,
        map_signature,
    ) -> bool:
        """Reuse only Nav2 instances known to have loaded the current map."""
        if not self.nav2_process.is_running():
            return True

        loaded_signature = getattr(self, "nav2_loaded_map_signature", None)
        if loaded_signature == map_signature:
            self.get_logger().info(
                "existing Nav2 process uses the current saved map; reusing it "
                "without restart; " + self.nav2_process.describe()
            )
            return True

        self.log_nav2_stop_request("saved map changed before localization")
        if not self.nav2_process.stop(self.shutdown_timeout_sec):
            self.fail_response(
                response,
                "existing Nav2 process did not stop after the saved map changed",
            )
            return False

        self.status.nav2_running = False
        self.status.nav2_ready = False
        self.status.amcl_pose_ready = False
        time.sleep(max(0.2, float(self.heavy_process_settle_sec)))
        return True

    def start_localizing(self, response):
        """Validate map files before starting or rechecking localization."""
        validation_error = self._saved_map_validation_error()
        if validation_error:
            return self.fail_response(
                response,
                "cannot start localization: " + validation_error,
            )

        map_signature = self._saved_map_signature()
        if not self._prepare_existing_nav2_before_localization(
            response,
            map_signature,
        ):
            return response

        result = super().start_localizing(response)
        if self.nav2_process.is_running():
            self.nav2_loaded_map_signature = map_signature
        return result


def main(args=None):
    """Run the mode manager with current-map validation."""
    node = None
    executor = None
    instance_lock = ModeManagerInstanceLock()
    try:
        instance_lock.acquire()
        rclpy.init(args=args)
        node = ValidatedMapModeManagerNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("mode_manager interrupted by SIGINT")
    except ExternalShutdownException:
        if node is not None:
            node.get_logger().warning("mode_manager requested guarded shutdown")
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
        instance_lock.release()


if __name__ == "__main__":
    main()
