# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Shared helpers for external ROS service calls.

These helpers keep workflow code focused on state transitions while centralizing
common Trigger-service call patterns such as waiting for availability, waiting
for completion, copying responses, and storing the last service error.
"""

import time

import rclpy
from rclpy.task import Future
from std_srvs.srv import Trigger


class ServiceClientMixin:
    """Provide small helpers for Trigger calls and future waiting."""

    def copy_response(self, target, source):
        """Copy one Trigger-like response into another response object.

        Args:
            target: Response object that will be returned to the caller.
            source: Internal response object whose fields should be copied.

        Returns:
            The updated ``target`` response.
        """
        target.success = bool(source.success)
        target.message = source.message
        return target

    def call_trigger_service(self, client, service_name, timeout_sec):
        """Call a Trigger service and store a readable error on failure.

        Args:
            client: ROS Trigger client instance.
            service_name: Fully qualified service name for logging.
            timeout_sec: Maximum wait duration for availability and response.

        Returns:
            ``True`` when the service call succeeds and returns success.
        """
        self.last_service_error = ""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.last_service_error = f"service unavailable: {service_name}"
            self.get_logger().error(self.last_service_error)
            return False

        future = client.call_async(Trigger.Request())
        if not self.wait_for_future(future, timeout_sec):
            self.last_service_error = f"service timeout: {service_name}"
            self.get_logger().error(self.last_service_error)
            return False

        result = future.result()
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            self.last_service_error = message
            self.get_logger().error(f"service failed: {service_name}: {message}")
            return False

        self.get_logger().info(f"service succeeded: {service_name}: {result.message}")
        return True

    def call_trigger_service_with_retry(
        self,
        client,
        service_name,
        timeout_sec,
        attempts=3,
        success_probe=None,
        initial_delay_sec=1.0,
    ):
        """Call an idempotent Trigger service with bounded backoff.

        ``success_probe`` is checked after an ambiguous timeout before another
        request is sent. This lets callers avoid a duplicate request when the
        desired runtime state is already visible despite a lost response.
        """
        attempts = max(1, int(attempts))
        delay_sec = max(0.0, float(initial_delay_sec))

        for attempt in range(1, attempts + 1):
            if self.call_trigger_service(client, service_name, timeout_sec):
                return True

            if success_probe is not None and success_probe():
                self.get_logger().warning(
                    f"service response was not confirmed, but its target state "
                    f"is ready: {service_name}"
                )
                self.last_service_error = ""
                return True

            self.get_logger().warning(
                f"service attempt {attempt}/{attempts} failed: {service_name}: "
                f"{self.last_service_error}"
            )
            if attempt < attempts:
                time.sleep(delay_sec)
                delay_sec *= 2.0
        return False

    def wait_for_future(self, future: Future, timeout_sec):
        """Wait for a ROS future until completion or timeout.

        Args:
            future: ROS future returned by an asynchronous client call.
            timeout_sec: Maximum wait duration in seconds.

        Returns:
            ``True`` if the future completes before the timeout.
        """
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)

        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            if self.get_clock().now().nanoseconds >= deadline:
                return False

        return future.done()
