# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Readiness helpers for ROS graph state and message activity.

The mode manager uses these helpers to decide whether a launched runtime
component is merely visible in the graph or actually usable for the next
workflow step.
"""

import time


class MessageReadiness:
    """Track whether a topic has published a fresh message recently."""

    def __init__(self, node, msg_type, topic_name, qos_profile, callback_group=None):
        """Subscribe to a topic and record message arrival timestamps.

        Args:
            node: ROS node used to create the subscription.
            msg_type: ROS message type of the monitored topic.
            topic_name: Fully qualified topic name to monitor.
            qos_profile: QoS profile passed to ``create_subscription``.
            callback_group: Optional callback group for the subscription.
        """
        self.node = node
        self.topic_name = topic_name
        self.message_count = 0
        self.last_message_time = None
        self.subscription = node.create_subscription(
            msg_type,
            topic_name,
            self.message_callback,
            qos_profile,
            callback_group=callback_group,
        )

    def message_callback(self, _msg):
        """Record the arrival of one more message."""
        self.message_count += 1
        self.last_message_time = time.monotonic()

    def wait_for_new_message(self, timeout_sec):
        """Wait for at least one new message after the call begins.

        Args:
            timeout_sec: Maximum wait duration in seconds.

        Returns:
            ``True`` if a new message arrives before the timeout.
        """
        initial_count = self.message_count
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            if self.message_count > initial_count:
                return True
            time.sleep(0.1)

        return False

    def has_recent_message(self, max_age_sec):
        """Return whether the topic published within the allowed age.

        Args:
            max_age_sec: Maximum allowed message age in seconds.

        Returns:
            ``True`` if the last observed message is recent enough.
        """
        if self.last_message_time is None:
            return False
        return (time.monotonic() - self.last_message_time) <= max_age_sec

    def has_message(self):
        """Return whether at least one message was observed since the last reset."""
        return self.last_message_time is not None

    def reset(self):
        """Forget earlier messages before a new runtime readiness sequence."""
        self.message_count = 0
        self.last_message_time = None


class TopicReadiness:
    """Check whether required ROS topics are visible in the graph."""

    def __init__(self, node, required_topics):
        """Store the node and topic names used for readiness checks.

        Args:
            node: ROS node used to inspect the current ROS graph.
            required_topics: Iterable of required topic names.
        """
        self.node = node
        self.required_topics = tuple(required_topics)

    def missing_topics(self):
        """Return required topics that are not visible in the ROS graph.

        Returns:
            List of missing topic names.
        """
        visible_topics = {
            topic_name
            for topic_name, _topic_types in self.node.get_topic_names_and_types()
        }
        return [
            topic_name
            for topic_name in self.required_topics
            if topic_name not in visible_topics
        ]

    def wait_until_ready(self, timeout_sec):
        """Wait until all required topics are visible.

        Args:
            timeout_sec: Maximum wait duration in seconds.

        Returns:
            Tuple ``(is_ready, missing_topics)``.
        """
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            missing = self.missing_topics()
            if not missing:
                return True, []
            time.sleep(0.2)

        return False, self.missing_topics()


class NodeReadiness:
    """Check whether required ROS nodes are visible in the graph."""

    def __init__(self, node, required_nodes):
        """Store the node and required fully qualified node names.

        Args:
            node: ROS node used to inspect the current ROS graph.
            required_nodes: Iterable of required fully qualified node names.
        """
        self.node = node
        self.required_nodes = tuple(required_nodes)

    def visible_nodes(self):
        """Return fully qualified node names visible in the ROS graph.

        Returns:
            Set of visible fully qualified node names.
        """
        visible = set()
        for node_name, namespace in self.node.get_node_names_and_namespaces():
            if namespace == "/":
                visible.add(f"/{node_name}")
            else:
                visible.add(f"{namespace.rstrip('/')}/{node_name}")
        return visible

    def missing_nodes(self):
        """Return required nodes that are not visible in the ROS graph.

        Returns:
            List of missing fully qualified node names.
        """
        visible = self.visible_nodes()
        return [
            node_name
            for node_name in self.required_nodes
            if node_name not in visible
        ]

    def wait_until_ready(self, timeout_sec):
        """Wait until all required nodes are visible.

        Args:
            timeout_sec: Maximum wait duration in seconds.

        Returns:
            Tuple ``(is_ready, missing_nodes)``.
        """
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            missing = self.missing_nodes()
            if not missing:
                return True, []
            time.sleep(0.2)

        return False, self.missing_nodes()
