# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Lightweight map view for the operator GUI.

This widget gives operators a compact map view during autonomous driving. It
intentionally draws only ``/map``, ``/amcl_pose``, and ``/planned_path``
instead of embedding a full RViz feature set.
"""

import math

from PyQt5.QtCore import QPointF
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPainter
from PyQt5.QtGui import QPainterPath
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import QWidget


class MapViewWidget(QWidget):
    """Render a simplified occupancy map, robot pose, and planned path."""

    def __init__(self, parent=None):
        """Initialize cached map, pose, and path state."""
        super().__init__(parent)
        self.setMinimumSize(240, 160)
        self.map_image = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.robot_pose = None
        self.path_points = []
        self.setAutoFillBackground(False)

    def update_map(self, msg):
        """Convert an ``OccupancyGrid`` message into a cached ``QImage``."""
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0:
            return

        image = QImage(width, height, QImage.Format_RGB32)
        data = msg.data
        for y in range(height):
            # ROS maps start at the bottom-left, while QImage starts at the
            # top-left, so the y axis is flipped while copying pixels.
            src_y = height - 1 - y
            for x in range(width):
                value = data[src_y * width + x]
                if value < 0:
                    color = QColor(245, 247, 251)
                elif value >= 65:
                    color = QColor(38, 38, 38)
                else:
                    color = QColor(255, 255, 255)
                image.setPixelColor(x, y, color)

        self.map_image = image
        self.map_width = width
        self.map_height = height
        self.resolution = float(msg.info.resolution)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.update()

    def update_pose(self, msg):
        """Cache only the robot position and yaw from an AMCL pose message."""
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.robot_pose = (float(pose.position.x), float(pose.position.y), yaw)
        self.update()

    def update_path(self, msg):
        """Cache the planned path as world-frame points."""
        points = []
        for pose_stamped in msg.poses:
            position = pose_stamped.pose.position
            points.append((float(position.x), float(position.y)))
        self.path_points = points
        self.update()

    def paintEvent(self, event):
        """Paint the cached map, path, and robot pose."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(255, 255, 255))

        if self.map_image is None:
            painter.setPen(QColor(51, 65, 85))
            painter.drawText(
                self.rect(),
                Qt.AlignCenter,
                "지도 데이터를 준비 중입니다.",
            )
            return

        target = self.map_target_rect()
        painter.drawImage(target, self.map_image)
        painter.setPen(QPen(QColor(216, 224, 234), 1))
        painter.drawRect(target)

        if len(self.path_points) >= 2:
            self.draw_path(painter, target)

        if self.robot_pose is not None:
            self.draw_robot(painter, target)

    def map_target_rect(self):
        """Return the fitted target rectangle that preserves map aspect ratio."""
        view = self.rect().adjusted(8, 8, -8, -8)
        image_ratio = self.map_width / max(self.map_height, 1)
        view_ratio = view.width() / max(view.height(), 1)
        if image_ratio > view_ratio:
            width = view.width()
            height = int(width / image_ratio)
        else:
            height = view.height()
            width = int(height * image_ratio)
        x = view.x() + (view.width() - width) // 2
        y = view.y() + (view.height() - height) // 2
        return view.__class__(x, y, width, height)

    def world_to_view(self, world_x, world_y, target):
        """Convert one map-frame point into widget coordinates."""
        map_x = (world_x - self.origin_x) / self.resolution
        map_y = (world_y - self.origin_y) / self.resolution
        px = target.x() + map_x / max(self.map_width, 1) * target.width()
        py = target.y() + (1.0 - map_y / max(self.map_height, 1)) * target.height()
        return QPointF(px, py)

    def draw_robot(self, painter, target):
        """Draw the robot position marker and heading line."""
        x, y, yaw = self.robot_pose
        center = self.world_to_view(x, y, target)
        radius = max(5.0, min(target.width(), target.height()) * 0.018)

        painter.setBrush(QColor(37, 99, 235))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(center, radius, radius)

        heading_len = radius * 2.2
        end = QPointF(
            center.x() + math.cos(yaw) * heading_len,
            center.y() - math.sin(yaw) * heading_len,
        )
        painter.setPen(QPen(QColor(5, 150, 105), 3))
        painter.drawLine(center, end)

    def draw_path(self, painter, target):
        """Draw the cached path and mark its start and goal points."""
        path = QPainterPath()
        first_x, first_y = self.path_points[0]
        path.moveTo(self.world_to_view(first_x, first_y, target))
        for world_x, world_y in self.path_points[1:]:
            path.lineTo(self.world_to_view(world_x, world_y, target))

        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(QColor(59, 130, 246), 3, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawPath(path)

        start = self.world_to_view(*self.path_points[0], target)
        goal = self.world_to_view(*self.path_points[-1], target)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(16, 185, 129))
        painter.drawEllipse(start, 4.5, 4.5)
        painter.setBrush(QColor(239, 68, 68))
        painter.drawEllipse(goal, 5.5, 5.5)
