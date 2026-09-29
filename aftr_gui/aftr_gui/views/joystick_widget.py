# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""AFTR GUI virtual joystick widget.

This widget only produces normalized ``x`` and ``y`` joystick values in the
``[-1.0, 1.0]`` range. The actual ``/cmd_vel`` conversion and manual-control
policy stay in ``operator_gui.py``.
"""

import math

from PyQt5.QtCore import QEvent, QPoint, QPointF, QSize, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QBrush, QPainter, QPen, QRadialGradient
from PyQt5.QtWidgets import QSizePolicy, QWidget


class JoystickWidget(QWidget):
    """Draw and emit a touch-friendly circular virtual joystick."""

    moved = pyqtSignal(float, float)

    def __init__(self, parent=None):
        """Initialize joystick geometry state and deadzone settings."""
        super().__init__(parent)
        self.setMinimumSize(340, 340)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.setFocusPolicy(Qt.NoFocus)

        self.joystick_pos = QPoint()
        self.center = QPoint()
        self.boundary_radius = 0
        self.knob_radius = 0
        self.grabbed = False
        self.deadzone = 0.05

        # The visible ring remains the motion range. The hit range is slightly
        # wider so a finger near the outline still starts the joystick.
        self.touch_hit_radius_scale = 1.12

    def sizeHint(self):
        """Prefer a large control surface suitable for a touchscreen."""
        return QSize(390, 390)

    def minimumSizeHint(self):
        """Keep the touch target large enough for reliable operator input."""
        return QSize(340, 340)

    def resizeEvent(self, event):
        """Recompute the joystick center and radii after a resize."""
        self.center = QPoint(self.width() // 2, self.height() // 2)
        self.boundary_radius = int(min(self.width(), self.height()) * 0.43)
        self.knob_radius = int(self.boundary_radius * 0.36)
        if not self.grabbed:
            self.joystick_pos = self.center
        super().resizeEvent(event)

    def paintEvent(self, event):
        """Paint the joystick background and knob."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        gradient = QRadialGradient(QPointF(self.center), float(self.boundary_radius))
        gradient.setColorAt(0.0, QColor("#FFFFFF"))
        gradient.setColorAt(0.65, QColor("#EEF3F8"))
        gradient.setColorAt(1.0, QColor("#D8E0EA"))

        ring_color = QColor("#2563EB") if self.grabbed else QColor("#94A3B8")
        painter.setPen(QPen(ring_color, 5 if self.grabbed else 4))
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(self.center, self.boundary_radius, self.boundary_radius)

        knob_color = QColor("#1D4ED8") if self.grabbed else QColor("#64748B")
        painter.setPen(
            QPen(QColor("#DBEAFE") if self.grabbed else QColor("#F8FAFC"), 4)
        )
        painter.setBrush(QBrush(knob_color))
        painter.drawEllipse(self.joystick_pos, self.knob_radius, self.knob_radius)
        painter.end()

    def event(self, event):
        """Handle native touch events instead of relying on mouse synthesis."""
        if event.type() in (QEvent.TouchBegin, QEvent.TouchUpdate, QEvent.TouchEnd):
            points = event.touchPoints()
            if event.type() == QEvent.TouchEnd or not points:
                self.reset_to_center()
                event.accept()
                return True

            position = points[0].pos().toPoint()
            if event.type() == QEvent.TouchBegin:
                if not self._inside_touch_hit_area(position):
                    event.ignore()
                    return False
                self.grabbed = True

            if self.grabbed:
                self.move_knob(position)
                event.accept()
                return True

        if event.type() == QEvent.TouchCancel:
            self.reset_to_center()
            event.accept()
            return True

        return super().event(event)

    def mousePressEvent(self, event):
        """Start from anywhere inside the enlarged circular touch area."""
        if event.button() != Qt.LeftButton:
            event.ignore()
            return
        if not self._inside_touch_hit_area(event.pos()):
            event.ignore()
            return

        self.grabbed = True
        self.move_knob(event.pos())
        event.accept()

    def mouseMoveEvent(self, event):
        """Move the knob while the pointer is dragging it."""
        if not self.grabbed:
            event.ignore()
            return
        self.move_knob(event.pos())
        event.accept()

    def mouseReleaseEvent(self, event):
        """Return the knob to the center and emit a zero command."""
        if event.button() == Qt.LeftButton:
            self.reset_to_center()
            event.accept()
            return
        event.ignore()

    def leaveEvent(self, event):
        """Stop safely if a mouse drag leaves the widget unexpectedly."""
        if self.grabbed and not self.underMouse():
            self.reset_to_center()
        super().leaveEvent(event)

    def hideEvent(self, event):
        """Stop whenever the page containing the joystick is hidden."""
        self.reset_to_center()
        super().hideEvent(event)

    def changeEvent(self, event):
        """Stop immediately when manual control disables the widget."""
        if event.type() == QEvent.EnabledChange and not self.isEnabled():
            self.reset_to_center()
        super().changeEvent(event)

    def _inside_touch_hit_area(self, pos):
        """Return whether a press is close enough to the visible control ring."""
        distance = math.hypot(
            pos.x() - self.center.x(),
            pos.y() - self.center.y(),
        )
        return distance <= self.boundary_radius * self.touch_hit_radius_scale

    def reset_to_center(self):
        """Release the joystick and emit the mandatory neutral command."""
        was_active = self.grabbed or self.joystick_pos != self.center
        self.grabbed = False
        self.joystick_pos = self.center
        if was_active:
            self.moved.emit(0.0, 0.0)
        self.update()

    def move_knob(self, pos):
        """Clamp the knob inside the visible ring and emit normalized axes."""
        dx = pos.x() - self.center.x()
        dy = pos.y() - self.center.y()
        distance = math.hypot(dx, dy)
        if distance > self.boundary_radius:
            angle = math.atan2(dy, dx)
            pos = QPoint(
                int(self.center.x() + math.cos(angle) * self.boundary_radius),
                int(self.center.y() + math.sin(angle) * self.boundary_radius),
            )

        self.joystick_pos = pos
        norm_x = (self.joystick_pos.x() - self.center.x()) / max(
            self.boundary_radius,
            1,
        )
        norm_y = -(self.joystick_pos.y() - self.center.y()) / max(
            self.boundary_radius,
            1,
        )
        if abs(norm_x) < self.deadzone:
            norm_x = 0.0
        if abs(norm_y) < self.deadzone:
            norm_y = 0.0
        self.moved.emit(float(norm_x), float(norm_y))
        self.update()
