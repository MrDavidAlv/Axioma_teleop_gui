# Copyright 2026 user
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QLinearGradient,
    QPainter,
    QPen,
)
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class JoystickPad(QWidget):
    """Widget personalizado de joystick virtual."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(250, 250)
        self._handle_pos = QPointF(0.0, 0.0)  # normalizado [-1, 1]
        self._dragging = False
        self.on_move = None  # callback(linear, angular)
        self.on_release = None  # callback()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        side = min(self.width(), self.height())
        cx, cy = self.width() / 2.0, self.height() / 2.0
        radius = side / 2.0 - 10

        # Background circle
        painter.setBrush(QBrush(QColor(50, 50, 50)))
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # Crosshair
        painter.setPen(QPen(QColor(100, 100, 100), 1, Qt.DashLine))
        painter.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
        painter.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))

        # Handle
        hx = cx + self._handle_pos.x() * radius
        hy = cy + self._handle_pos.y() * radius
        handle_r = 20

        gradient = QLinearGradient(hx - handle_r, hy - handle_r,
                                   hx + handle_r, hy + handle_r)
        gradient.setColorAt(0, QColor(100, 150, 255))
        gradient.setColorAt(1, QColor(30, 80, 200))
        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(QColor(200, 200, 255), 2))
        painter.drawEllipse(QPointF(hx, hy), handle_r, handle_r)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._update_handle(event.pos())

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._update_handle(event.pos())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = False
            self._handle_pos = QPointF(0.0, 0.0)
            self.update()
            if self.on_release:
                self.on_release()

    def _update_handle(self, pos):
        side = min(self.width(), self.height())
        cx, cy = self.width() / 2.0, self.height() / 2.0
        radius = side / 2.0 - 10

        dx = (pos.x() - cx) / radius
        dy = (pos.y() - cy) / radius

        # Clamp to unit circle
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > 1.0:
            dx /= dist
            dy /= dist

        self._handle_pos = QPointF(dx, dy)
        self.update()

        if self.on_move:
            # Y invertido: arriba = forward positivo
            self.on_move(-dy, -dx)


class JoystickMode(QWidget):
    """Modo de teleoperación con joystick virtual."""

    def __init__(self, node):
        super().__init__()
        self._node = node

        layout = QVBoxLayout(self)

        # --- Speed slider ---
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel('Max speed:'))
        self._speed_slider = QSlider(Qt.Horizontal)
        self._speed_slider.setRange(10, 200)
        self._speed_slider.setValue(50)
        self._speed_slider.setTickInterval(10)
        self._speed_slider.setTickPosition(QSlider.TicksBelow)
        speed_layout.addWidget(self._speed_slider)
        self._speed_label = QLabel('0.50 m/s')
        self._speed_slider.valueChanged.connect(
            lambda v: self._speed_label.setText(f'{v / 100.0:.2f} m/s')
        )
        speed_layout.addWidget(self._speed_label)
        layout.addLayout(speed_layout)

        # --- Joystick pad ---
        self._pad = JoystickPad()
        self._pad.on_move = self._on_joystick_move
        self._pad.on_release = self._on_joystick_release
        layout.addWidget(self._pad, stretch=1)

        # --- Value labels ---
        values_layout = QHBoxLayout()
        self._linear_label = QLabel('Linear: 0.00')
        self._angular_label = QLabel('Angular: 0.00')
        self._linear_label.setStyleSheet('font-size: 14px;')
        self._angular_label.setStyleSheet('font-size: 14px;')
        values_layout.addWidget(self._linear_label)
        values_layout.addWidget(self._angular_label)
        layout.addLayout(values_layout)

    def _on_joystick_move(self, linear, angular):
        scale = self._speed_slider.value() / 100.0
        lin = linear * scale
        ang = angular * scale
        self._node.set_velocity(lin, ang)
        self._linear_label.setText(f'Linear: {lin:.2f}')
        self._angular_label.setText(f'Angular: {ang:.2f}')

    def _on_joystick_release(self):
        self._node.stop()
        self._linear_label.setText('Linear: 0.00')
        self._angular_label.setText('Angular: 0.00')
