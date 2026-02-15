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

from PyQt5.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtCore import Qt


class KeyboardMode(QWidget):
    """Modo de teleoperación con botones tipo teclado (grid 3x3)."""

    # (linear, angular) para cada botón
    _BUTTONS = [
        ('↖', 1.0, 1.0),   ('↑', 1.0, 0.0),  ('↗', 1.0, -1.0),
        ('←', 0.0, 1.0),   ('*', 0.0, 0.0),   ('→', 0.0, -1.0),
        ('↙', -1.0, -1.0), ('↓', -1.0, 0.0),  ('↘', -1.0, 1.0),
    ]

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

        # --- Button grid ---
        grid = QGridLayout()
        grid.setSpacing(4)
        for idx, (label, lin, ang) in enumerate(self._BUTTONS):
            btn = QPushButton(label)
            btn.setMinimumSize(80, 60)
            btn.setStyleSheet('font-size: 16px; font-weight: bold;')
            btn.pressed.connect(self._make_press_handler(lin, ang))
            btn.released.connect(self._node.stop)
            row, col = divmod(idx, 3)
            grid.addWidget(btn, row, col)
        layout.addLayout(grid)
        layout.addStretch()

    def _make_press_handler(self, lin_dir, ang_dir):
        def handler():
            scale = self._speed_slider.value() / 100.0
            self._node.set_velocity(lin_dir * scale, ang_dir * scale)
        return handler
