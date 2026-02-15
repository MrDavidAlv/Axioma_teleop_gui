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

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class SliderMode(QWidget):
    """Modo de teleoperación con sliders para linear y angular."""

    def __init__(self, node):
        super().__init__()
        self._node = node

        layout = QVBoxLayout(self)

        # --- Linear velocity ---
        layout.addWidget(QLabel('Linear velocity (m/s):'))
        lin_layout = QHBoxLayout()

        self._lin_slider = QSlider(Qt.Horizontal)
        self._lin_slider.setRange(-100, 100)
        self._lin_slider.setValue(0)
        self._lin_slider.setTickInterval(10)
        self._lin_slider.setTickPosition(QSlider.TicksBelow)
        lin_layout.addWidget(self._lin_slider)

        self._lin_spin = QDoubleSpinBox()
        self._lin_spin.setRange(-1.0, 1.0)
        self._lin_spin.setSingleStep(0.05)
        self._lin_spin.setDecimals(2)
        self._lin_spin.setValue(0.0)
        lin_layout.addWidget(self._lin_spin)

        lin_reset_btn = QPushButton('Reset')
        lin_reset_btn.clicked.connect(lambda: self._lin_slider.setValue(0))
        lin_layout.addWidget(lin_reset_btn)

        layout.addLayout(lin_layout)

        # --- Angular velocity ---
        layout.addWidget(QLabel('Angular velocity (rad/s):'))
        ang_layout = QHBoxLayout()

        self._ang_slider = QSlider(Qt.Horizontal)
        self._ang_slider.setRange(-300, 300)
        self._ang_slider.setValue(0)
        self._ang_slider.setTickInterval(30)
        self._ang_slider.setTickPosition(QSlider.TicksBelow)
        ang_layout.addWidget(self._ang_slider)

        self._ang_spin = QDoubleSpinBox()
        self._ang_spin.setRange(-3.0, 3.0)
        self._ang_spin.setSingleStep(0.1)
        self._ang_spin.setDecimals(2)
        self._ang_spin.setValue(0.0)
        ang_layout.addWidget(self._ang_spin)

        ang_reset_btn = QPushButton('Reset')
        ang_reset_btn.clicked.connect(lambda: self._ang_slider.setValue(0))
        ang_layout.addWidget(ang_reset_btn)

        layout.addLayout(ang_layout)

        # --- STOP button ---
        stop_btn = QPushButton('STOP')
        stop_btn.setStyleSheet(
            'QPushButton { background-color: #cc0000; color: white; '
            'font-size: 18px; font-weight: bold; padding: 12px; }'
            'QPushButton:pressed { background-color: #990000; }'
        )
        stop_btn.clicked.connect(self.reset)
        layout.addWidget(stop_btn)

        layout.addStretch()

        # --- Connections ---
        self._lin_slider.valueChanged.connect(self._on_lin_slider)
        self._lin_spin.valueChanged.connect(self._on_lin_spin)
        self._ang_slider.valueChanged.connect(self._on_ang_slider)
        self._ang_spin.valueChanged.connect(self._on_ang_spin)

    def _on_lin_slider(self, value):
        real = value / 100.0
        self._lin_spin.blockSignals(True)
        self._lin_spin.setValue(real)
        self._lin_spin.blockSignals(False)
        self._publish()

    def _on_lin_spin(self, value):
        self._lin_slider.blockSignals(True)
        self._lin_slider.setValue(int(value * 100))
        self._lin_slider.blockSignals(False)
        self._publish()

    def _on_ang_slider(self, value):
        real = value / 100.0
        self._ang_spin.blockSignals(True)
        self._ang_spin.setValue(real)
        self._ang_spin.blockSignals(False)
        self._publish()

    def _on_ang_spin(self, value):
        self._ang_slider.blockSignals(True)
        self._ang_slider.setValue(int(value * 100))
        self._ang_slider.blockSignals(False)
        self._publish()

    def _publish(self):
        self._node.set_velocity(
            self._lin_spin.value(),
            -self._ang_spin.value()
        )

    def reset(self):
        """Resetea sliders y spinboxes a cero."""
        self._lin_slider.setValue(0)
        self._ang_slider.setValue(0)
        self._node.stop()
