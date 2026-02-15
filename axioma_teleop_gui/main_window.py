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
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from axioma_teleop_gui.widgets.joystick_mode import JoystickMode
from axioma_teleop_gui.widgets.keyboard_mode import KeyboardMode
from axioma_teleop_gui.widgets.slider_mode import SliderMode


class MainWindow(QMainWindow):
    """Ventana principal de la GUI de teleoperación."""

    def __init__(self, node):
        super().__init__()
        self._node = node
        self.setWindowTitle('Axioma Teleop GUI')
        self.setMinimumSize(420, 480)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # --- Topic selector ---
        topic_layout = QHBoxLayout()
        topic_layout.addWidget(QLabel('Topic:'))
        self._topic_edit = QLineEdit('cmd_vel')
        topic_layout.addWidget(self._topic_edit)
        set_btn = QPushButton('Set')
        set_btn.clicked.connect(self._on_set_topic)
        topic_layout.addWidget(set_btn)
        layout.addLayout(topic_layout)

        # --- Mode selector ---
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel('Mode:'))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems([
            'Keyboard Buttons',
            'Virtual Joystick',
            'Sliders',
        ])
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_layout.addWidget(self._mode_combo)
        layout.addLayout(mode_layout)

        # --- Stacked widget ---
        self._stack = QStackedWidget()
        self._keyboard_mode = KeyboardMode(node)
        self._joystick_mode = JoystickMode(node)
        self._slider_mode = SliderMode(node)
        self._stack.addWidget(self._keyboard_mode)
        self._stack.addWidget(self._joystick_mode)
        self._stack.addWidget(self._slider_mode)
        layout.addWidget(self._stack)

    def _on_set_topic(self):
        topic = self._topic_edit.text().strip()
        if topic:
            self._node.set_topic(topic)

    def _on_mode_changed(self, index):
        self._node.stop()
        self._slider_mode.reset()
        self._stack.setCurrentIndex(index)

    def closeEvent(self, event):
        self._node.stop()
        event.accept()
