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

import threading

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class TeleopNode(Node):
    """Nodo ROS2 de teleoperación que publica Twist a cmd_vel."""

    def __init__(self):
        super().__init__('axioma_teleop_gui')
        self._lock = threading.Lock()
        self._linear = 0.0
        self._angular = 0.0
        self._topic_name = 'cmd_vel'
        self._publisher = self.create_publisher(Twist, self._topic_name, 10)
        self._timer = self.create_timer(1.0 / 15.0, self._publish_twist)
        self._spin_thread = None

    def _publish_twist(self):
        msg = Twist()
        with self._lock:
            msg.linear.x = self._linear
            msg.angular.z = self._angular
        self._publisher.publish(msg)

    def set_velocity(self, linear, angular):
        """Establece la velocidad deseada (thread-safe)."""
        with self._lock:
            self._linear = float(linear)
            self._angular = float(angular)

    def stop(self):
        """Detiene el robot."""
        self.set_velocity(0.0, 0.0)

    def set_topic(self, name):
        """Cambia el tópico de publicación en runtime."""
        if name == self._topic_name:
            return
        self.stop()
        self.destroy_publisher(self._publisher)
        self._topic_name = name
        self._publisher = self.create_publisher(Twist, self._topic_name, 10)
        self.get_logger().info(f'Topic changed to: {self._topic_name}')

    def start_spinning(self):
        """Lanza un daemon thread que hace spin del nodo."""
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True
        )
        self._spin_thread.start()

    def _spin_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
