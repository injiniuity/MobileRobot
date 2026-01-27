#!/usr/bin/env python3
import os

os.environ.setdefault(
    "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GuiTrigger(Node):
    def __init__(self):
        super().__init__("gui_trigger")
        self._gui_pub = self.create_publisher(String, "/gui_cmd", 10)
        self.create_subscription(String, "/keyboard_raw", self._raw_cb, 10)
        self.create_subscription(String, "/arrived_point", self._arrived_cb, 10)

    def _raw_cb(self, msg):
        cmd = msg.data.strip()
        if not cmd:
            return
        if cmd in ("1", "3", "4", "SPACE", "P", "U"):
            self._gui_pub.publish(String(data=cmd))
            self.get_logger().info(f"GUI command sent: {cmd}")

    def _arrived_cb(self, msg):
        point = msg.data.strip()
        if not point:
            return
        self.get_logger().info(f"Arrived point received: {point}")


def main():
    rclpy.init()
    node = GuiTrigger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
