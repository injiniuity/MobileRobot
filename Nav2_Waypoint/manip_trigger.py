#!/usr/bin/env python3
import os

os.environ.setdefault(
    "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ManipTrigger(Node):
    def __init__(self):
        super().__init__("manip_trigger")
        self._manip_pub = self.create_publisher(String, "/manip_cmd", 10)
        self.create_subscription(String, "/keyboard_raw", self._raw_cb, 10)
        self.create_subscription(String, "/arrived_point", self._arrived_cb, 10)

    def _raw_cb(self, msg):
        cmd = msg.data.strip()
        if not cmd:
            return
        if cmd.lower() == "q":
            self._manip_pub.publish(String(data="Q"))
            self.get_logger().info("Manipulator command sent: Q")
        elif cmd.lower() == "w":
            self._manip_pub.publish(String(data="W"))
            self.get_logger().info("Manipulator command sent: W")

    def _arrived_cb(self, msg):
        point = msg.data.strip()
        if not point:
            return
        self.get_logger().info(f"Arrived point received: {point}")


def main():
    rclpy.init()
    node = ManipTrigger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
