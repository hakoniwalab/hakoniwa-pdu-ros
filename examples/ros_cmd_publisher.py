from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdPublisher(Node):
    def __init__(self) -> None:
        super().__init__("hakoniwa_cmd_publisher")
        self._publisher = self.create_publisher(Twist, "/hakoniwa/drone/cmd", 10)
        self._seq = 0.0
        self.create_timer(1.0, self._on_timer)

    def _on_timer(self) -> None:
        msg = Twist()
        msg.linear.x = 100.0 + self._seq
        msg.angular.z = 200.0 + self._seq
        self._publisher.publish(msg)
        self.get_logger().info(
            f"cmd linear.x={msg.linear.x:.2f} angular.z={msg.angular.z:.2f}"
        )
        self._seq += 1.0


def main() -> None:
    rclpy.init()
    node = CmdPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
