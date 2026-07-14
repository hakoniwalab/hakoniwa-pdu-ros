from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PosSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("hakoniwa_pos_subscriber")
        self.create_subscription(Twist, "/pdu/hakoniwa/drone/pos", self._on_msg, 10)

    def _on_msg(self, msg: Twist) -> None:
        self.get_logger().info(
            f"pos linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}) "
            f"angular=({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})"
        )


def main() -> None:
    rclpy.init()
    node = PosSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
