from __future__ import annotations

import signal
import time
from pathlib import Path

from hakoniwa_pdu_ros.env_setup import configure_import_paths
from hakoniwa_pdu_ros.type_mapper import pdu_bytes_to_ros_msg, ros_msg_to_pdu_bytes

configure_import_paths()

from hakoniwa_pdu_endpoint.c_endpoint import Endpoint, PduKey  # noqa: E402
from hakoniwa_pdu_endpoint.c_endpoint_async import EndpointAsync, PduEvent  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402


def main() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    endpoint_config = repo_root / "config" / "sample" / "endpoint_zenoh_connect.json"

    endpoint = Endpoint("hakoniwa_pdu_ros_example_peer", "inout")
    async_endpoint = EndpointAsync(endpoint)
    async_endpoint.open(str(endpoint_config))
    async_endpoint.start()
    async_endpoint.post_start()
    async_endpoint.start_dispatch()

    running = True

    def _stop(*_args) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    def _on_cmd(event: PduEvent) -> None:
        msg = pdu_bytes_to_ros_msg(event.payload, "geometry_msgs/Twist")
        print(
            f"[cmd] linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}) "
            f"angular=({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})"
        )

    async_endpoint.on_recv_by_name(PduKey(robot="Drone", pdu="cmd"), _on_cmd)

    try:
        seq = 0.0
        while running:
            msg = Twist()
            msg.linear.x = seq
            msg.linear.y = seq + 1.0
            msg.linear.z = seq + 2.0
            msg.angular.z = seq + 3.0
            async_endpoint.send_by_name(
                PduKey(robot="Drone", pdu="pos"),
                ros_msg_to_pdu_bytes(msg, "geometry_msgs/Twist"),
            )
            print(f"[pos] sent linear.x={msg.linear.x:.2f} angular.z={msg.angular.z:.2f}")
            seq += 1.0
            time.sleep(1.0)
    finally:
        async_endpoint.stop_dispatch()
        async_endpoint.stop()
        async_endpoint.close()


if __name__ == "__main__":
    main()
