from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from add_two_ints_rpc_fixture import create_add_two_ints_client
from hakoniwa_pdu_ros.service_binding import load_service_binding
from hakoniwa_pdu_ros.service_client_node import HakoniwaRosServiceClientNode
from hakoniwa_pdu_ros.service_config_generator import generate_service_configs


REPO_ROOT = Path(__file__).resolve().parents[2]
BINDING = REPO_ROOT / "config" / "service" / "add_two_ints.json"
OFFSETS = REPO_ROOT / "test" / "fixtures" / "offset"


def test_hakoniwa_rpc_client_calls_ros_service_server(tmp_path: Path) -> None:
    generated = generate_service_configs(
        BINDING,
        output_dir=tmp_path,
        offset_dir=OFFSETS,
    )
    binding = load_service_binding(BINDING)
    library = os.environ["HAKO_PDU_RPC_LIBRARY"]

    rclpy.init()
    ros_server = Node("add_two_ints_ros_server")

    def add(request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        return response

    service = ros_server.create_service(AddTwoInts, "/add_two_ints", add)
    bridge = HakoniwaRosServiceClientNode(binding, generated, library)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ros_server)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, name="ros-service-client-e2e")
    spin_thread.start()

    rpc_client, typed_client = create_add_two_ints_client(
        library,
        generated.client_config,
        generated.endpoint_config,
    )
    try:
        rpc_client.start()
        _wait_rpc_connections(bridge, expected=1)

        first = typed_client.create_request()
        first.a = 20
        first.b = 22
        assert typed_client.call(first, timeout_usec=2_000_000).sum == 42

        second = typed_client.create_request()
        second.a = 19
        second.b = 23
        assert typed_client.call(second, timeout_usec=2_000_000).sum == 42
    finally:
        rpc_client.close()
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        executor.remove_node(bridge)
        executor.remove_node(ros_server)
        bridge.destroy_node()
        ros_server.destroy_service(service)
        ros_server.destroy_node()
        rclpy.shutdown()


def _wait_rpc_connections(
    bridge: HakoniwaRosServiceClientNode,
    *,
    expected: int,
    timeout_sec: float = 5.0,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if sum(server.connected_count() for server in bridge._raw_servers) == expected:
            return
        time.sleep(0.01)
    raise AssertionError("Hakoniwa RPC Client did not connect to the Service Bridge")
