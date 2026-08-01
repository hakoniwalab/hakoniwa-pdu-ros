from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from add_two_ints_rpc_fixture import AddTwoIntsRpcServer
from hakoniwa_pdu_ros.service_binding import load_service_binding
from hakoniwa_pdu_ros.service_config_generator import generate_service_configs
from hakoniwa_pdu_ros.service_server_node import HakoniwaRosServiceServerNode


REPO_ROOT = Path(__file__).resolve().parents[2]
BINDING = REPO_ROOT / "config" / "service" / "add_two_ints.json"
ENDPOINT_CONFIG = REPO_ROOT / "config" / "service" / "rpc-endpoints.json"
OFFSETS = REPO_ROOT / "test" / "fixtures" / "offset"


def test_ros_service_server_calls_typed_hakoniwa_rpc(tmp_path: Path) -> None:
    generated = generate_service_configs(
        BINDING,
        output_dir=tmp_path,
        offset_dir=OFFSETS,
    )
    library_path = os.environ["HAKO_PDU_RPC_LIBRARY"]
    server_result: list[tuple[int, int, int]] = []
    server_errors: list[BaseException] = []

    with AddTwoIntsRpcServer(
        library_path,
        generated.server_config,
        ENDPOINT_CONFIG,
    ) as rpc_server:
        rpc_server.start()
        rclpy.init()
        bridge = HakoniwaRosServiceServerNode(
            load_service_binding(BINDING),
            generated.client_config,
            library_path,
        )
        caller = Node("add_two_ints_test_client")
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(bridge)
        executor.add_node(caller)
        spin_thread = threading.Thread(target=executor.spin, name="ros-executor")
        spin_thread.start()
        server_thread: threading.Thread | None = None
        try:
            client = caller.create_client(AddTwoInts, "/add_two_ints")
            assert client.wait_for_service(timeout_sec=3.0)

            def serve() -> None:
                try:
                    server_result.append(rpc_server.serve_once(timeout_sec=5.0))
                except BaseException as error:
                    server_errors.append(error)

            server_thread = threading.Thread(target=serve, name="rpc-server")
            server_thread.start()

            request = AddTwoInts.Request()
            request.a = 20
            request.b = 22
            future = client.call_async(request)
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)

            assert future.done()
            assert future.exception() is None
            assert future.result().sum == 42
            server_thread.join(timeout=5.0)
            assert not server_thread.is_alive()
            assert not server_errors
            assert server_result == [(20, 22, 42)]
        finally:
            if server_thread is not None and server_thread.is_alive():
                server_thread.join(timeout=5.0)
            executor.shutdown(timeout_sec=5.0)
            spin_thread.join(timeout=5.0)
            executor.remove_node(caller)
            executor.remove_node(bridge)
            caller.destroy_node()
            bridge.destroy_node()
            rclpy.shutdown()
