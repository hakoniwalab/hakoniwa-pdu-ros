from __future__ import annotations

import json
import os
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from add_two_ints_rpc_fixture import AddTwoIntsRpcServer
from hakoniwa_pdu_ros import service_server_node as service_server_module
from hakoniwa_pdu_ros.service_binding import load_service_binding
from hakoniwa_pdu_ros.service_config_generator import generate_service_configs
from hakoniwa_pdu_ros.service_server_node import (
    HakoniwaRosServiceServerNode,
    _spin_service_executor,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
BINDING = REPO_ROOT / "config" / "service" / "add_two_ints.json"
ENDPOINT_CONFIG = REPO_ROOT / "config" / "service" / "rpc-endpoints.json"
MUX_ENDPOINT_CONFIG = REPO_ROOT / "config" / "service" / "server-mux-endpoint.json"
OFFSETS = REPO_ROOT / "test" / "fixtures" / "offset"


def test_ros_service_server_calls_typed_hakoniwa_rpc(tmp_path: Path) -> None:
    with _bridge_runtime(tmp_path, BINDING) as (rpc_server, client, _bridge):
        server_result: list[tuple[int, int, int]] = []
        server_errors: list[BaseException] = []

        def serve() -> None:
            try:
                server_result.append(rpc_server.serve_once(timeout_sec=5.0))
            except BaseException as error:
                server_errors.append(error)

        server_thread = threading.Thread(target=serve, name="rpc-server")
        server_thread.start()

        future = _call(client, 20, 22)
        assert _wait_done(future)
        assert future.exception() is None
        assert future.result().sum == 42
        server_thread.join(timeout=5.0)
        assert not server_thread.is_alive()
        assert not server_errors
        assert server_result == [(20, 22, 42)]


def test_ros_service_server_supports_consecutive_calls(tmp_path: Path) -> None:
    with _bridge_runtime(tmp_path, BINDING) as (rpc_server, client, _bridge):
        results = []
        for left, right in ((1, 2), (20, 22)):
            server_thread = threading.Thread(target=rpc_server.serve_once)
            server_thread.start()
            future = _call(client, left, right)
            assert _wait_done(future)
            results.append(future.result().sum)
            server_thread.join(timeout=5.0)
            assert not server_thread.is_alive()

        assert results == [3, 42]


def test_ros_service_server_supports_max_clients_and_rejects_busy(
    tmp_path: Path,
) -> None:
    with _bridge_runtime(tmp_path, BINDING) as (rpc_server, client, bridge):
        futures = [_call(client, index, 10) for index in range(4)]
        requests = [rpc_server.receive_request(timeout_sec=5.0) for _ in range(4)]
        pool = bridge._pools["Service/Add"]
        assert pool.active_count == 4

        busy_future = _call(client, 99, 1)
        time.sleep(0.1)
        assert not busy_future.done()
        assert pool.active_count == 4

        for request in requests:
            rpc_server.send_sum(request)
        assert all(_wait_done(future) for future in futures)
        assert sorted(future.result().sum for future in futures) == [10, 11, 12, 13]
        assert _wait_until(lambda: pool.active_count == 0)


def test_timeout_discards_late_success_and_pool_is_reusable(tmp_path: Path) -> None:
    binding = _binding_with_timeout(tmp_path, timeout_msec=100)
    with _bridge_runtime(tmp_path / "runtime", binding) as (
        rpc_server,
        client,
        bridge,
    ):
        timed_out = _call(client, 20, 22)
        late_request = rpc_server.receive_request(timeout_sec=5.0)
        time.sleep(0.2)
        rpc_server.send_sum(late_request)

        pool = bridge._pools["Service/Add"]
        assert _wait_until(lambda: pool.active_count == 0)
        assert not timed_out.done()

        next_future = _call(client, 1, 2)
        next_request = rpc_server.receive_request(timeout_sec=5.0)
        rpc_server.send_sum(next_request)
        assert _wait_done(next_future)
        assert next_future.result().sum == 3


def test_shutdown_cancels_active_call_and_closes_runtime(tmp_path: Path) -> None:
    with _bridge_runtime(tmp_path, BINDING) as (rpc_server, client, bridge):
        ros_future = _call(client, 20, 22)
        _request = rpc_server.receive_request(timeout_sec=5.0)
        pool = bridge._pools["Service/Add"]
        assert pool.active_count == 1

        cancel_errors: list[BaseException] = []

        def acknowledge_cancel() -> None:
            try:
                # The mux API returns the token to use for this cancel event;
                # it need not be numerically identical to the request token.
                token = rpc_server.receive_cancel()
                rpc_server.send_cancel(token)
            except BaseException as error:
                cancel_errors.append(error)

        cancel_thread = threading.Thread(
            target=acknowledge_cancel,
            name="rpc-cancel-server",
        )
        cancel_thread.start()

        bridge._close_runtime()

        cancel_thread.join(timeout=5.0)
        assert not cancel_thread.is_alive()
        assert not cancel_errors
        assert _wait_until(lambda: pool.active_count == 0)
        assert pool.acquire() is None
        assert bridge._services == []
        assert bridge._pools == {}
        assert not ros_future.done()
        # Let the executor retrieve the intentional callback rejection before
        # the context shuts the executor down; otherwise rclpy reports a
        # misleading "exception was never retrieved" during object teardown.
        bridge.executor.create_task(lambda: None)
        time.sleep(0.05)


def test_request_conversion_error_synthesizes_no_response_and_node_continues(
    tmp_path: Path,
    monkeypatch,
) -> None:
    with _bridge_runtime(tmp_path, BINDING) as (rpc_server, client, bridge):
        original_copy = service_server_module.copy_matching_fields
        logged_errors: list[str] = []
        monkeypatch.setattr(
            bridge,
            "get_logger",
            lambda: _RecordingLogger(logged_errors),
        )

        def fail_request(source: object, target: object) -> None:
            if isinstance(source, AddTwoInts.Request):
                raise ValueError("injected request conversion failure")
            original_copy(source, target)

        monkeypatch.setattr(
            service_server_module,
            "copy_matching_fields",
            fail_request,
        )
        rejected = _call(client, 20, 22)
        pool = bridge._pools["Service/Add"]
        assert _wait_until(lambda: bool(logged_errors))
        assert _wait_until(lambda: pool.active_count == 0)
        assert not rejected.done()
        assert any("direction=request" in message for message in logged_errors)
        assert any("ros_service=/add_two_ints" in message for message in logged_errors)
        assert any("rpc_service=Service/Add" in message for message in logged_errors)
        # Wake the executor so it retrieves the intentional callback
        # rejection and re-enters spin before the recovery request.
        bridge.executor.create_task(lambda: None)
        time.sleep(0.05)

        monkeypatch.setattr(
            service_server_module,
            "copy_matching_fields",
            original_copy,
        )
        server_thread = threading.Thread(target=rpc_server.serve_once)
        server_thread.start()
        recovered = _call(client, 1, 2)
        assert _wait_done(recovered)
        assert recovered.result().sum == 3
        server_thread.join(timeout=5.0)
        assert not server_thread.is_alive()


def test_response_conversion_error_synthesizes_no_response_and_node_continues(
    tmp_path: Path,
    monkeypatch,
) -> None:
    with _bridge_runtime(tmp_path, BINDING) as (rpc_server, client, bridge):
        original_copy = service_server_module.copy_matching_fields
        logged_errors: list[str] = []
        monkeypatch.setattr(
            bridge,
            "get_logger",
            lambda: _RecordingLogger(logged_errors),
        )

        def fail_response(source: object, target: object) -> None:
            if isinstance(target, AddTwoInts.Response):
                raise ValueError("injected response conversion failure")
            original_copy(source, target)

        monkeypatch.setattr(
            service_server_module,
            "copy_matching_fields",
            fail_response,
        )
        server_thread = threading.Thread(target=rpc_server.serve_once)
        server_thread.start()
        rejected = _call(client, 20, 22)
        pool = bridge._pools["Service/Add"]
        assert _wait_until(lambda: bool(logged_errors))
        assert _wait_until(lambda: pool.active_count == 0)
        assert not rejected.done()
        server_thread.join(timeout=5.0)
        assert not server_thread.is_alive()
        assert any("direction=response" in message for message in logged_errors)
        assert any("ros_service=/add_two_ints" in message for message in logged_errors)
        assert any("rpc_service=Service/Add" in message for message in logged_errors)
        bridge.executor.create_task(lambda: None)
        time.sleep(0.05)

        monkeypatch.setattr(
            service_server_module,
            "copy_matching_fields",
            original_copy,
        )
        recovery_thread = threading.Thread(target=rpc_server.serve_once)
        recovery_thread.start()
        recovered = _call(client, 1, 2)
        assert _wait_done(recovered)
        assert recovered.result().sum == 3
        recovery_thread.join(timeout=5.0)
        assert not recovery_thread.is_alive()


@contextmanager
def _bridge_runtime(
    output_dir: Path,
    binding_path: Path,
) -> Iterator[tuple[AddTwoIntsRpcServer, object, HakoniwaRosServiceServerNode]]:
    generated = generate_service_configs(
        binding_path,
        output_dir=output_dir,
        offset_dir=OFFSETS,
    )
    library_path = os.environ["HAKO_PDU_RPC_LIBRARY"]
    binding = load_service_binding(binding_path)

    with AddTwoIntsRpcServer(
        library_path,
        generated.server_config,
        MUX_ENDPOINT_CONFIG,
    ) as rpc_server:
        rpc_server.start()
        rclpy.init()
        bridge = HakoniwaRosServiceServerNode(
            binding,
            generated.client_config,
            library_path,
            generated.services,
        )
        caller = Node("add_two_ints_test_client")
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(bridge)
        executor.add_node(caller)
        spin_thread = threading.Thread(
            target=_spin_service_executor,
            args=(executor,),
            name="ros-executor",
        )
        spin_thread.start()
        try:
            client = caller.create_client(AddTwoInts, "/add_two_ints")
            assert client.wait_for_service(timeout_sec=3.0)
            # ROS discovery and RPC transport establishment are independent.
            # Give the already-started RPC clients a short, deterministic
            # connection window before issuing the first request.
            rpc_server.wait_connected(4)
            yield rpc_server, client, bridge
        finally:
            executor.shutdown(timeout_sec=5.0)
            spin_thread.join(timeout=5.0)
            executor.remove_node(caller)
            executor.remove_node(bridge)
            caller.destroy_node()
            bridge.destroy_node()
            rclpy.shutdown()


def _call(client: object, left: int, right: int):
    request = AddTwoInts.Request()
    request.a = left
    request.b = right
    return client.call_async(request)


class _RecordingLogger:
    def __init__(self, errors: list[str]) -> None:
        self._errors = errors

    def error(self, message: str) -> None:
        self._errors.append(message)


def _wait_done(future: object, timeout_sec: float = 5.0) -> bool:
    return _wait_until(future.done, timeout_sec=timeout_sec)


def _wait_until(predicate, timeout_sec: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def _binding_with_timeout(tmp_path: Path, *, timeout_msec: int) -> Path:
    tmp_path.mkdir(parents=True, exist_ok=True)
    binding_path = tmp_path / "add_two_ints.json"
    data = json.loads(BINDING.read_text(encoding="utf-8"))
    data["bindings"][0]["timeout_msec"] = timeout_msec
    data["rpc"]["endpoint_config"] = str(ENDPOINT_CONFIG)
    binding_path.write_text(json.dumps(data), encoding="utf-8")
    return binding_path
