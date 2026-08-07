from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from hakoniwa_pdu_rpc import (
    ActionMuxServer,
    ActionServerEvent,
    ActionTerminalStatus,
)
from hakoniwa_pdu_ros.action_binding import (
    ActionBinding,
    ActionBindingConfig,
    ActionRuntimeConfig,
)
from hakoniwa_pdu_ros.action_server_node import HakoniwaRosActionServerNode


RPC_ROOT = Path("/opt/src/hakoniwa-pdu-rpc")
ACTION_CONFIG = RPC_ROOT / "test/configs/action_resolved.json"
CLIENT_ENDPOINT_CONFIG = RPC_ROOT / "test/configs/action_mux_e2e/endpoints.json"
SERVER_ENDPOINT_CONFIG = RPC_ROOT / "test/configs/action_mux_e2e/server_endpoint.json"
ACTION_NAME = "fibonacci"
ROS_ACTION_NAME = "/fibonacci"


def _binding_config() -> ActionBindingConfig:
    return ActionBindingConfig(
        runtime=ActionRuntimeConfig(
            node_id="fibonacci-client",
            client_name="hakoniwa-pdu-ros-action-e2e",
            action_config=ACTION_CONFIG,
            endpoint_config=CLIENT_ENDPOINT_CONFIG,
        ),
        actions=(
            ActionBinding(
                ros_name=ROS_ACTION_NAME,
                ros_type="example_interfaces/action/Fibonacci",
                hakoniwa_action=ACTION_NAME,
                pdu_action_type="sample_action_msgs/Fibonacci",
                goal_response_timeout_msec=3000,
            ),
        ),
    )


def _wait_future(future, timeout_sec: float = 5.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.001)
    raise TimeoutError("ROS Action future timed out")


def _wait_server_event(
    server: ActionMuxServer,
    expected: ActionServerEvent,
    timeout_sec: float = 5.0,
):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        event = server.poll()
        if event.event == expected:
            return event
        if event.event != ActionServerEvent.NONE:
            raise RuntimeError(f"unexpected Hakoniwa Action event: {event.event}")
        time.sleep(0.001)
    raise TimeoutError(f"Hakoniwa Action event timed out: {expected.name}")


def _run_success_server(server: ActionMuxServer, observed_goal_ids: list[bytes]) -> None:
    incoming = _wait_server_event(server, ActionServerEvent.GOAL_REQUEST)
    assert incoming.goal is not None
    observed_goal_ids.append(incoming.goal.goal_id)
    server.accept_goal(incoming.action_name, incoming.goal)

    feedback = server.create_feedback_buffer(ACTION_NAME)
    server.send_feedback(ACTION_NAME, incoming.goal, feedback)

    result = server.create_result_buffer(ACTION_NAME)
    server.complete(
        ACTION_NAME,
        incoming.goal,
        ActionTerminalStatus.SUCCEEDED,
        result,
    )


def _run_cancel_server(server: ActionMuxServer, observed_goal_ids: list[bytes]) -> None:
    incoming = _wait_server_event(server, ActionServerEvent.GOAL_REQUEST)
    assert incoming.goal is not None
    observed_goal_ids.append(incoming.goal.goal_id)
    server.accept_goal(incoming.action_name, incoming.goal)

    cancel = _wait_server_event(server, ActionServerEvent.CANCEL_REQUEST)
    assert cancel.goal == incoming.goal
    server.accept_cancel(cancel.action_name, cancel.goal)

    result = server.create_result_buffer(ACTION_NAME)
    server.complete(
        ACTION_NAME,
        incoming.goal,
        ActionTerminalStatus.CANCELED,
        result,
    )


def _start_ros_bridge(library: str):
    rclpy.init()
    bridge = HakoniwaRosActionServerNode(_binding_config(), library)
    client_node = rclpy.create_node("hakoniwa_pdu_ros_action_e2e_client")
    client = ActionClient(client_node, Fibonacci, ROS_ACTION_NAME)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if client.server_is_ready():
            return bridge, client_node, client, executor, spin_thread
        time.sleep(0.01)
    raise TimeoutError("ROS Action bridge did not become ready")


def _stop_ros_bridge(bridge, client_node, client, executor, spin_thread) -> None:
    client.destroy()
    executor.remove_node(client_node)
    executor.remove_node(bridge)
    client_node.destroy_node()
    bridge.destroy_node()
    executor.shutdown(timeout_sec=3.0)
    spin_thread.join(timeout=3.0)
    rclpy.shutdown()


def test_native_action_goal_feedback_result_and_goal_uuid_identity():
    library = os.environ["HAKO_PDU_RPC_LIBRARY"]
    server = ActionMuxServer(
        library,
        "fibonacci-server",
        ACTION_CONFIG,
        SERVER_ENDPOINT_CONFIG,
    )
    server.start()

    bridge = client_node = client = executor = spin_thread = None
    observed_goal_ids: list[bytes] = []
    server_thread = threading.Thread(
        target=_run_success_server,
        args=(server, observed_goal_ids),
        daemon=True,
    )
    try:
        bridge, client_node, client, executor, spin_thread = _start_ros_bridge(library)
        server_thread.start()

        feedback_seen = threading.Event()
        goal = Fibonacci.Goal()
        goal.order = 10
        goal_handle = _wait_future(
            client.send_goal_async(
                goal,
                feedback_callback=lambda _feedback: feedback_seen.set(),
            )
        )
        assert goal_handle.accepted
        result = _wait_future(goal_handle.get_result_async())
        assert result.status == 4  # action_msgs/msg/GoalStatus.STATUS_SUCCEEDED
        assert feedback_seen.wait(2.0)

        server_thread.join(timeout=2.0)
        assert not server_thread.is_alive()
        assert observed_goal_ids == [bytes(goal_handle.goal_id.uuid)]
    finally:
        if bridge is not None:
            _stop_ros_bridge(bridge, client_node, client, executor, spin_thread)
        server.close()


def test_native_action_cancel_maps_to_canceled_result():
    library = os.environ["HAKO_PDU_RPC_LIBRARY"]
    server = ActionMuxServer(
        library,
        "fibonacci-server",
        ACTION_CONFIG,
        SERVER_ENDPOINT_CONFIG,
    )
    server.start()

    bridge = client_node = client = executor = spin_thread = None
    observed_goal_ids: list[bytes] = []
    server_thread = threading.Thread(
        target=_run_cancel_server,
        args=(server, observed_goal_ids),
        daemon=True,
    )
    try:
        bridge, client_node, client, executor, spin_thread = _start_ros_bridge(library)
        server_thread.start()

        goal = Fibonacci.Goal()
        goal.order = 47
        goal_handle = _wait_future(client.send_goal_async(goal))
        assert goal_handle.accepted

        cancel_response = _wait_future(goal_handle.cancel_goal_async())
        assert len(cancel_response.goals_canceling) == 1

        result = _wait_future(goal_handle.get_result_async())
        assert result.status == 5  # action_msgs/msg/GoalStatus.STATUS_CANCELED

        server_thread.join(timeout=2.0)
        assert not server_thread.is_alive()
        assert observed_goal_ids == [bytes(goal_handle.goal_id.uuid)]
    finally:
        if bridge is not None:
            _stop_ros_bridge(bridge, client_node, client, executor, spin_thread)
        server.close()
