from __future__ import annotations

import json
import os
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Callable, Iterator

import rclpy
from action_msgs.msg import GoalStatus
from action_tutorials_interfaces.action import Fibonacci
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hakoniwa_pdu_rpc import ActionTerminalStatus

from fibonacci_action_fixture import FibonacciActionServer
from hakoniwa_pdu_ros.action_binding import load_action_binding
from hakoniwa_pdu_ros.action_config_generator import generate_action_configs
from hakoniwa_pdu_ros.action_server_node import HakoniwaRosActionServerNode


REPO_ROOT = Path(__file__).resolve().parents[2]
BINDING = REPO_ROOT / "config" / "action" / "fibonacci.json"


def test_ros_action_server_bridges_goal_feedback_and_result(
    tmp_path: Path,
) -> None:
    with _bridge_runtime(tmp_path) as (server, client):
        served: list[list[int]] = []
        errors: list[BaseException] = []

        def serve() -> None:
            try:
                served.append(server.serve_once())
            except BaseException as error:
                errors.append(error)

        server_thread = threading.Thread(target=serve, name="action-server")
        server_thread.start()

        feedback: list[list[int]] = []
        goal = Fibonacci.Goal()
        goal.order = 8
        send_future = client.send_goal_async(
            goal,
            feedback_callback=lambda message: feedback.append(
                list(message.feedback.partial_sequence)
            ),
        )
        assert _wait_done(send_future)
        goal_handle = send_future.result()
        assert goal_handle.accepted

        result_future = goal_handle.get_result_async()
        assert _wait_done(result_future)
        wrapped_result = result_future.result()
        expected = [0, 1, 1, 2, 3, 5, 8, 13]
        assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        assert list(wrapped_result.result.sequence) == expected

        server_thread.join(timeout=5.0)
        assert not server_thread.is_alive()
        assert not errors
        assert served == [expected]
        assert server.observed_order == 8
        assert feedback == server.feedback_sequences
        assert feedback[-1] == expected


def test_ros_action_server_propagates_goal_rejection(tmp_path: Path) -> None:
    with _bridge_runtime(tmp_path) as (server, client):
        def reject() -> None:
            goal, order = server.wait_goal()
            assert order == 0
            server.reject_goal(goal)

        server_thread, errors = _start_server_task(reject)
        goal = Fibonacci.Goal()
        goal.order = 0
        send_future = client.send_goal_async(goal)
        assert _wait_done(send_future)
        assert not send_future.result().accepted
        _assert_server_task(server_thread, errors)


def test_ros_action_server_supports_two_consecutive_goals(
    tmp_path: Path,
) -> None:
    with _bridge_runtime(tmp_path) as (server, client):
        served: list[list[int]] = []

        def serve_twice() -> None:
            served.append(server.serve_once())
            served.append(server.serve_once())

        server_thread, errors = _start_server_task(serve_twice)
        results = [
            _send_goal_and_wait_result(client, order)
            for order in (6, 9)
        ]
        _assert_server_task(server_thread, errors)

        assert results == [
            [0, 1, 1, 2, 3, 5],
            [0, 1, 1, 2, 3, 5, 8, 13, 21],
        ]
        assert served == results


def test_ros_action_server_propagates_accepted_cancel(tmp_path: Path) -> None:
    with _bridge_runtime(tmp_path) as (server, client):
        def cancel() -> None:
            goal, order = server.wait_goal()
            assert order == 20
            server.accept_goal(goal)
            cancel_goal = server.wait_cancel()
            assert cancel_goal == goal
            server.accept_cancel(cancel_goal)
            server.complete(
                goal,
                [0, 1, 1, 2, 3],
                ActionTerminalStatus.CANCELED,
            )

        server_thread, errors = _start_server_task(cancel)
        goal = Fibonacci.Goal()
        goal.order = 20
        send_future = client.send_goal_async(goal)
        assert _wait_done(send_future)
        goal_handle = send_future.result()
        assert goal_handle.accepted

        result_future = goal_handle.get_result_async()
        cancel_future = goal_handle.cancel_goal_async()
        assert _wait_done(cancel_future)
        assert len(cancel_future.result().goals_canceling) == 1
        assert _wait_done(result_future)
        wrapped_result = result_future.result()
        assert wrapped_result.status == GoalStatus.STATUS_CANCELED
        assert list(wrapped_result.result.sequence) == [0, 1, 1, 2, 3]
        _assert_server_task(server_thread, errors)


def test_ros_action_server_propagates_rejected_cancel(tmp_path: Path) -> None:
    with _bridge_runtime(tmp_path) as (server, client):
        expected = [0, 1, 1, 2, 3, 5, 8]

        def reject_cancel() -> None:
            goal, order = server.wait_goal()
            assert order == 7
            server.accept_goal(goal)
            cancel_goal = server.wait_cancel()
            assert cancel_goal == goal
            server.reject_cancel(cancel_goal)
            server.complete(goal, expected)

        server_thread, errors = _start_server_task(reject_cancel)
        goal = Fibonacci.Goal()
        goal.order = 7
        send_future = client.send_goal_async(goal)
        assert _wait_done(send_future)
        goal_handle = send_future.result()
        assert goal_handle.accepted

        result_future = goal_handle.get_result_async()
        cancel_future = goal_handle.cancel_goal_async()
        assert _wait_done(cancel_future)
        assert cancel_future.result().goals_canceling == []
        assert _wait_done(result_future)
        wrapped_result = result_future.result()
        assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        assert list(wrapped_result.result.sequence) == expected
        _assert_server_task(server_thread, errors)


def test_ros_action_result_can_win_while_cancel_is_pending(
    tmp_path: Path,
) -> None:
    with _bridge_runtime(tmp_path) as (server, client):
        expected = [0, 1, 1, 2, 3, 5]

        def finish_before_cancel_decision() -> None:
            goal, order = server.wait_goal()
            assert order == 6
            server.accept_goal(goal)
            cancel_goal = server.wait_cancel()
            assert cancel_goal == goal
            server.complete(goal, expected)

        server_thread, errors = _start_server_task(
            finish_before_cancel_decision
        )
        goal = Fibonacci.Goal()
        goal.order = 6
        send_future = client.send_goal_async(goal)
        assert _wait_done(send_future)
        goal_handle = send_future.result()
        assert goal_handle.accepted

        result_future = goal_handle.get_result_async()
        cancel_future = goal_handle.cancel_goal_async()
        assert _wait_done(cancel_future)
        assert cancel_future.result().goals_canceling == []
        assert _wait_done(result_future)
        wrapped_result = result_future.result()
        assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        assert list(wrapped_result.result.sequence) == expected
        _assert_server_task(server_thread, errors)


def test_ros_action_server_enforces_slot_limit_with_active_goals(
    tmp_path: Path,
) -> None:
    with _bridge_runtime(tmp_path) as (server, client):
        accepted_goals: list[object] = []
        orders = [5, 6, 7, 8]
        release_results = threading.Event()

        def accept_four_then_complete() -> None:
            for expected_order in orders:
                goal, order = server.wait_goal()
                assert order == expected_order
                server.accept_goal(goal)
                accepted_goals.append(goal)
            if not release_results.wait(5.0):
                raise TimeoutError("Result release was not requested")
            for goal, order in zip(accepted_goals, orders):
                sequence = _fibonacci(order)
                server.complete(goal, sequence)

        server_thread, errors = _start_server_task(
            accept_four_then_complete
        )
        handles = []
        for order in orders:
            goal = Fibonacci.Goal()
            goal.order = order
            send_future = client.send_goal_async(goal)
            assert _wait_done(send_future)
            handle = send_future.result()
            assert handle.accepted
            handles.append(handle)

        overflow = Fibonacci.Goal()
        overflow.order = 9
        overflow_future = client.send_goal_async(overflow)
        assert _wait_done(overflow_future)
        overflow_accepted = overflow_future.result().accepted
        release_results.set()
        assert not overflow_accepted

        for handle, order in zip(handles, orders):
            result_future = handle.get_result_async()
            assert _wait_done(result_future)
            result = result_future.result()
            assert result.status == GoalStatus.STATUS_SUCCEEDED
            assert list(result.result.sequence) == _fibonacci(order)
        _assert_server_task(server_thread, errors)


def test_ros_action_server_rejects_goal_on_hakoniwa_response_timeout(
    tmp_path: Path,
) -> None:
    binding = _binding_with_goal_timeout(tmp_path, timeout_msec=100)
    with _bridge_runtime(
        tmp_path / "runtime",
        binding,
    ) as (server, client):
        received = threading.Event()

        def leave_goal_pending() -> None:
            _, order = server.wait_goal()
            assert order == 10
            received.set()
            time.sleep(0.3)

        server_thread, errors = _start_server_task(leave_goal_pending)
        goal = Fibonacci.Goal()
        goal.order = 10
        send_future = client.send_goal_async(goal)
        assert received.wait(3.0)
        assert _wait_done(send_future)
        assert not send_future.result().accepted
        _assert_server_task(server_thread, errors)


@contextmanager
def _bridge_runtime(
    output_dir: Path,
    binding_path: Path = BINDING,
) -> Iterator[tuple[FibonacciActionServer, ActionClient]]:
    generated = generate_action_configs(
        binding_path,
        output_dir=output_dir,
        pdu_type_resolver=lambda _ros_type, _override: (
            "sample_action_msgs/Fibonacci"
        ),
    )
    config = load_action_binding(binding_path)
    library_path = os.environ["HAKO_PDU_RPC_LIBRARY"]

    with FibonacciActionServer(
        library_path,
        generated.output_dir / "resolved-action.json",
        generated.output_dir / "endpoints.json",
    ) as server:
        server.start()
        rclpy.init()
        bridge = HakoniwaRosActionServerNode(
            config,
            generated,
            library_path,
        )
        caller = Node("fibonacci_action_test_client")
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(bridge)
        executor.add_node(caller)
        spin_thread = threading.Thread(
            target=executor.spin,
            name="ros-action-executor",
        )
        spin_thread.start()
        try:
            client = ActionClient(caller, Fibonacci, "/fibonacci")
            assert client.wait_for_server(timeout_sec=3.0)
            server.wait_connected()
            yield server, client
        finally:
            executor.shutdown(timeout_sec=5.0)
            spin_thread.join(timeout=5.0)
            executor.remove_node(caller)
            executor.remove_node(bridge)
            caller.destroy_node()
            bridge.destroy_node()
            rclpy.shutdown()


def _wait_done(future: object, timeout_sec: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if future.done():
            return True
        time.sleep(0.01)
    return future.done()


def _start_server_task(
    callback: Callable[[], None],
) -> tuple[threading.Thread, list[BaseException]]:
    errors: list[BaseException] = []

    def run() -> None:
        try:
            callback()
        except BaseException as error:
            errors.append(error)

    thread = threading.Thread(target=run, name="action-server-scenario")
    thread.start()
    return thread, errors


def _assert_server_task(
    thread: threading.Thread,
    errors: list[BaseException],
) -> None:
    thread.join(timeout=5.0)
    assert not thread.is_alive()
    assert not errors


def _send_goal_and_wait_result(
    client: ActionClient,
    order: int,
) -> list[int]:
    goal = Fibonacci.Goal()
    goal.order = order
    send_future = client.send_goal_async(goal)
    assert _wait_done(send_future)
    goal_handle = send_future.result()
    assert goal_handle.accepted
    result_future = goal_handle.get_result_async()
    assert _wait_done(result_future)
    wrapped_result = result_future.result()
    assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
    return list(wrapped_result.result.sequence)


def _fibonacci(order: int) -> list[int]:
    sequence = [0]
    if order >= 2:
        sequence.append(1)
    while len(sequence) < order:
        sequence.append(sequence[-1] + sequence[-2])
    return sequence


def _binding_with_goal_timeout(
    output_dir: Path,
    *,
    timeout_msec: int,
) -> Path:
    raw = json.loads(BINDING.read_text(encoding="utf-8"))
    raw["action"]["transport_config"] = str(
        (BINDING.parent / "fibonacci-transport.json").resolve()
    )
    raw["bindings"][0]["goal_response_timeout_msec"] = timeout_msec
    path = output_dir / "fibonacci-timeout.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(raw, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path
