from __future__ import annotations

import os
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

import rclpy
from action_tutorials_interfaces.action import Fibonacci
from rclpy.action import (
    ActionServer as RosActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hakoniwa_pdu_rpc import (
    ActionClient as HakoniwaActionClient,
    ActionClientEvent,
    ActionDecision,
    ActionTerminalStatus,
    make_typed_action_client,
)
from hakoniwa_pdu_ros.action_binding import load_action_binding
from hakoniwa_pdu_ros.action_client_node import HakoniwaRosActionClientNode
from hakoniwa_pdu_ros.action_config_generator import generate_action_configs


REPO_ROOT = Path(__file__).resolve().parents[2]
BINDING = REPO_ROOT / "config" / "action" / "fibonacci.json"
ACTION_NAME = "fibonacci"


class FibonacciRosActionServer(Node):
    def __init__(self) -> None:
        super().__init__("fibonacci_ros_action_server")
        self.feedback_sequences: list[list[int]] = []
        self.server = RosActionServer(
            self,
            Fibonacci,
            "/fibonacci",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )

    def _goal(self, request: Fibonacci.Goal) -> GoalResponse:
        return GoalResponse.REJECT if request.order <= 0 else GoalResponse.ACCEPT

    def _cancel(self, goal_handle: object) -> CancelResponse:
        if goal_handle.request.order == 30:
            return CancelResponse.REJECT
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle: object) -> Fibonacci.Result:
        order = goal_handle.request.order
        sequence = [0]
        if order >= 2:
            sequence.append(1)
        while len(sequence) < order:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = sequence
                return result
            sequence.append(sequence[-1] + sequence[-2])
            feedback = Fibonacci.Feedback()
            feedback.partial_sequence = sequence
            self.feedback_sequences.append(list(sequence))
            goal_handle.publish_feedback(feedback)
            time.sleep(0.01)
        if order == 3:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        return result

    def destroy_node(self) -> bool:
        self.server.destroy()
        return super().destroy_node()


def test_hakoniwa_action_client_bridges_goal_feedback_result_and_reject(
    tmp_path: Path,
) -> None:
    with _bridge_runtime(tmp_path) as (server, typed_client, action):
        goal_id = bytes(range(1, 17))
        goal = action.create_goal()
        goal.order = 8
        handle = action.send_goal(goal, goal_id, timeout_usec=3_000_000)

        events = _collect_until_result(typed_client, handle.goal_id)
        assert events[0].event == ActionClientEvent.GOAL_RESPONSE
        assert events[0].decision == ActionDecision.ACCEPTED
        feedback = [
            list(event.feedback.partial_sequence)
            for event in events
            if event.event == ActionClientEvent.FEEDBACK
        ]
        result = events[-1]
        expected = [0, 1, 1, 2, 3, 5, 8, 13]
        assert feedback == server.feedback_sequences
        assert feedback[-1] == expected
        assert result.event == ActionClientEvent.RESULT
        assert result.terminal_status == ActionTerminalStatus.SUCCEEDED
        assert list(result.result.sequence) == expected

        rejected = action.create_goal()
        rejected.order = 0
        rejected_handle = action.send_goal(
            rejected,
            bytes([0x21]) * 16,
            timeout_usec=3_000_000,
        )
        response = _wait_event(
            typed_client,
            rejected_handle.goal_id,
            ActionClientEvent.GOAL_RESPONSE,
        )
        assert response.decision == ActionDecision.REJECTED

        aborted = action.create_goal()
        aborted.order = 3
        aborted_handle = action.send_goal(
            aborted,
            bytes([0x22]) * 16,
            timeout_usec=3_000_000,
        )
        aborted_result = _wait_event(
            typed_client,
            aborted_handle.goal_id,
            ActionClientEvent.RESULT,
        )
        assert aborted_result.terminal_status == ActionTerminalStatus.ABORTED
        assert list(aborted_result.result.sequence) == [0, 1, 1]


def test_hakoniwa_action_client_bridges_cancel_accept_and_reject(
    tmp_path: Path,
) -> None:
    with _bridge_runtime(tmp_path) as (_server, typed_client, action):
        accepted_goal = action.create_goal()
        accepted_goal.order = 40
        accepted_handle = action.send_goal(
            accepted_goal,
            bytes([0x31]) * 16,
            timeout_usec=3_000_000,
        )
        response = _wait_event(
            typed_client,
            accepted_handle.goal_id,
            ActionClientEvent.GOAL_RESPONSE,
        )
        assert response.decision == ActionDecision.ACCEPTED
        action.cancel_goal(accepted_handle)
        cancel_response = _wait_event(
            typed_client,
            accepted_handle.goal_id,
            ActionClientEvent.CANCEL_RESPONSE,
        )
        assert cancel_response.decision == ActionDecision.ACCEPTED
        canceled = _wait_event(
            typed_client,
            accepted_handle.goal_id,
            ActionClientEvent.RESULT,
        )
        assert canceled.terminal_status == ActionTerminalStatus.CANCELED

        rejected_goal = action.create_goal()
        rejected_goal.order = 30
        rejected_handle = action.send_goal(
            rejected_goal,
            bytes([0x41]) * 16,
            timeout_usec=3_000_000,
        )
        response = _wait_event(
            typed_client,
            rejected_handle.goal_id,
            ActionClientEvent.GOAL_RESPONSE,
        )
        assert response.decision == ActionDecision.ACCEPTED
        action.cancel_goal(rejected_handle)
        cancel_response = _wait_event(
            typed_client,
            rejected_handle.goal_id,
            ActionClientEvent.CANCEL_RESPONSE,
        )
        assert cancel_response.decision == ActionDecision.REJECTED
        result = _wait_event(
            typed_client,
            rejected_handle.goal_id,
            ActionClientEvent.RESULT,
            timeout_sec=5.0,
        )
        assert result.terminal_status == ActionTerminalStatus.SUCCEEDED


@contextmanager
def _bridge_runtime(
    output_dir: Path,
) -> Iterator[tuple[FibonacciRosActionServer, object, object]]:
    generated = generate_action_configs(
        BINDING,
        output_dir=output_dir,
        pdu_type_resolver=lambda _ros_type, _override: (
            "sample_action_msgs/Fibonacci"
        ),
    )
    config = load_action_binding(BINDING)
    library_path = os.environ["HAKO_PDU_RPC_LIBRARY"]

    rclpy.init()
    ros_server = FibonacciRosActionServer()
    bridge = HakoniwaRosActionClientNode(config, generated, library_path)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ros_server)
    executor.add_node(bridge)
    spin_thread = threading.Thread(
        target=executor.spin,
        name="ros-action-client-bridge-executor",
    )
    spin_thread.start()
    raw_client = HakoniwaActionClient(
        library_path,
        "fibonacci-client",
        "native-action-client-bridge-test",
        generated.output_dir / "resolved-action.json",
        generated.output_dir / "endpoints.json",
    )
    try:
        raw_client.start()
        typed_client = make_typed_action_client(
            raw_client,
            generated.output_dir / "resolved-action.json",
            packages={ACTION_NAME: "pdu.python.sample_action_msgs"},
        )
        deadline = time.monotonic() + 5.0
        while not raw_client.is_running() and time.monotonic() < deadline:
            time.sleep(0.001)
        assert raw_client.is_running()
        yield ros_server, typed_client, typed_client.action(ACTION_NAME)
    finally:
        raw_client.close()
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        executor.remove_node(bridge)
        executor.remove_node(ros_server)
        bridge.destroy_node()
        ros_server.destroy_node()
        rclpy.shutdown()


def _collect_until_result(
    typed_client: object,
    goal_id: bytes,
    timeout_sec: float = 5.0,
) -> list[object]:
    events = []
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        event = typed_client.poll()
        if event.event == ActionClientEvent.NONE:
            time.sleep(0.001)
            continue
        if event.goal is None or event.goal.goal_id != goal_id:
            continue
        events.append(event)
        if event.event == ActionClientEvent.RESULT:
            return events
    raise TimeoutError("Hakoniwa Action Result was not received")


def _wait_event(
    typed_client: object,
    goal_id: bytes,
    expected: ActionClientEvent,
    timeout_sec: float = 3.0,
) -> object:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        event = typed_client.poll()
        if event.event == ActionClientEvent.NONE:
            time.sleep(0.001)
            continue
        if event.goal is None or event.goal.goal_id != goal_id:
            continue
        if event.event == expected:
            return event
    raise TimeoutError(f"Hakoniwa Action {expected.name} was not received")
