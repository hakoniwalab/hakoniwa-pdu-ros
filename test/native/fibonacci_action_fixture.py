from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from hakoniwa_pdu_rpc import (
    ActionServer,
    ActionServerEvent,
    ActionTerminalStatus,
    ServerGoalHandle,
    load_action_wire,
)


ACTION_NAME = "fibonacci"
SERVER_NODE_ID = "fibonacci-server"


class FibonacciActionServer:
    """Small Hakoniwa Action Server used by the real ROS bridge tests."""

    def __init__(
        self,
        library_path: str | Path,
        action_config_path: str | Path,
        endpoint_config_path: str | Path,
    ) -> None:
        self._server = ActionServer(
            library_path,
            SERVER_NODE_ID,
            action_config_path,
            endpoint_config_path,
        )
        # The native RPC source checkout carries the generated Registry
        # package used until sample_action_msgs is published in hakoniwa-pdu.
        self._wire = load_action_wire(
            "sample_action_msgs/Fibonacci",
            package="pdu.python.sample_action_msgs",
        )
        self.observed_order: int | None = None
        self.feedback_sequences: list[list[int]] = []

    def start(self) -> None:
        self._server.start()

    def wait_connected(self, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._server.is_running():
                return
            time.sleep(0.001)
        raise TimeoutError("Fibonacci Action TCP connection timed out")

    def serve_once(self, timeout_sec: float = 5.0) -> list[int]:
        incoming, order = self.wait_goal(timeout_sec)
        self.observed_order = order
        if order <= 0 or order > 47:
            self.reject_goal(incoming)
            return []

        self.accept_goal(incoming)
        sequence = [0]
        if order >= 2:
            sequence.append(1)
        while len(sequence) < order:
            sequence.append(sequence[-1] + sequence[-2])
            self.send_feedback(incoming, sequence)
            time.sleep(0.01)
        self.complete(incoming, sequence)
        return sequence

    def wait_goal(
        self,
        timeout_sec: float = 5.0,
    ) -> tuple[ServerGoalHandle, int]:
        incoming = self._wait_event(ActionServerEvent.GOAL_REQUEST, timeout_sec)
        request = self._wire.request_decode(incoming.pdu)
        return incoming.goal, int(request.body.order)

    def wait_cancel(self, timeout_sec: float = 5.0) -> ServerGoalHandle:
        return self._wait_event(
            ActionServerEvent.CANCEL_REQUEST,
            timeout_sec,
        ).goal

    def _wait_event(
        self,
        expected: ActionServerEvent,
        timeout_sec: float,
    ) -> Any:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            incoming = self._server.poll()
            if incoming.event == ActionServerEvent.NONE:
                time.sleep(0.001)
                continue
            if incoming.event != expected:
                raise RuntimeError(
                    f"unexpected Action server event: {incoming.event}"
                )
            if incoming.action_name != ACTION_NAME or incoming.goal is None:
                raise RuntimeError(
                    f"unexpected Action event: {incoming.action_name!r}"
                )
            return incoming

        raise TimeoutError(f"Fibonacci Action {expected.name} was not received")

    def accept_goal(self, goal: ServerGoalHandle) -> None:
        self._server.accept_goal(ACTION_NAME, goal)

    def reject_goal(self, goal: ServerGoalHandle) -> None:
        self._server.reject_goal(ACTION_NAME, goal)

    def accept_cancel(self, goal: ServerGoalHandle) -> None:
        self._server.accept_cancel(ACTION_NAME, goal)

    def reject_cancel(self, goal: ServerGoalHandle) -> None:
        self._server.reject_cancel(ACTION_NAME, goal)

    def send_feedback(
        self,
        goal: ServerGoalHandle,
        sequence: list[int],
    ) -> None:
        buffer = self._server.create_feedback_buffer(ACTION_NAME)
        packet = self._wire.feedback_decode(buffer)
        packet.body.partial_sequence = list(sequence)
        self._server.send_feedback(
            ACTION_NAME,
            goal,
            self._wire.feedback_encode(packet),
        )
        self.feedback_sequences.append(list(sequence))

    def complete(
        self,
        goal: ServerGoalHandle,
        sequence: list[int],
        status: ActionTerminalStatus = ActionTerminalStatus.SUCCEEDED,
    ) -> None:
        buffer = self._server.create_result_buffer(ACTION_NAME)
        packet = self._wire.response_decode(buffer)
        packet.body.sequence = list(sequence)
        self._server.complete(
            ACTION_NAME,
            goal,
            status,
            self._wire.response_encode(packet),
        )

    def close(self) -> None:
        self._server.close()

    def __enter__(self) -> "FibonacciActionServer":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()
