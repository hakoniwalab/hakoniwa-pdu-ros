from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

from hakoniwa_pdu_ros.action_client_runtime import ActionClientRuntime


class Event(Enum):
    NONE = 0
    GOAL_RESPONSE = 1
    FEEDBACK = 2
    CANCEL_RESPONSE = 3
    RESULT = 4


@dataclass(frozen=True)
class Goal:
    goal_id: bytes


@dataclass(frozen=True)
class PollResult:
    event: Event
    goal: Goal | None = None
    decision: object | None = None
    pdu: bytes = b""


class Decision(Enum):
    ACCEPTED = 1
    REJECTED = 2


class FakeClient:
    def __init__(self) -> None:
        self.events: list[PollResult] = []
        self.running = False
        self.closed = False

    def start(self) -> None:
        self.running = True

    def is_running(self) -> bool:
        return self.running

    def send_goal(self, action_name, pdu, goal_id, timeout_usec=0):
        del action_name, pdu, timeout_usec
        goal = Goal(bytes(goal_id))
        self.events.append(
            PollResult(Event.GOAL_RESPONSE, goal, Decision.ACCEPTED)
        )
        return goal

    def cancel_goal(self, action_name, goal):
        del action_name
        self.events.append(
            PollResult(Event.CANCEL_RESPONSE, goal, Decision.ACCEPTED)
        )

    def poll(self):
        if self.events:
            return self.events.pop(0)
        return PollResult(Event.NONE)

    def stop(self) -> None:
        self.running = False

    def close(self) -> None:
        self.closed = True


def _goal_id(seed: int) -> bytes:
    return bytes((seed + index) & 0xFF for index in range(16))


def test_runtime_dispatches_two_goals_independently():
    client = FakeClient()
    runtime = ActionClientRuntime(client)
    runtime.start()
    try:
        first, accepted1 = runtime.submit_goal(
            "fibonacci", b"goal-1", _goal_id(0x10), timeout_usec=100_000
        )
        second, accepted2 = runtime.submit_goal(
            "fibonacci", b"goal-2", _goal_id(0x40), timeout_usec=100_000
        )
        assert accepted1.decision == Decision.ACCEPTED
        assert accepted2.decision == Decision.ACCEPTED

        client.events.extend(
            [
                PollResult(Event.FEEDBACK, second.goal, pdu=b"second"),
                PollResult(Event.FEEDBACK, first.goal, pdu=b"first"),
            ]
        )
        event1 = runtime.wait_for(first, {"FEEDBACK"}, timeout_sec=1.0)
        event2 = runtime.wait_for(second, {"FEEDBACK"}, timeout_sec=1.0)
        assert event1.pdu == b"first"
        assert event2.pdu == b"second"
    finally:
        runtime.close()


def test_cancel_wait_does_not_consume_feedback_or_result():
    client = FakeClient()
    runtime = ActionClientRuntime(client)
    runtime.start()
    try:
        session, _ = runtime.submit_goal(
            "fibonacci", b"goal", _goal_id(0x70), timeout_usec=100_000
        )
        client.events.extend(
            [
                PollResult(Event.FEEDBACK, session.goal, pdu=b"feedback"),
                PollResult(Event.RESULT, session.goal, pdu=b"result"),
            ]
        )
        cancel = runtime.cancel(session, timeout_sec=1.0)
        assert cancel.event == Event.CANCEL_RESPONSE
        feedback = runtime.wait_for(session, {"FEEDBACK"}, timeout_sec=1.0)
        result = runtime.wait_for(session, {"RESULT"}, timeout_sec=1.0)
        assert feedback.pdu == b"feedback"
        assert result.pdu == b"result"
    finally:
        runtime.close()


def test_close_stops_and_destroys_client():
    client = FakeClient()
    runtime = ActionClientRuntime(client)
    runtime.start()
    time.sleep(0.01)
    runtime.close()
    assert not client.running
    assert client.closed
