from __future__ import annotations

from enum import Enum
from types import SimpleNamespace

import pytest

from hakoniwa_pdu_ros.action_mapping import (
    ActionConversionError,
    RosActionServerTypeMapper,
    RosGoalTerminalAction,
    goal_id_from_ros,
    terminal_action_for,
)


class Goal:
    def __init__(self) -> None:
        self.order = 0


class Feedback:
    def __init__(self) -> None:
        self.partial_sequence = []


class Result:
    def __init__(self) -> None:
        self.sequence = []


def mapper() -> RosActionServerTypeMapper:
    return RosActionServerTypeMapper(
        ros_type="sample_action_msgs/action/Fibonacci",
        pdu_type="sample_action_msgs/Fibonacci",
        ros_goal_type=Goal,
        ros_feedback_type=Feedback,
        ros_result_type=Result,
    )


def test_ros_goal_uuid_is_preserved_as_exactly_16_bytes() -> None:
    value = bytes(range(1, 17))
    assert goal_id_from_ros(SimpleNamespace(uuid=list(value))) == value
    assert goal_id_from_ros(value) == value


@pytest.mark.parametrize("value", [bytes(15), bytes(17), bytes(16)])
def test_rejects_invalid_or_zero_ros_goal_uuid(value: bytes) -> None:
    with pytest.raises(ValueError, match="Goal UUID"):
        goal_id_from_ros(value)


def test_goal_mapping_uses_a_typed_body_supplied_by_rpc() -> None:
    ros_goal = Goal()
    ros_goal.order = 10
    typed_goal = SimpleNamespace(order=0)

    mapped = mapper().goal_to_typed(ros_goal, typed_goal)

    assert mapped is typed_goal
    assert mapped.order == 10


def test_feedback_and_result_map_typed_bodies_without_packet_knowledge() -> None:
    support = mapper()

    feedback = support.feedback_to_ros(
        SimpleNamespace(partial_sequence=[1, 1, 2])
    )
    result = support.result_to_ros(SimpleNamespace(sequence=[0, 1, 1, 2, 3]))

    assert feedback.partial_sequence == [1, 1, 2]
    assert result.sequence == [0, 1, 1, 2, 3]


def test_conversion_failure_contains_direction_and_types(monkeypatch) -> None:
    def fail(_source, _target):
        raise RuntimeError("broken body")

    monkeypatch.setattr(
        "hakoniwa_pdu_ros.action_mapping.copy_matching_fields", fail
    )

    with pytest.raises(ActionConversionError) as raised:
        mapper().result_to_ros(SimpleNamespace(sequence=[]))

    assert raised.value.direction == "typed_pdu_result_to_ros"
    assert raised.value.ros_type == "sample_action_msgs/action/Fibonacci"
    assert raised.value.pdu_type == "sample_action_msgs/Fibonacci"
    assert "broken body" in str(raised.value)


class TerminalStatus(Enum):
    SUCCEEDED = 1
    CANCELED = 2
    ABORTED = 3
    UNSPECIFIED = 0


@pytest.mark.parametrize(
    ("status", "expected"),
    [
        (TerminalStatus.SUCCEEDED, RosGoalTerminalAction.SUCCEED),
        (TerminalStatus.CANCELED, RosGoalTerminalAction.CANCELED),
        (TerminalStatus.ABORTED, RosGoalTerminalAction.ABORT),
    ],
)
def test_maps_terminal_status_to_ros_goal_operation(status, expected) -> None:
    assert terminal_action_for(status) is expected


def test_rejects_nonterminal_status() -> None:
    with pytest.raises(ValueError, match="Unsupported Action terminal status"):
        terminal_action_for(TerminalStatus.UNSPECIFIED)
