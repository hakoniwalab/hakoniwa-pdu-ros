from __future__ import annotations

from types import SimpleNamespace

import pytest

from hakoniwa_pdu_ros.action_goal_context import ActionGoalContextRegistry


def ros_handle(seed: int):
    return SimpleNamespace(goal_id=SimpleNamespace(uuid=bytes([seed]) * 16))


def test_generates_valid_unique_hakoniwa_goal_ids() -> None:
    first = ActionGoalContextRegistry.new_hakoniwa_goal_id()
    second = ActionGoalContextRegistry.new_hakoniwa_goal_id()

    assert len(first) == 16
    assert any(first)
    assert first != second


def test_binds_ros_goal_to_accepted_hakoniwa_goal_in_callback_order() -> None:
    registry = ActionGoalContextRegistry()
    first_id = bytes(range(1, 17))
    second_id = bytes(range(17, 33))
    first_hakoniwa_handle = object()
    second_hakoniwa_handle = object()
    registry.register_hakoniwa_accepted(
        "fibonacci", first_id, first_hakoniwa_handle
    )
    registry.register_hakoniwa_accepted(
        "fibonacci", second_id, second_hakoniwa_handle
    )

    first = registry.bind_ros_accepted("fibonacci", ros_handle(1))
    second = registry.bind_ros_accepted("fibonacci", ros_handle(2))

    assert first.hakoniwa_goal_id == first_id
    assert first.hakoniwa_goal_handle is first_hakoniwa_handle
    assert registry.find_by_ros(bytes([1]) * 16) is first
    assert second.hakoniwa_goal_id == second_id
    assert registry.find_by_hakoniwa(second_id) is second


def test_keeps_each_action_acceptance_queue_independent() -> None:
    registry = ActionGoalContextRegistry()
    fibonacci_id = bytes([1]) * 16
    navigate_id = bytes([2]) * 16
    registry.register_hakoniwa_accepted("fibonacci", fibonacci_id, object())
    registry.register_hakoniwa_accepted("navigate", navigate_id, object())

    navigate = registry.bind_ros_accepted("navigate", ros_handle(3))
    fibonacci = registry.bind_ros_accepted("fibonacci", ros_handle(4))

    assert navigate.hakoniwa_goal_id == navigate_id
    assert fibonacci.hakoniwa_goal_id == fibonacci_id


def test_terminal_removal_clears_both_indexes() -> None:
    registry = ActionGoalContextRegistry()
    hakoniwa_id = bytes(range(1, 17))
    registry.register_hakoniwa_accepted("fibonacci", hakoniwa_id, object())
    context = registry.bind_ros_accepted("fibonacci", ros_handle(5))

    assert registry.remove_by_hakoniwa(hakoniwa_id) is context
    assert registry.find_by_hakoniwa(hakoniwa_id) is None
    assert registry.find_by_ros(bytes([5]) * 16) is None


def test_rejects_duplicate_and_unmatched_goal_registration() -> None:
    registry = ActionGoalContextRegistry()
    hakoniwa_id = bytes(range(1, 17))
    registry.register_hakoniwa_accepted("fibonacci", hakoniwa_id, object())

    with pytest.raises(ValueError, match="duplicate Hakoniwa"):
        registry.register_hakoniwa_accepted("fibonacci", hakoniwa_id, object())
    with pytest.raises(LookupError, match="no accepted Hakoniwa"):
        registry.bind_ros_accepted("unknown", ros_handle(6))
