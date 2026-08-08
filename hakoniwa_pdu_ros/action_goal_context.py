from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from threading import Lock
from typing import Any
from uuid import uuid4

from hakoniwa_pdu_ros.action_mapping import goal_id_from_ros


@dataclass
class ActionGoalContext:
    """Correlation owned by the ROS/Hakoniwa Action adapter.

    Protocol state remains owned by the ROS GoalHandle and hakoniwa-pdu-rpc.
    This object only relates their independent identifiers and handles.
    """

    action_name: str
    hakoniwa_goal_id: bytes
    hakoniwa_goal_handle: Any
    ros_goal_id: bytes | None = None
    ros_goal_handle: Any | None = None


class ActionGoalContextRegistry:
    """Thread-safe two-way Goal correlation without a Bridge state machine."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._by_hakoniwa: dict[bytes, ActionGoalContext] = {}
        self._by_ros: dict[bytes, ActionGoalContext] = {}
        self._accepted_order: dict[str, deque[bytes]] = defaultdict(deque)

    @staticmethod
    def new_hakoniwa_goal_id() -> bytes:
        return uuid4().bytes

    def register_hakoniwa_accepted(
        self,
        action_name: str,
        hakoniwa_goal_id: bytes,
        hakoniwa_goal_handle: Any,
    ) -> ActionGoalContext:
        goal_id = _validate_hakoniwa_goal_id(hakoniwa_goal_id)
        with self._lock:
            if goal_id in self._by_hakoniwa:
                raise ValueError("duplicate Hakoniwa goal_id")
            context = ActionGoalContext(
                action_name=action_name,
                hakoniwa_goal_id=goal_id,
                hakoniwa_goal_handle=hakoniwa_goal_handle,
            )
            self._by_hakoniwa[goal_id] = context
            self._accepted_order[action_name].append(goal_id)
            return context

    def bind_ros_accepted(
        self,
        action_name: str,
        ros_goal_handle: Any,
    ) -> ActionGoalContext:
        ros_goal_id = goal_id_from_ros(ros_goal_handle.goal_id)
        with self._lock:
            if ros_goal_id in self._by_ros:
                raise ValueError("duplicate ROS Goal UUID")
            pending = self._accepted_order.get(action_name)
            if not pending:
                raise LookupError(
                    f"no accepted Hakoniwa Goal is pending for Action {action_name!r}"
                )
            hakoniwa_goal_id = pending.popleft()
            if not pending:
                self._accepted_order.pop(action_name, None)
            context = self._by_hakoniwa[hakoniwa_goal_id]
            context.ros_goal_id = ros_goal_id
            context.ros_goal_handle = ros_goal_handle
            self._by_ros[ros_goal_id] = context
            return context

    def find_by_hakoniwa(self, goal_id: bytes) -> ActionGoalContext | None:
        with self._lock:
            return self._by_hakoniwa.get(bytes(goal_id))

    def find_by_ros(self, goal_id: object) -> ActionGoalContext | None:
        ros_goal_id = goal_id_from_ros(goal_id)
        with self._lock:
            return self._by_ros.get(ros_goal_id)

    def remove_by_hakoniwa(self, goal_id: bytes) -> ActionGoalContext | None:
        key = bytes(goal_id)
        with self._lock:
            context = self._by_hakoniwa.pop(key, None)
            if context is None:
                return None
            if context.ros_goal_id is not None:
                self._by_ros.pop(context.ros_goal_id, None)
            pending = self._accepted_order.get(context.action_name)
            if pending is not None:
                try:
                    pending.remove(key)
                except ValueError:
                    pass
                if not pending:
                    self._accepted_order.pop(context.action_name, None)
            return context

    def active_contexts(self) -> tuple[ActionGoalContext, ...]:
        with self._lock:
            return tuple(self._by_hakoniwa.values())

    def clear(self) -> tuple[ActionGoalContext, ...]:
        with self._lock:
            contexts = tuple(self._by_hakoniwa.values())
            self._by_hakoniwa.clear()
            self._by_ros.clear()
            self._accepted_order.clear()
            return contexts


def _validate_hakoniwa_goal_id(value: bytes) -> bytes:
    goal_id = bytes(value)
    if len(goal_id) != 16 or not any(goal_id):
        raise ValueError("Hakoniwa goal_id must be 16 bytes and not all zero")
    return goal_id
