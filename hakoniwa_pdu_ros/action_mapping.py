from __future__ import annotations

import importlib
from dataclasses import dataclass
from enum import Enum

from hakoniwa_pdu_ros.type_mapper import copy_matching_fields


class RosGoalTerminalAction(Enum):
    SUCCEED = "succeed"
    CANCELED = "canceled"
    ABORT = "abort"


class ActionConversionError(ValueError):
    def __init__(
        self,
        *,
        direction: str,
        ros_type: str,
        pdu_type: str,
        cause: BaseException,
    ) -> None:
        self.direction = direction
        self.ros_type = ros_type
        self.pdu_type = pdu_type
        self.cause = cause
        super().__init__(
            "action conversion failed: "
            f"direction={direction} ros_type={ros_type} "
            f"pdu_type={pdu_type} error={cause}"
        )


@dataclass(frozen=True)
class RosActionServerTypeMapper:
    """Map ROS messages to typed PDU Action body objects.

    Packet classes, converters, headers and wire buffers are owned by
    ``hakoniwa-pdu-rpc.TypedActionClient`` and never loaded here.
    """

    ros_type: str
    pdu_type: str
    ros_goal_type: type
    ros_feedback_type: type
    ros_result_type: type
    ros_action_type: type | None = None

    @classmethod
    def load(cls, ros_type: str, pdu_type: str) -> "RosActionServerTypeMapper":
        ros_package, ros_namespace, action_name = ros_type.split("/", 2)
        if ros_namespace != "action":
            raise ValueError(
                f"ROS action type must use package/action/Type form: {ros_type}"
            )
        pdu_parts = pdu_type.split("/", 1)
        if len(pdu_parts) != 2 or not all(pdu_parts):
            raise ValueError(f"PDU type must use package/Type form: {pdu_type}")
        if pdu_parts[1] != action_name:
            raise ValueError(
                "PDU action type basename must match ROS action type: "
                f"{pdu_type} != {ros_type}"
            )

        try:
            ros_action = getattr(
                importlib.import_module(f"{ros_package}.action"), action_name
            )
        except (ModuleNotFoundError, AttributeError) as error:
            raise ValueError(
                f"ROS Action type support is not installed: {ros_type}"
            ) from error

        return cls(
            ros_type=ros_type,
            pdu_type=pdu_type,
            ros_goal_type=ros_action.Goal,
            ros_feedback_type=ros_action.Feedback,
            ros_result_type=ros_action.Result,
            ros_action_type=ros_action,
        )

    def goal_to_typed(self, ros_goal: object, typed_goal: object) -> object:
        return self._copy(
            ros_goal,
            typed_goal,
            direction="ros_goal_to_typed_pdu",
        )

    def feedback_to_ros(self, typed_feedback: object) -> object:
        return self._copy(
            typed_feedback,
            self.ros_feedback_type(),
            direction="typed_pdu_feedback_to_ros",
        )

    def result_to_ros(self, typed_result: object) -> object:
        return self._copy(
            typed_result,
            self.ros_result_type(),
            direction="typed_pdu_result_to_ros",
        )

    def _copy(self, source: object, target: object, *, direction: str) -> object:
        try:
            return copy_matching_fields(source, target)
        except BaseException as error:
            raise ActionConversionError(
                direction=direction,
                ros_type=self.ros_type,
                pdu_type=self.pdu_type,
                cause=error,
            ) from error


@dataclass(frozen=True)
class RosActionClientTypeMapper:
    """Map typed PDU Action bodies to ROS Action Client messages.

    Packet allocation, headers and wire conversion stay inside
    ``hakoniwa-pdu-rpc.TypedActionServer``.  This mapper owns only the
    application body conversion at the ROS boundary.
    """

    ros_type: str
    pdu_type: str
    ros_goal_type: type
    ros_feedback_type: type
    ros_result_type: type
    ros_action_type: type | None = None

    @classmethod
    def load(cls, ros_type: str, pdu_type: str) -> "RosActionClientTypeMapper":
        support = RosActionServerTypeMapper.load(ros_type, pdu_type)
        return cls(
            ros_type=support.ros_type,
            pdu_type=support.pdu_type,
            ros_goal_type=support.ros_goal_type,
            ros_feedback_type=support.ros_feedback_type,
            ros_result_type=support.ros_result_type,
            ros_action_type=support.ros_action_type,
        )

    def goal_to_ros(self, typed_goal: object) -> object:
        return self._copy(
            typed_goal,
            self.ros_goal_type(),
            direction="typed_pdu_goal_to_ros",
        )

    def feedback_to_typed(
        self,
        ros_feedback: object,
        typed_feedback: object,
    ) -> object:
        return self._copy(
            ros_feedback,
            typed_feedback,
            direction="ros_feedback_to_typed_pdu",
        )

    def result_to_typed(
        self,
        ros_result: object,
        typed_result: object,
    ) -> object:
        return self._copy(
            ros_result,
            typed_result,
            direction="ros_result_to_typed_pdu",
        )

    def _copy(self, source: object, target: object, *, direction: str) -> object:
        try:
            return copy_matching_fields(source, target)
        except BaseException as error:
            raise ActionConversionError(
                direction=direction,
                ros_type=self.ros_type,
                pdu_type=self.pdu_type,
                cause=error,
            ) from error

def goal_id_from_ros(goal_id: object) -> bytes:
    """Convert a ROS UUID message (or its sequence) to a native Goal ID."""

    value = getattr(goal_id, "uuid", goal_id)
    try:
        result = bytes(value)
    except (TypeError, ValueError) as error:
        raise ValueError("ROS Goal UUID must be a 16-byte sequence") from error
    if len(result) != 16:
        raise ValueError("ROS Goal UUID must contain exactly 16 bytes")
    if not any(result):
        raise ValueError("ROS Goal UUID must not be all zero")
    return result


def terminal_action_for(status: object) -> RosGoalTerminalAction:
    """Map a PDU-RPC terminal status to the ROS GoalHandle operation."""

    name = status if isinstance(status, str) else getattr(status, "name", None)
    mapping = {
        "SUCCEEDED": RosGoalTerminalAction.SUCCEED,
        "CANCELED": RosGoalTerminalAction.CANCELED,
        "ABORTED": RosGoalTerminalAction.ABORT,
    }
    try:
        return mapping[name]
    except (KeyError, TypeError) as error:
        raise ValueError(f"Unsupported Action terminal status: {status!r}") from error
