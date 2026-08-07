from __future__ import annotations

import importlib
from dataclasses import dataclass
from functools import lru_cache

from hakoniwa_pdu_ros.type_mapper import copy_matching_fields


@dataclass(frozen=True)
class ActionWire:
    request_type: type
    request_from_pdu: callable
    request_to_pdu: callable
    feedback_type: type
    feedback_from_pdu: callable
    result_type: type
    result_from_pdu: callable

    def encode_goal(self, ros_goal: object, template: bytes) -> bytes:
        packet = self.request_from_pdu(bytearray(template))
        copy_matching_fields(ros_goal, packet.body)
        return bytes(self.request_to_pdu(packet))

    def decode_feedback(self, data: bytes, ros_action_type: type) -> object:
        packet = self.feedback_from_pdu(bytearray(data))
        feedback = ros_action_type.Feedback()
        copy_matching_fields(packet.body, feedback)
        return feedback

    def decode_result(self, data: bytes, ros_action_type: type) -> object:
        packet = self.result_from_pdu(bytearray(data))
        result = ros_action_type.Result()
        copy_matching_fields(packet.body, result)
        return result


@lru_cache(maxsize=None)
def load_action_wire(type_name: str) -> ActionWire:
    package_name, action_name = type_name.split("/", 1)
    if not package_name or not action_name:
        raise ValueError(
            f"PDU Action type must use package/Type form: {type_name}"
        )

    request_name = f"{action_name}ActionRequest"
    feedback_name = f"{action_name}ActionFeedback"
    result_name = f"{action_name}ActionResponse"
    package = f"hakoniwa_pdu.pdu_msgs.{package_name}"

    request_type, request_from_pdu, request_to_pdu = _load_packet(
        package, request_name
    )
    feedback_type, feedback_from_pdu, _ = _load_packet(package, feedback_name)
    result_type, result_from_pdu, _ = _load_packet(package, result_name)
    return ActionWire(
        request_type=request_type,
        request_from_pdu=request_from_pdu,
        request_to_pdu=request_to_pdu,
        feedback_type=feedback_type,
        feedback_from_pdu=feedback_from_pdu,
        result_type=result_type,
        result_from_pdu=result_from_pdu,
    )


def _load_packet(package: str, packet_name: str) -> tuple[type, callable, callable]:
    conv = importlib.import_module(f"{package}.pdu_conv_{packet_name}")
    pytype = importlib.import_module(f"{package}.pdu_pytype_{packet_name}")
    return (
        getattr(pytype, packet_name),
        getattr(conv, f"pdu_to_py_{packet_name}"),
        getattr(conv, f"py_to_pdu_{packet_name}"),
    )
