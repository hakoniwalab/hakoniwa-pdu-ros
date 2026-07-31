import sys
import types
from enum import Enum

from hakoniwa_pdu_ros.config_loader import QosConfig
from hakoniwa_pdu_ros.qos import (
    describe_qos,
    make_incompatible_qos_callback,
    to_rclpy_qos_profile,
)


class _HistoryPolicy(Enum):
    KEEP_LAST = 1
    KEEP_ALL = 2


class _ReliabilityPolicy(Enum):
    RELIABLE = 1
    BEST_EFFORT = 2


class _DurabilityPolicy(Enum):
    VOLATILE = 1
    TRANSIENT_LOCAL = 2


class _QoSProfile:
    def __init__(self, **kwargs: object) -> None:
        self.__dict__.update(kwargs)


def test_translate_best_effort_qos_to_rclpy(monkeypatch) -> None:
    qos_module = types.ModuleType("rclpy.qos")
    qos_module.HistoryPolicy = _HistoryPolicy
    qos_module.ReliabilityPolicy = _ReliabilityPolicy
    qos_module.DurabilityPolicy = _DurabilityPolicy
    qos_module.QoSProfile = _QoSProfile
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos_module)

    profile = to_rclpy_qos_profile(
        QosConfig(
            history="keep_all",
            depth=4,
            reliability="best_effort",
            durability="transient_local",
        )
    )

    assert profile.history is _HistoryPolicy.KEEP_ALL
    assert profile.depth == 4
    assert profile.reliability is _ReliabilityPolicy.BEST_EFFORT
    assert profile.durability is _DurabilityPolicy.TRANSIENT_LOCAL


def test_describe_qos_is_explicit() -> None:
    assert describe_qos(QosConfig(reliability="best_effort")) == (
        "history=keep_last, depth=10, "
        "reliability=best_effort, durability=volatile"
    )


def test_incompatible_qos_callback_reports_requested_profile() -> None:
    warnings: list[str] = []
    callback = make_incompatible_qos_callback(
        warnings.append,
        "/joint_states",
        QosConfig(reliability="best_effort"),
    )

    callback(types.SimpleNamespace(last_policy_kind=7))

    assert len(warnings) == 1
    assert "/joint_states" in warnings[0]
    assert "reliability=best_effort" in warnings[0]
    assert "last_policy_kind=7" in warnings[0]
