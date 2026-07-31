from __future__ import annotations

from typing import Any, Callable

from hakoniwa_pdu_ros.config_loader import QosConfig


def to_rclpy_qos_profile(config: QosConfig) -> Any:
    """Translate the binding-level QoS contract into an rclpy QoSProfile."""
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    return QoSProfile(
        history={
            "keep_last": HistoryPolicy.KEEP_LAST,
            "keep_all": HistoryPolicy.KEEP_ALL,
        }[config.history],
        depth=config.depth,
        reliability={
            "reliable": ReliabilityPolicy.RELIABLE,
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
        }[config.reliability],
        durability={
            "volatile": DurabilityPolicy.VOLATILE,
            "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
        }[config.durability],
    )


def describe_qos(config: QosConfig) -> str:
    return (
        f"history={config.history}, depth={config.depth}, "
        f"reliability={config.reliability}, durability={config.durability}"
    )


def make_incompatible_qos_callback(
    warn: Callable[[str], None],
    topic: str,
    config: QosConfig,
) -> Callable[[object], None]:
    """Build a ROS-event callback without leaking ROS types into config tests."""

    def _on_incompatible_qos(event: object) -> None:
        policy_kind = getattr(event, "last_policy_kind", "unknown")
        warn(
            f"incompatible publisher QoS detected for subscription {topic}: "
            f"requested {describe_qos(config)}, last_policy_kind={policy_kind}"
        )

    return _on_incompatible_qos
