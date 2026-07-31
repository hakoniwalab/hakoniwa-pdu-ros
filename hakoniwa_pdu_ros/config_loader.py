from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from hakoniwa_pdu_ros.pdu_definition import PduDefinition


@dataclass(frozen=True)
class PduKeyConfig:
    robot_name: str
    pdu_name: str


@dataclass(frozen=True)
class QosConfig:
    history: str = "keep_last"
    depth: int = 10
    reliability: str = "reliable"
    durability: str = "volatile"


@dataclass(frozen=True)
class BindingConfig:
    direction: str
    pdu_key: PduKeyConfig
    topic: str
    channel_id: int
    pdu_size: int
    pdu_type: str
    qos: QosConfig


@dataclass(frozen=True)
class BindingRootConfig:
    endpoint_config: Path
    pdu_def_path: Path
    bindings: list[BindingConfig]


def load_config(path: str | Path) -> BindingRootConfig:
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as f:
        raw = json.load(f)

    endpoint_config = _resolve_path(config_path.parent, raw["endpoint_config"])
    endpoint_config_json = _load_json(endpoint_config)
    pdu_def_path = _resolve_path(endpoint_config.parent, endpoint_config_json["pdu_def_path"])

    pdu_definition = PduDefinition()
    pdu_definition.load(pdu_def_path)

    bindings = [
        binding
        for entry in raw["bindings"]
        for binding in _parse_binding(entry, pdu_definition)
    ]
    _validate_unique_topics(bindings)
    return BindingRootConfig(
        endpoint_config=endpoint_config,
        pdu_def_path=pdu_def_path,
        bindings=bindings,
    )


def _parse_binding(entry: dict, pdu_definition: PduDefinition) -> list[BindingConfig]:
    raw_direction = entry.get("direction")
    if raw_direction is not None and raw_direction not in {"pdu_to_ros", "ros_to_pdu"}:
        raise ValueError(f"Unsupported binding direction: {raw_direction}")

    if raw_direction is None:
        directions = ["pdu_to_ros", "ros_to_pdu"]
    else:
        directions = [raw_direction]

    pdu_key_entry = entry["pdu_key"]
    robot_name = pdu_key_entry["robot_name"]
    pdu_name = pdu_key_entry["pdu_name"]
    pdu = pdu_definition.get(robot_name, pdu_name)
    ros_topic = entry.get("topic", _default_ros_topic(robot_name, pdu_name))
    _validate_ros_topic(ros_topic)
    qos = _parse_qos(entry.get("qos"))

    return [
        BindingConfig(
            direction=direction,
            pdu_key=PduKeyConfig(robot_name=robot_name, pdu_name=pdu_name),
            topic=_topic_for_direction(direction, ros_topic),
            channel_id=pdu.channel_id,
            pdu_size=pdu.pdu_size,
            pdu_type=pdu.type,
            qos=qos,
        )
        for direction in directions
    ]


def _parse_qos(raw: object) -> QosConfig:
    if raw is None:
        return QosConfig()
    if not isinstance(raw, dict):
        raise ValueError("Binding qos must be an object")

    allowed_keys = {"history", "depth", "reliability", "durability"}
    unknown_keys = sorted(set(raw) - allowed_keys)
    if unknown_keys:
        raise ValueError(f"Unsupported binding qos fields: {', '.join(unknown_keys)}")

    history = _parse_qos_choice(
        raw,
        "history",
        default="keep_last",
        allowed={"keep_last", "keep_all"},
    )
    reliability = _parse_qos_choice(
        raw,
        "reliability",
        default="reliable",
        allowed={"reliable", "best_effort"},
    )
    durability = _parse_qos_choice(
        raw,
        "durability",
        default="volatile",
        allowed={"volatile", "transient_local"},
    )
    depth = raw.get("depth", 10)
    if isinstance(depth, bool) or not isinstance(depth, int) or depth <= 0:
        raise ValueError("Binding qos.depth must be a positive integer")

    return QosConfig(
        history=history,
        depth=depth,
        reliability=reliability,
        durability=durability,
    )


def _parse_qos_choice(
    raw: dict,
    field: str,
    *,
    default: str,
    allowed: set[str],
) -> str:
    value = raw.get(field, default)
    if not isinstance(value, str) or value not in allowed:
        supported = ", ".join(sorted(allowed))
        raise ValueError(f"Binding qos.{field} must be one of: {supported}")
    return value


def _default_ros_topic(robot_name: str, pdu_name: str) -> str:
    return f"/{robot_name}/{pdu_name}"


def _topic_for_direction(direction: str, ros_topic: str) -> str:
    normalized = _normalize_topic(ros_topic)
    if direction == "pdu_to_ros":
        return f"/pdu{normalized}"
    return normalized


def _normalize_topic(topic: str) -> str:
    return topic if topic.startswith("/") else f"/{topic}"


def _validate_ros_topic(topic: str) -> None:
    normalized = _normalize_topic(topic)
    if normalized == "/pdu" or normalized.startswith("/pdu/"):
        raise ValueError(
            "The /pdu namespace is reserved for PDU-owned mirror topics. "
            f"Use a ROS-owned topic outside /pdu: {normalized}"
        )


def _validate_unique_topics(bindings: list[BindingConfig]) -> None:
    bindings_by_topic: dict[str, list[BindingConfig]] = {}
    for binding in bindings:
        bindings_by_topic.setdefault(binding.topic, []).append(binding)

    conflicting_topics = [topic for topic, topic_bindings in bindings_by_topic.items() if len(topic_bindings) > 1]
    if conflicting_topics:
        details = []
        for topic in sorted(conflicting_topics):
            refs = ", ".join(
                f"{binding.direction}:{binding.pdu_key.robot_name}/{binding.pdu_key.pdu_name}"
                for binding in bindings_by_topic[topic]
            )
            details.append(f"{topic} ({refs})")
        raise ValueError(
            "Refusing multiple bindings on the same ROS topic. "
            "Use unique topics so each ROS topic has exactly one bridge owner: "
            + "; ".join(details)
        )


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _resolve_path(base_dir: Path, raw_path: str) -> Path:
    path = Path(raw_path)
    return path if path.is_absolute() else (base_dir / path).resolve()
