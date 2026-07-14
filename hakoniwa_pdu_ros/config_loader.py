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
class BindingConfig:
    direction: str
    pdu_key: PduKeyConfig
    topic: str
    channel_id: int
    pdu_size: int
    pdu_type: str


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
    _validate_no_bidirectional_topic_conflicts(bindings)
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

    return [
        BindingConfig(
            direction=direction,
            pdu_key=PduKeyConfig(robot_name=robot_name, pdu_name=pdu_name),
            topic=_topic_for_direction(direction, ros_topic),
            channel_id=pdu.channel_id,
            pdu_size=pdu.pdu_size,
            pdu_type=pdu.type,
        )
        for direction in directions
    ]


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


def _validate_no_bidirectional_topic_conflicts(bindings: list[BindingConfig]) -> None:
    directions_by_topic: dict[str, set[str]] = {}
    for binding in bindings:
        directions_by_topic.setdefault(binding.topic, set()).add(binding.direction)

    conflicting_topics = [
        topic
        for topic, directions in directions_by_topic.items()
        if {"pdu_to_ros", "ros_to_pdu"}.issubset(directions)
    ]
    if conflicting_topics:
        topics = ", ".join(sorted(conflicting_topics))
        raise ValueError(
            "Refusing bidirectional bindings on the same ROS topic. "
            f"Use separate topics or omit topic for safe defaults: {topics}"
        )


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _resolve_path(base_dir: Path, raw_path: str) -> Path:
    path = Path(raw_path)
    return path if path.is_absolute() else (base_dir / path).resolve()
