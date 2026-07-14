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

    bindings = [_parse_binding(entry, pdu_definition) for entry in raw["bindings"]]
    return BindingRootConfig(
        endpoint_config=endpoint_config,
        pdu_def_path=pdu_def_path,
        bindings=bindings,
    )


def _parse_binding(entry: dict, pdu_definition: PduDefinition) -> BindingConfig:
    direction = entry.get("direction", "bidirectional")
    if direction not in {"pdu_to_ros", "ros_to_pdu", "bidirectional"}:
        raise ValueError(f"Unsupported binding direction: {direction}")

    pdu_key_entry = entry["pdu_key"]
    robot_name = pdu_key_entry["robot_name"]
    pdu_name = pdu_key_entry["pdu_name"]
    pdu = pdu_definition.get(robot_name, pdu_name)

    return BindingConfig(
        direction=direction,
        pdu_key=PduKeyConfig(robot_name=robot_name, pdu_name=pdu_name),
        topic=entry["topic"],
        channel_id=pdu.channel_id,
        pdu_size=pdu.pdu_size,
        pdu_type=pdu.type,
    )


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _resolve_path(base_dir: Path, raw_path: str) -> Path:
    path = Path(raw_path)
    return path if path.is_absolute() else (base_dir / path).resolve()
