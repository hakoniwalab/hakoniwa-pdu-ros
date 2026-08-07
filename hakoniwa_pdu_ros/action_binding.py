from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ActionRuntimeConfig:
    node_id: str
    client_name: str
    action_config: Path
    endpoint_config: Path
    delta_time_usec: int = 1000
    time_source_type: str = "real"


@dataclass(frozen=True)
class ActionBinding:
    ros_name: str
    ros_type: str
    hakoniwa_action: str
    pdu_action_type: str
    goal_response_timeout_msec: int = 5000


@dataclass(frozen=True)
class ActionBindingConfig:
    runtime: ActionRuntimeConfig
    actions: tuple[ActionBinding, ...]


def load_action_binding(path: str | Path) -> ActionBindingConfig:
    config_path = Path(path).resolve()
    data = json.loads(config_path.read_text(encoding="utf-8"))
    if data.get("version") != 1:
        raise ValueError("Action Binding version must be 1")

    runtime_data = _mapping(data, "runtime")
    runtime = ActionRuntimeConfig(
        node_id=_required_string(runtime_data, "node_id"),
        client_name=_required_string(runtime_data, "client_name"),
        action_config=_resolve_path(config_path, _required_string(runtime_data, "action_config")),
        endpoint_config=_resolve_path(config_path, _required_string(runtime_data, "endpoint_config")),
        delta_time_usec=_positive_int(runtime_data, "delta_time_usec", 1000),
        time_source_type=runtime_data.get("time_source_type", "real"),
    )
    if runtime.time_source_type not in {"real", "simulation"}:
        raise ValueError("runtime.time_source_type must be real or simulation")

    raw_actions = data.get("actions")
    if not isinstance(raw_actions, list) or not raw_actions:
        raise ValueError("actions must be a non-empty list")

    actions = []
    ros_names: set[str] = set()
    hako_names: set[str] = set()
    for index, item in enumerate(raw_actions):
        if not isinstance(item, dict):
            raise ValueError(f"actions[{index}] must be an object")
        binding = ActionBinding(
            ros_name=_required_string(item, "ros_name"),
            ros_type=_required_string(item, "ros_type"),
            hakoniwa_action=_required_string(item, "hakoniwa_action"),
            pdu_action_type=_required_string(item, "pdu_action_type"),
            goal_response_timeout_msec=_positive_int(
                item, "goal_response_timeout_msec", 5000
            ),
        )
        if binding.ros_name in ros_names:
            raise ValueError(f"duplicate ROS Action name: {binding.ros_name}")
        if binding.hakoniwa_action in hako_names:
            raise ValueError(
                f"duplicate Hakoniwa Action binding: {binding.hakoniwa_action}"
            )
        ros_names.add(binding.ros_name)
        hako_names.add(binding.hakoniwa_action)
        actions.append(binding)

    return ActionBindingConfig(runtime=runtime, actions=tuple(actions))


def _mapping(data: dict, name: str) -> dict:
    value = data.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _required_string(data: dict, name: str) -> str:
    value = data.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _positive_int(data: dict, name: str, default: int) -> int:
    value = data.get(name, default)
    if not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _resolve_path(config_path: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = config_path.parent / path
    return path.resolve()
