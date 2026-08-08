from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ActionEndpointRef:
    node_id: str


@dataclass(frozen=True)
class ActionRuntimeConfig:
    transport_config: Path
    delta_time_usec: int = 1000
    time_source_type: str = "real"


@dataclass(frozen=True)
class ActionHeapConfig:
    goal_bytes: int | None = None
    result_bytes: int | None = None
    feedback_bytes: int | None = None


@dataclass(frozen=True)
class ActionBinding:
    ros_name: str
    ros_type: str
    hakoniwa_action: str
    client_endpoint: ActionEndpointRef
    server_endpoint: ActionEndpointRef
    slot_count: int
    goal_response_timeout_msec: int
    heap: ActionHeapConfig
    pdu_action_type: str | None = None


@dataclass(frozen=True)
class ActionBindingConfig:
    source_path: Path
    version: int
    action: ActionRuntimeConfig
    bindings: tuple[ActionBinding, ...]


def load_action_binding(path: str | Path) -> ActionBindingConfig:
    source_path = Path(path).expanduser().resolve()
    raw = _load_json(source_path)
    _require_object(raw, "root")
    _reject_unknown(raw, {"$schema", "version", "action", "bindings"}, "root")

    version = raw.get("version")
    if version != 1:
        raise ValueError("Action Binding version must be 1")

    action = _parse_action(raw.get("action"), source_path.parent)
    transport = _load_transport(action.transport_config)

    raw_bindings = raw.get("bindings")
    if not isinstance(raw_bindings, list) or not raw_bindings:
        raise ValueError("Action Binding bindings must be a non-empty array")
    bindings = tuple(
        _parse_binding(entry, index) for index, entry in enumerate(raw_bindings)
    )
    _validate_unique_bindings(bindings)
    _validate_endpoint_refs(bindings, transport)

    return ActionBindingConfig(
        source_path=source_path,
        version=version,
        action=action,
        bindings=bindings,
    )


def _parse_action(raw: object, base_dir: Path) -> ActionRuntimeConfig:
    _require_object(raw, "action")
    _reject_unknown(
        raw,
        {"transport_config", "delta_time_usec", "time_source_type"},
        "action",
    )
    transport_raw = _require_string(
        raw.get("transport_config"), "action.transport_config"
    )
    transport_config = Path(transport_raw).expanduser()
    if not transport_config.is_absolute():
        transport_config = (base_dir / transport_config).resolve()
    if not transport_config.is_file():
        raise ValueError(f"Action transport config does not exist: {transport_config}")

    return ActionRuntimeConfig(
        transport_config=transport_config,
        delta_time_usec=_positive_int(
            raw.get("delta_time_usec", 1000), "action.delta_time_usec"
        ),
        time_source_type=_require_string(
            raw.get("time_source_type", "real"), "action.time_source_type"
        ),
    )


def _parse_binding(raw: object, index: int) -> ActionBinding:
    path = f"bindings[{index}]"
    _require_object(raw, path)
    _reject_unknown(
        raw,
        {
            "ros_name",
            "ros_type",
            "hakoniwa_action",
            "client_endpoint",
            "server_endpoint",
            "slot_count",
            "goal_response_timeout_msec",
            "heap",
            "pdu_action_type",
        },
        path,
    )

    ros_name = _require_string(raw.get("ros_name"), f"{path}.ros_name")
    if not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_/]*", ros_name):
        raise ValueError(f"{path}.ros_name must be an absolute ROS action name")

    ros_type = _require_string(raw.get("ros_type"), f"{path}.ros_type")
    if not re.fullmatch(
        r"[A-Za-z][A-Za-z0-9_]*/action/[A-Za-z][A-Za-z0-9_]*", ros_type
    ):
        raise ValueError(f"{path}.ros_type must use package/action/Type form")

    pdu_action_type = raw.get("pdu_action_type")
    if pdu_action_type is not None:
        pdu_action_type = _require_string(
            pdu_action_type, f"{path}.pdu_action_type"
        )
        if not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_]*/[A-Za-z][A-Za-z0-9_]*", pdu_action_type
        ):
            raise ValueError(f"{path}.pdu_action_type must use package/Type form")
        if pdu_action_type.rsplit("/", 1)[-1] != ros_type.rsplit("/", 1)[-1]:
            raise ValueError(
                f"{path}.pdu_action_type basename must match ros_type"
            )

    return ActionBinding(
        ros_name=ros_name,
        ros_type=ros_type,
        hakoniwa_action=_require_string(
            raw.get("hakoniwa_action"), f"{path}.hakoniwa_action"
        ),
        client_endpoint=_parse_endpoint_ref(
            raw.get("client_endpoint"), f"{path}.client_endpoint"
        ),
        server_endpoint=_parse_endpoint_ref(
            raw.get("server_endpoint"), f"{path}.server_endpoint"
        ),
        slot_count=_positive_int(raw.get("slot_count"), f"{path}.slot_count"),
        goal_response_timeout_msec=_positive_int(
            raw.get("goal_response_timeout_msec"),
            f"{path}.goal_response_timeout_msec",
        ),
        heap=_parse_heap(raw.get("heap"), path),
        pdu_action_type=pdu_action_type,
    )


def _parse_endpoint_ref(raw: object, path: str) -> ActionEndpointRef:
    _require_object(raw, path)
    _reject_unknown(raw, {"node_id"}, path)
    return ActionEndpointRef(
        node_id=_require_string(raw.get("node_id"), f"{path}.node_id")
    )


def _parse_heap(raw: object, binding_path: str) -> ActionHeapConfig:
    if raw is None:
        return ActionHeapConfig()
    path = f"{binding_path}.heap"
    _require_object(raw, path)
    _reject_unknown(raw, {"goal_bytes", "result_bytes", "feedback_bytes"}, path)
    return ActionHeapConfig(
        goal_bytes=_optional_nonnegative_int(raw, "goal_bytes", path),
        result_bytes=_optional_nonnegative_int(raw, "result_bytes", path),
        feedback_bytes=_optional_nonnegative_int(raw, "feedback_bytes", path),
    )


def _load_transport(path: Path) -> dict:
    raw = _load_json(path)
    _require_object(raw, "transport")
    _reject_unknown(
        raw,
        {"protocol", "packetVersion", "queueDepth", "endpoints"},
        "transport",
    )
    if raw.get("protocol") != "tcp":
        raise ValueError("Action transport protocol must be 'tcp'")
    packet_version = raw.get("packetVersion", "v2")
    if packet_version not in {"v1", "v2"}:
        raise ValueError("Action transport packetVersion must be 'v1' or 'v2'")
    _positive_int(raw.get("queueDepth", 64), "transport.queueDepth")

    endpoints = raw.get("endpoints")
    _require_object(endpoints, "transport.endpoints")
    if len(endpoints) < 2:
        raise ValueError("transport.endpoints must contain at least two endpoints")
    for node_id, endpoint in endpoints.items():
        if not isinstance(node_id, str) or not node_id:
            raise ValueError("transport endpoint node IDs must be non-empty strings")
        _validate_transport_endpoint(endpoint, f"transport.endpoints.{node_id}")
    return raw


def _validate_transport_endpoint(raw: object, path: str) -> None:
    _require_object(raw, path)
    _reject_unknown(raw, {"role", "local", "remote", "options"}, path)
    role = raw.get("role")
    if role not in {"server", "client"}:
        raise ValueError(f"{path}.role must be 'server' or 'client'")
    address_key = "local" if role == "server" else "remote"
    forbidden_key = "remote" if role == "server" else "local"
    if forbidden_key in raw:
        raise ValueError(f"{path}.{forbidden_key} is invalid for role {role!r}")
    address = raw.get(address_key)
    _require_object(address, f"{path}.{address_key}")
    _reject_unknown(address, {"address", "port"}, f"{path}.{address_key}")
    _require_string(address.get("address"), f"{path}.{address_key}.address")
    port = address.get("port")
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError(f"{path}.{address_key}.port must be an integer in 1..65535")
    if "options" in raw:
        _require_object(raw["options"], f"{path}.options")


def _validate_unique_bindings(bindings: tuple[ActionBinding, ...]) -> None:
    _reject_duplicates((binding.ros_name for binding in bindings), "ROS action name")
    _reject_duplicates(
        (binding.hakoniwa_action for binding in bindings), "Hakoniwa action name"
    )


def _validate_endpoint_refs(
    bindings: tuple[ActionBinding, ...], transport: dict
) -> None:
    endpoints = transport["endpoints"]
    for index, binding in enumerate(bindings):
        client = binding.client_endpoint.node_id
        server = binding.server_endpoint.node_id
        if client == server:
            raise ValueError(
                f"bindings[{index}] client_endpoint and server_endpoint must differ"
            )
        for role, node_id in (("client", client), ("server", server)):
            if node_id not in endpoints:
                raise ValueError(
                    f"bindings[{index}].{role}_endpoint node_id is not present "
                    f"in action.transport_config: {node_id}"
                )
        if endpoints[client]["role"] == endpoints[server]["role"]:
            raise ValueError(
                f"bindings[{index}] endpoints must use complementary TCP roles"
            )


def _reject_duplicates(values, label: str) -> None:
    seen: set[str] = set()
    for value in values:
        if value in seen:
            raise ValueError(f"Duplicate {label}: {value}")
        seen.add(value)


def _load_json(path: Path) -> object:
    try:
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"Failed to load JSON file {path}: {error}") from error


def _require_object(value: object, path: str) -> None:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be an object")


def _reject_unknown(value: dict, allowed: set[str], path: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ValueError(f"Unsupported {path} fields: {', '.join(unknown)}")


def _require_string(value: object, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{path} must be a non-empty string")
    return value


def _positive_int(value: object, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{path} must be a positive integer")
    return value


def _optional_nonnegative_int(raw: dict, key: str, path: str) -> int | None:
    if key not in raw:
        return None
    value = raw[key]
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{path}.{key} must be a non-negative integer")
    return value
