from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class EndpointRef:
    node_id: str
    endpoint_id: str


@dataclass(frozen=True)
class RpcBindingConfig:
    client_endpoint: EndpointRef
    endpoint_config: Path
    delta_time_usec: int = 1000
    time_source_type: str = "real"


@dataclass(frozen=True)
class HeapConfig:
    request_bytes: int = 0
    response_bytes: int = 0


@dataclass(frozen=True)
class ServiceBinding:
    ros_name: str
    ros_type: str
    hakoniwa_service: str
    server_endpoints: tuple[EndpointRef, ...]
    max_clients: int
    timeout_msec: int
    heap: HeapConfig
    pdu_service_type: str | None = None


@dataclass(frozen=True)
class ServiceBindingConfig:
    source_path: Path
    version: int
    kind: str
    rpc: RpcBindingConfig
    bindings: tuple[ServiceBinding, ...]


def load_service_binding(path: str | Path) -> ServiceBindingConfig:
    source_path = Path(path).expanduser().resolve()
    raw = _load_json(source_path)
    _require_object(raw, "root")
    _reject_unknown(raw, {"$schema", "version", "kind", "rpc", "bindings"}, "root")

    version = raw.get("version")
    if version != 1:
        raise ValueError("Service Binding version must be 1")

    kind = raw.get("kind")
    if kind != "ros_service_server":
        raise ValueError(
            "Service Binding kind must be 'ros_service_server'; "
            "the ROS Service Client direction is not implemented"
        )

    rpc = _parse_rpc(raw.get("rpc"), source_path.parent)
    raw_bindings = raw.get("bindings")
    if not isinstance(raw_bindings, list) or not raw_bindings:
        raise ValueError("Service Binding bindings must be a non-empty array")
    bindings = tuple(
        _parse_binding(entry, index) for index, entry in enumerate(raw_bindings)
    )
    _validate_unique_bindings(bindings)
    _validate_endpoint_refs(rpc, bindings)

    return ServiceBindingConfig(
        source_path=source_path,
        version=version,
        kind=kind,
        rpc=rpc,
        bindings=bindings,
    )


def service_key(service_name: str) -> str:
    name = service_name.rsplit("/", 1)[-1]
    snake = re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()
    normalized = re.sub(r"[^a-z0-9_]+", "_", snake).strip("_")
    if not normalized:
        raise ValueError(f"Cannot derive service key from {service_name!r}")
    return normalized


def _parse_rpc(raw: object, base_dir: Path) -> RpcBindingConfig:
    _require_object(raw, "rpc")
    _reject_unknown(
        raw,
        {"client_endpoint", "endpoint_config", "delta_time_usec", "time_source_type"},
        "rpc",
    )
    client_endpoint = _parse_endpoint_ref(raw.get("client_endpoint"), "rpc.client_endpoint")
    endpoint_config_raw = _require_string(raw.get("endpoint_config"), "rpc.endpoint_config")
    endpoint_config = Path(endpoint_config_raw).expanduser()
    if not endpoint_config.is_absolute():
        endpoint_config = (base_dir / endpoint_config).resolve()
    if not endpoint_config.is_file():
        raise ValueError(f"RPC endpoint config does not exist: {endpoint_config}")

    delta_time_usec = _positive_int(raw.get("delta_time_usec", 1000), "rpc.delta_time_usec")
    time_source_type = _require_string(
        raw.get("time_source_type", "real"), "rpc.time_source_type"
    )
    return RpcBindingConfig(
        client_endpoint=client_endpoint,
        endpoint_config=endpoint_config,
        delta_time_usec=delta_time_usec,
        time_source_type=time_source_type,
    )


def _parse_binding(raw: object, index: int) -> ServiceBinding:
    path = f"bindings[{index}]"
    _require_object(raw, path)
    _reject_unknown(
        raw,
        {
            "ros_name",
            "ros_type",
            "hakoniwa_service",
            "server_endpoints",
            "max_clients",
            "timeout_msec",
            "heap",
            "pdu_service_type",
        },
        path,
    )
    ros_name = _require_string(raw.get("ros_name"), f"{path}.ros_name")
    if not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_/]*", ros_name):
        raise ValueError(f"{path}.ros_name must be an absolute ROS service name")

    ros_type = _require_string(raw.get("ros_type"), f"{path}.ros_type")
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*/srv/[A-Za-z][A-Za-z0-9_]*", ros_type):
        raise ValueError(f"{path}.ros_type must use package/srv/Type form")

    endpoints_raw = raw.get("server_endpoints")
    if not isinstance(endpoints_raw, list) or not endpoints_raw:
        raise ValueError(f"{path}.server_endpoints must be a non-empty array")
    server_endpoints = tuple(
        _parse_endpoint_ref(item, f"{path}.server_endpoints[{endpoint_index}]")
        for endpoint_index, item in enumerate(endpoints_raw)
    )

    heap = _parse_heap(raw.get("heap"), path)
    pdu_service_type = raw.get("pdu_service_type")
    if pdu_service_type is not None:
        pdu_service_type = _require_string(
            pdu_service_type, f"{path}.pdu_service_type"
        )

    return ServiceBinding(
        ros_name=ros_name,
        ros_type=ros_type,
        hakoniwa_service=_require_string(
            raw.get("hakoniwa_service"), f"{path}.hakoniwa_service"
        ),
        server_endpoints=server_endpoints,
        max_clients=_positive_int(raw.get("max_clients"), f"{path}.max_clients"),
        timeout_msec=_positive_int(raw.get("timeout_msec"), f"{path}.timeout_msec"),
        heap=heap,
        pdu_service_type=pdu_service_type,
    )


def _parse_heap(raw: object, binding_path: str) -> HeapConfig:
    if raw is None:
        return HeapConfig()
    path = f"{binding_path}.heap"
    _require_object(raw, path)
    _reject_unknown(raw, {"request_bytes", "response_bytes"}, path)
    return HeapConfig(
        request_bytes=_nonnegative_int(raw.get("request_bytes", 0), f"{path}.request_bytes"),
        response_bytes=_nonnegative_int(
            raw.get("response_bytes", 0), f"{path}.response_bytes"
        ),
    )


def _parse_endpoint_ref(raw: object, path: str) -> EndpointRef:
    _require_object(raw, path)
    _reject_unknown(raw, {"node_id", "endpoint_id"}, path)
    return EndpointRef(
        node_id=_require_string(raw.get("node_id"), f"{path}.node_id"),
        endpoint_id=_require_string(raw.get("endpoint_id"), f"{path}.endpoint_id"),
    )


def _validate_unique_bindings(bindings: tuple[ServiceBinding, ...]) -> None:
    _reject_duplicates((binding.ros_name for binding in bindings), "ROS service name")
    _reject_duplicates(
        (binding.hakoniwa_service for binding in bindings), "Hakoniwa service name"
    )
    _reject_duplicates(
        (service_key(binding.hakoniwa_service) for binding in bindings), "service key"
    )


def _validate_endpoint_refs(
    rpc: RpcBindingConfig, bindings: tuple[ServiceBinding, ...]
) -> None:
    raw = _load_json(rpc.endpoint_config)
    if not isinstance(raw, list):
        raise ValueError("RPC endpoint config must be an array")
    available: set[tuple[str, str]] = set()
    for node in raw:
        if not isinstance(node, dict):
            continue
        node_id = node.get("nodeId")
        endpoints = node.get("endpoints")
        if not isinstance(node_id, str) or not isinstance(endpoints, list):
            continue
        for endpoint in endpoints:
            if isinstance(endpoint, dict) and isinstance(endpoint.get("id"), str):
                available.add((node_id, endpoint["id"]))

    refs = [rpc.client_endpoint]
    refs.extend(endpoint for binding in bindings for endpoint in binding.server_endpoints)
    for endpoint in refs:
        key = (endpoint.node_id, endpoint.endpoint_id)
        if key not in available:
            raise ValueError(
                "Endpoint reference is not present in rpc.endpoint_config: "
                f"{endpoint.node_id}/{endpoint.endpoint_id}"
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


def _nonnegative_int(value: object, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{path} must be a non-negative integer")
    return value
