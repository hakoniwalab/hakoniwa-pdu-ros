from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class EndpointRef:
    node_id: str


@dataclass(frozen=True)
class ServiceRuntimeConfig:
    transport_config: Path
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
    client_endpoint: EndpointRef
    server_endpoint: EndpointRef
    max_clients: int
    timeout_msec: int
    heap: HeapConfig
    pdu_service_type: str | None = None


@dataclass(frozen=True)
class ServiceBindingConfig:
    source_path: Path
    version: int
    service: ServiceRuntimeConfig
    bindings: tuple[ServiceBinding, ...]


def load_service_binding(path: str | Path) -> ServiceBindingConfig:
    source_path = Path(path).expanduser().resolve()
    raw = _load_json(source_path)
    _require_object(raw, "root")
    _reject_unknown(raw, {"$schema", "version", "service", "bindings"}, "root")
    if raw.get("version") != 1:
        raise ValueError("Service Binding version must be 1")

    service = _parse_service(raw.get("service"), source_path.parent)
    transport = _load_transport(service.transport_config)
    raw_bindings = raw.get("bindings")
    if not isinstance(raw_bindings, list) or not raw_bindings:
        raise ValueError("Service Binding bindings must be a non-empty array")
    bindings = tuple(
        _parse_binding(entry, index) for index, entry in enumerate(raw_bindings)
    )
    _validate_unique_bindings(bindings)
    _validate_endpoint_refs(bindings, transport)
    return ServiceBindingConfig(source_path, 1, service, bindings)


def service_key(service_name: str) -> str:
    name = service_name.rsplit("/", 1)[-1]
    snake = re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()
    normalized = re.sub(r"[^a-z0-9_]+", "_", snake).strip("_")
    if not normalized:
        raise ValueError(f"Cannot derive service key from {service_name!r}")
    return normalized


def _parse_service(raw: object, base_dir: Path) -> ServiceRuntimeConfig:
    _require_object(raw, "service")
    _reject_unknown(
        raw,
        {"transport_config", "delta_time_usec", "time_source_type"},
        "service",
    )
    transport = Path(
        _require_string(raw.get("transport_config"), "service.transport_config")
    ).expanduser()
    if not transport.is_absolute():
        transport = (base_dir / transport).resolve()
    if not transport.is_file():
        raise ValueError(f"Service transport config does not exist: {transport}")
    return ServiceRuntimeConfig(
        transport_config=transport,
        delta_time_usec=_positive_int(
            raw.get("delta_time_usec", 1000), "service.delta_time_usec"
        ),
        time_source_type=_require_string(
            raw.get("time_source_type", "real"), "service.time_source_type"
        ),
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
            "client_endpoint",
            "server_endpoint",
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
    if not re.fullmatch(
        r"[A-Za-z][A-Za-z0-9_]*/srv/[A-Za-z][A-Za-z0-9_]*", ros_type
    ):
        raise ValueError(f"{path}.ros_type must use package/srv/Type form")

    pdu_service_type = raw.get("pdu_service_type")
    if pdu_service_type is not None:
        pdu_service_type = _require_string(
            pdu_service_type, f"{path}.pdu_service_type"
        )
        if not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_]*/[A-Za-z][A-Za-z0-9_]*",
            pdu_service_type,
        ):
            raise ValueError(f"{path}.pdu_service_type must use package/Type form")
        if pdu_service_type.rsplit("/", 1)[-1] != ros_type.rsplit("/", 1)[-1]:
            raise ValueError(
                f"{path}.pdu_service_type basename must match ros_type"
            )

    return ServiceBinding(
        ros_name=ros_name,
        ros_type=ros_type,
        hakoniwa_service=_require_string(
            raw.get("hakoniwa_service"), f"{path}.hakoniwa_service"
        ),
        client_endpoint=_parse_endpoint_ref(
            raw.get("client_endpoint"), f"{path}.client_endpoint"
        ),
        server_endpoint=_parse_endpoint_ref(
            raw.get("server_endpoint"), f"{path}.server_endpoint"
        ),
        max_clients=_positive_int(raw.get("max_clients"), f"{path}.max_clients"),
        timeout_msec=_positive_int(
            raw.get("timeout_msec"), f"{path}.timeout_msec"
        ),
        heap=_parse_heap(raw.get("heap"), path),
        pdu_service_type=pdu_service_type,
    )


def _parse_heap(raw: object, binding_path: str) -> HeapConfig:
    if raw is None:
        return HeapConfig()
    path = f"{binding_path}.heap"
    _require_object(raw, path)
    _reject_unknown(raw, {"request_bytes", "response_bytes"}, path)
    return HeapConfig(
        request_bytes=_nonnegative_int(
            raw.get("request_bytes", 0), f"{path}.request_bytes"
        ),
        response_bytes=_nonnegative_int(
            raw.get("response_bytes", 0), f"{path}.response_bytes"
        ),
    )


def _parse_endpoint_ref(raw: object, path: str) -> EndpointRef:
    _require_object(raw, path)
    _reject_unknown(raw, {"node_id"}, path)
    return EndpointRef(_require_string(raw.get("node_id"), f"{path}.node_id"))


def _load_transport(path: Path) -> dict:
    raw = _load_json(path)
    _require_object(raw, "transport")
    _reject_unknown(
        raw,
        {"protocol", "packetVersion", "queueDepth", "endpoints"},
        "transport",
    )
    if raw.get("protocol") != "tcp":
        raise ValueError("Service transport protocol must be 'tcp'")
    if raw.get("packetVersion", "v2") not in {"v1", "v2"}:
        raise ValueError("Service transport packetVersion must be 'v1' or 'v2'")
    _positive_int(raw.get("queueDepth", 64), "transport.queueDepth")
    endpoints = raw.get("endpoints")
    _require_object(endpoints, "transport.endpoints")
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


def _validate_unique_bindings(bindings: tuple[ServiceBinding, ...]) -> None:
    _reject_duplicates((binding.ros_name for binding in bindings), "ROS service name")
    _reject_duplicates(
        (binding.hakoniwa_service for binding in bindings), "Hakoniwa service name"
    )
    _reject_duplicates(
        (service_key(binding.hakoniwa_service) for binding in bindings),
        "service key",
    )


def _validate_endpoint_refs(
    bindings: tuple[ServiceBinding, ...], transport: dict
) -> None:
    endpoints = transport["endpoints"]
    referenced = {
        endpoint.node_id
        for binding in bindings
        for endpoint in (binding.client_endpoint, binding.server_endpoint)
    }
    missing = sorted(referenced - set(endpoints))
    extra = sorted(set(endpoints) - referenced)
    if missing:
        raise ValueError(
            "transport.endpoints is missing referenced node IDs: "
            + ", ".join(missing)
        )
    if extra:
        raise ValueError(
            "transport.endpoints has unreferenced node IDs: " + ", ".join(extra)
        )
    for index, binding in enumerate(bindings):
        if binding.client_endpoint.node_id == binding.server_endpoint.node_id:
            raise ValueError(
                f"bindings[{index}] client_endpoint and server_endpoint must differ"
            )
        if endpoints[binding.client_endpoint.node_id]["role"] != "client":
            raise ValueError(
                f"bindings[{index}].client_endpoint must reference a client transport"
            )
        if endpoints[binding.server_endpoint.node_id]["role"] != "server":
            raise ValueError(
                f"bindings[{index}].server_endpoint must reference a server transport"
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
