from __future__ import annotations

import importlib
import importlib.util
import json
import os
import pkgutil
import tempfile
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from hakoniwa_pdu_ros.service_binding import (
    EndpointRef,
    ServiceBinding,
    ServiceBindingConfig,
    load_service_binding,
    service_key,
)


PDU_METADATA_SIZE = 24


@dataclass(frozen=True)
class ResolvedService:
    binding: ServiceBinding
    pdu_service_type: str
    request_base_size: int
    response_base_size: int


@dataclass(frozen=True)
class GeneratedServiceConfigs:
    output_dir: Path
    server_config: Path
    client_config: Path
    services: tuple[ResolvedService, ...]


def generate_service_configs(
    config_path: str | Path,
    *,
    output_dir: str | Path | None = None,
    offset_dir: str | Path | None = None,
    ros_interface_resolver: Callable[[str], None] | None = None,
    pdu_type_resolver: Callable[[str, str | None], str] | None = None,
) -> GeneratedServiceConfigs:
    config = load_service_binding(config_path)
    offsets = _resolve_offset_dir(offset_dir)
    resolve_ros = ros_interface_resolver or resolve_installed_ros_service
    resolve_pdu = pdu_type_resolver or resolve_installed_pdu_service_type

    resolved: list[ResolvedService] = []
    for binding in config.bindings:
        resolve_ros(binding.ros_type)
        pdu_type = resolve_pdu(binding.ros_type, binding.pdu_service_type)
        resolved.append(
            ResolvedService(
                binding=binding,
                pdu_service_type=pdu_type,
                request_base_size=resolve_offset_size(
                    offsets, f"{pdu_type}RequestPacket"
                ),
                response_base_size=resolve_offset_size(
                    offsets, f"{pdu_type}ResponsePacket"
                ),
            )
        )

    target_dir = _resolve_output_dir(config, output_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    server_path = target_dir / "rpc-server-services.json"
    client_path = target_dir / "rpc-client-services.json"

    server_data = _build_role_config(config, resolved, role="server")
    client_data = _build_role_config(config, resolved, role="client")
    _write_json_atomic(server_path, server_data)
    _write_json_atomic(client_path, client_data)
    return GeneratedServiceConfigs(
        output_dir=target_dir,
        server_config=server_path,
        client_config=client_path,
        services=tuple(resolved),
    )


def resolve_installed_ros_service(ros_type: str) -> None:
    package_name, _, service_name = ros_type.split("/", 2)
    try:
        module = importlib.import_module(f"{package_name}.srv")
        getattr(module, service_name)
    except (ModuleNotFoundError, AttributeError) as error:
        raise ValueError(f"ROS service interface is not installed: {ros_type}") from error

    try:
        from ament_index_python.packages import get_package_share_directory
    except ModuleNotFoundError as error:
        raise ValueError(
            "ament_index_python is required to resolve installed .srv definitions"
        ) from error

    share_dir = Path(get_package_share_directory(package_name))
    srv_path = share_dir / "srv" / f"{service_name}.srv"
    if not srv_path.is_file():
        raise ValueError(f"Installed ROS .srv definition was not found: {srv_path}")


def resolve_installed_pdu_service_type(
    ros_type: str, explicit_type: str | None
) -> str:
    service_name = ros_type.rsplit("/", 1)[-1]
    if explicit_type is not None:
        package_name, resolved_name = _split_pdu_type(explicit_type)
        if resolved_name != service_name:
            raise ValueError(
                "PDU service type basename must match ROS service type: "
                f"{explicit_type} != {ros_type}"
            )
        _validate_pdu_modules(package_name, resolved_name)
        return explicit_type

    try:
        pdu_msgs = importlib.import_module("hakoniwa_pdu.pdu_msgs")
    except ModuleNotFoundError as error:
        raise ValueError(
            "hakoniwa-pdu is required to resolve generated service types"
        ) from error

    candidates = []
    for module in pkgutil.iter_modules(pdu_msgs.__path__):
        if not module.ispkg:
            continue
        try:
            _validate_pdu_modules(module.name, service_name)
        except ValueError:
            continue
        candidates.append(f"{module.name}/{service_name}")

    if not candidates:
        raise ValueError(f"Generated PDU service type was not found for {ros_type}")
    if len(candidates) > 1:
        joined = ", ".join(sorted(candidates))
        raise ValueError(
            f"Generated PDU service type is ambiguous for {ros_type}: {joined}. "
            "Set pdu_service_type explicitly."
        )
    return candidates[0]


def resolve_offset_size(offset_dir: Path, packet_type: str) -> int:
    package_name, type_name = _split_pdu_type(packet_type)
    direct = offset_dir / package_name / f"{type_name}.offset"
    if direct.is_file():
        path = direct
    else:
        matches = sorted(offset_dir.rglob(f"{type_name}.offset"))
        package_matches = [match for match in matches if match.parent.name == package_name]
        if len(package_matches) != 1:
            raise ValueError(
                f"Expected one offset file for {packet_type} under {offset_dir}, "
                f"found {len(package_matches)}"
            )
        path = package_matches[0]

    lines = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not lines:
        raise ValueError(f"Offset file is empty: {path}")
    fields = lines[-1].split(":")
    if len(fields) < 6:
        raise ValueError(f"Invalid offset line in {path}: {lines[-1]}")
    try:
        member_offset = int(fields[4])
        member_size = 8 if fields[0] == "varray" else int(fields[5])
    except ValueError as error:
        raise ValueError(f"Invalid numeric offset data in {path}: {lines[-1]}") from error
    return _align8(member_offset + member_size + 8)


def _build_role_config(
    config: ServiceBindingConfig,
    services: list[ResolvedService],
    *,
    role: str,
) -> dict:
    entries = []
    for resolved in services:
        binding = resolved.binding
        clients = _build_clients(binding, config.rpc.client_endpoint)
        server_endpoints = (
            [_endpoint_json(endpoint) for endpoint in binding.server_endpoints]
            if role == "server"
            else []
        )
        entries.append(
            {
                "name": binding.hakoniwa_service,
                "type": resolved.pdu_service_type,
                "maxClients": binding.max_clients,
                "pduSize": {
                    "server": {
                        # Native PDU-RPC pairs the server-side heap field with
                        # the response packet (client base size).
                        "heapSize": binding.heap.response_bytes,
                        "baseSize": resolved.request_base_size,
                    },
                    "client": {
                        # Native PDU-RPC pairs the client-side heap field with
                        # the request packet (server base size).
                        "heapSize": binding.heap.request_bytes,
                        "baseSize": resolved.response_base_size,
                    },
                },
                "server_endpoints": server_endpoints,
                "clients": clients,
            }
        )
    return {"pduMetaDataSize": PDU_METADATA_SIZE, "services": entries}


def _build_clients(binding: ServiceBinding, endpoint: EndpointRef) -> list[dict]:
    key = service_key(binding.hakoniwa_service)
    return [
        {
            "name": f"hakoniwa_pdu_ros_{key}_{index}",
            "requestChannelId": 2 * index,
            "responseChannelId": 2 * index + 1,
            "client_endpoint": _endpoint_json(endpoint),
        }
        for index in range(binding.max_clients)
    ]


def _endpoint_json(endpoint: EndpointRef) -> dict:
    return {"nodeId": endpoint.node_id, "endpointId": endpoint.endpoint_id}


def _resolve_offset_dir(value: str | Path | None) -> Path:
    raw = value if value is not None else os.environ.get("HAKO_BINARY_PATH")
    if raw is None:
        raise ValueError("Specify --offset-dir or set HAKO_BINARY_PATH")
    path = Path(raw).expanduser().resolve()
    if not path.is_dir():
        raise ValueError(f"Offset directory does not exist: {path}")
    return path


def _resolve_output_dir(
    config: ServiceBindingConfig, value: str | Path | None
) -> Path:
    if value is not None:
        return Path(value).expanduser().resolve()
    base_dir = Path.cwd() / "build" / "generated" / "service"
    return (base_dir / config.source_path.stem).resolve()


def _validate_pdu_modules(package_name: str, service_name: str) -> None:
    required = (
        f"pdu_pytype_{service_name}RequestPacket",
        f"pdu_pytype_{service_name}ResponsePacket",
        f"pdu_conv_{service_name}RequestPacket",
        f"pdu_conv_{service_name}ResponsePacket",
    )
    missing = []
    for module_name in required:
        qualified_name = (
            f"hakoniwa_pdu.pdu_msgs.{package_name}.{module_name}"
        )
        if importlib.util.find_spec(qualified_name) is None:
            missing.append(module_name)
    if missing:
        raise ValueError(
            "Generated PDU modules are incomplete for "
            f"{package_name}/{service_name}: {', '.join(missing)}"
        )


def _split_pdu_type(type_name: str) -> tuple[str, str]:
    parts = type_name.split("/", 1)
    if len(parts) != 2 or not all(parts):
        raise ValueError(f"PDU type must use package/Type form: {type_name}")
    return parts[0], parts[1]


def _align8(value: int) -> int:
    return ((value + 7) // 8) * 8


def _write_json_atomic(path: Path, data: dict) -> None:
    serialized = json.dumps(deepcopy(data), indent=2, ensure_ascii=False) + "\n"
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            stream.write(serialized)
            temporary_path = Path(stream.name)
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
