from __future__ import annotations

import importlib
import importlib.util
import json
import os
import pkgutil
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from hakoniwa_pdu_ros.action_binding import (
    ActionBinding,
    ActionBindingConfig,
    load_action_binding,
)


@dataclass(frozen=True)
class ResolvedAction:
    binding: ActionBinding
    pdu_action_type: str


@dataclass(frozen=True)
class GeneratedActionConfigs:
    output_dir: Path
    manifest: Path
    generated_files: tuple[Path, ...]
    actions: tuple[ResolvedAction, ...]


def generate_action_configs(
    config_path: str | Path,
    *,
    output_dir: str | Path | None = None,
    ros_interface_resolver: Callable[[str], None] | None = None,
    pdu_type_resolver: Callable[[str, str | None], str] | None = None,
    rpc_generator: Callable[[Path, Path], list[Path]] | None = None,
) -> GeneratedActionConfigs:
    """Resolve a ROS Action Binding and delegate native config generation.

    This module owns only ROS/PDU type resolution and the semantic Binding
    mapping. Channel, packet, endpoint and default heap rules remain owned by
    ``hakoniwa-pdu-rpc``.
    """

    config = load_action_binding(config_path)
    resolve_ros = ros_interface_resolver or resolve_installed_ros_action
    resolve_pdu = pdu_type_resolver or resolve_installed_pdu_action_type
    generate_rpc = rpc_generator or _load_rpc_generator()

    resolved = []
    for binding in config.bindings:
        resolve_ros(binding.ros_type)
        resolved.append(
            ResolvedAction(
                binding=binding,
                pdu_action_type=resolve_pdu(
                    binding.ros_type, binding.pdu_action_type
                ),
            )
        )

    target_dir = _resolve_output_dir(config, output_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = target_dir / "hakoniwa-action.json"
    _write_json_atomic(manifest_path, _build_rpc_manifest(config, resolved))
    generated = tuple(generate_rpc(manifest_path, target_dir))
    return GeneratedActionConfigs(
        output_dir=target_dir,
        manifest=manifest_path,
        generated_files=generated,
        actions=tuple(resolved),
    )


def resolve_installed_ros_action(ros_type: str) -> None:
    package_name, _, action_name = ros_type.split("/", 2)
    try:
        module = importlib.import_module(f"{package_name}.action")
        getattr(module, action_name)
    except (ModuleNotFoundError, AttributeError) as error:
        raise ValueError(f"ROS action interface is not installed: {ros_type}") from error

    try:
        from ament_index_python.packages import get_package_share_directory
    except ModuleNotFoundError as error:
        raise ValueError(
            "ament_index_python is required to resolve installed .action definitions"
        ) from error

    action_path = (
        Path(get_package_share_directory(package_name))
        / "action"
        / f"{action_name}.action"
    )
    if not action_path.is_file():
        raise ValueError(f"Installed ROS .action definition was not found: {action_path}")


def resolve_installed_pdu_action_type(
    ros_type: str, explicit_type: str | None
) -> str:
    action_name = ros_type.rsplit("/", 1)[-1]
    if explicit_type is not None:
        package_name, resolved_name = _split_pdu_type(explicit_type)
        if resolved_name != action_name:
            raise ValueError(
                "PDU action type basename must match ROS action type: "
                f"{explicit_type} != {ros_type}"
            )
        _validate_pdu_modules(package_name, resolved_name)
        return explicit_type

    try:
        pdu_msgs = importlib.import_module("hakoniwa_pdu.pdu_msgs")
    except ModuleNotFoundError as error:
        raise ValueError(
            "hakoniwa-pdu is required to resolve generated action types"
        ) from error

    candidates = []
    for module in pkgutil.iter_modules(pdu_msgs.__path__):
        if not module.ispkg:
            continue
        try:
            _validate_pdu_modules(module.name, action_name)
        except ValueError:
            continue
        candidates.append(f"{module.name}/{action_name}")

    if not candidates:
        raise ValueError(f"Generated PDU action type was not found for {ros_type}")
    if len(candidates) > 1:
        joined = ", ".join(sorted(candidates))
        raise ValueError(
            f"Generated PDU action type is ambiguous for {ros_type}: {joined}. "
            "Set pdu_action_type explicitly."
        )
    return candidates[0]


def _build_rpc_manifest(
    config: ActionBindingConfig, actions: list[ResolvedAction]
) -> dict:
    transport = json.loads(
        config.action.transport_config.read_text(encoding="utf-8")
    )
    entries = []
    for resolved in actions:
        binding = resolved.binding
        entry = {
            "name": binding.hakoniwa_action,
            "type": resolved.pdu_action_type,
            "slotCount": binding.slot_count,
            "clientEndpoint": {"nodeId": binding.client_endpoint.node_id},
            "serverEndpoint": {"nodeId": binding.server_endpoint.node_id},
        }
        heap = {
            key: value
            for key, value in (
                ("requestSize", binding.heap.goal_bytes),
                ("responseSize", binding.heap.result_bytes),
                ("feedbackSize", binding.heap.feedback_bytes),
            )
            if value is not None
        }
        if heap:
            entry["bufferHeap"] = heap
        entries.append(entry)
    return {"version": 1, "actions": entries, "transport": transport}


def _validate_pdu_modules(package_name: str, action_name: str) -> None:
    required = tuple(
        f"{prefix}_{action_name}Action{suffix}"
        for suffix in ("Request", "Response", "Feedback")
        for prefix in ("pdu_pytype", "pdu_conv")
    )
    missing = []
    for module_name in required:
        qualified_name = f"hakoniwa_pdu.pdu_msgs.{package_name}.{module_name}"
        if importlib.util.find_spec(qualified_name) is None:
            missing.append(module_name)
    if missing:
        raise ValueError(
            "Generated PDU modules are incomplete for "
            f"{package_name}/{action_name}: {', '.join(missing)}"
        )


def _load_rpc_generator() -> Callable[[Path, Path], list[Path]]:
    try:
        from hakoniwa_pdu_rpc.action_config_generator import generate
    except ModuleNotFoundError as error:
        raise ValueError(
            "hakoniwa-pdu-rpc with the installed Action generator is required"
        ) from error
    return generate


def _split_pdu_type(type_name: str) -> tuple[str, str]:
    parts = type_name.split("/", 1)
    if len(parts) != 2 or not all(parts):
        raise ValueError(f"PDU type must use package/Type form: {type_name}")
    return parts[0], parts[1]


def _resolve_output_dir(
    config: ActionBindingConfig, value: str | Path | None
) -> Path:
    if value is not None:
        return Path(value).expanduser().resolve()
    return (
        Path.cwd() / "build" / "generated" / "action" / config.source_path.stem
    ).resolve()


def _write_json_atomic(path: Path, data: dict) -> None:
    serialized = json.dumps(data, indent=2, ensure_ascii=False) + "\n"
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
