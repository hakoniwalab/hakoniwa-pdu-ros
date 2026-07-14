from __future__ import annotations

import json
from pathlib import Path

from hakoniwa_pdu_ros.config_loader import BindingConfig, BindingRootConfig, load_config


class ZenohIoValidationError(ValueError):
    pass


def build_zenoh_io(bindings: list[BindingConfig]) -> dict:
    robots: dict[str, dict[str, bool]] = {}
    for binding in bindings:
        robot_pdus = robots.setdefault(binding.pdu_key.robot_name, {})
        current = robot_pdus.get(binding.pdu_key.pdu_name, False)
        robot_pdus[binding.pdu_key.pdu_name] = current or binding.direction == "pdu_to_ros"

    return {
        "robots": [
            {
                "name": robot_name,
                "pdu": [
                    {"name": pdu_name, "notify_on_recv": notify_on_recv}
                    for pdu_name, notify_on_recv in sorted(pdus.items())
                ],
            }
            for robot_name, pdus in sorted(robots.items())
        ]
    }


def validate_zenoh_io_for_config(config_path: str | Path) -> None:
    root_config = load_config(config_path)
    endpoint_config = _load_json(root_config.endpoint_config)
    comm_path = _resolve_path(root_config.endpoint_config.parent, endpoint_config["comm"])
    comm = _load_json(comm_path)
    if comm.get("protocol") != "zenoh":
        return

    expected = build_zenoh_io(root_config.bindings)
    actual = comm.get("zenoh", {}).get("io")
    if _canonical_io(actual) == _canonical_io(expected):
        return

    raise ZenohIoValidationError(_format_validation_error(config_path, comm_path, expected, actual))


def write_zenoh_io(binding_config_path: str | Path, comm_path: str | Path) -> None:
    root_config = load_config(binding_config_path)
    comm_path = Path(comm_path).expanduser().resolve()
    comm = _load_json(comm_path)
    if comm.get("protocol") != "zenoh":
        raise ValueError(f"comm config is not zenoh protocol: {comm_path}")
    comm.setdefault("zenoh", {})["io"] = build_zenoh_io(root_config.bindings)
    _write_json(comm_path, comm)


def print_zenoh_io(binding_config_path: str | Path) -> None:
    root_config = load_config(binding_config_path)
    print(json.dumps(build_zenoh_io(root_config.bindings), indent=2))


def _canonical_io(io: object) -> object:
    if not isinstance(io, dict):
        return None
    robots = []
    for robot in io.get("robots", []):
        if not isinstance(robot, dict):
            continue
        pdus = []
        for pdu in robot.get("pdu", []):
            if not isinstance(pdu, dict):
                continue
            pdus.append(
                {
                    "name": pdu.get("name"),
                    "notify_on_recv": bool(pdu.get("notify_on_recv", False)),
                }
            )
        robots.append({"name": robot.get("name"), "pdu": sorted(pdus, key=lambda item: item["name"])})
    return {"robots": sorted(robots, key=lambda item: item["name"])}


def _format_validation_error(
    config_path: str | Path,
    comm_path: Path,
    expected: dict,
    actual: object,
) -> str:
    return (
        "Zenoh io config does not match bridge bindings.\n\n"
        f"Binding config: {Path(config_path).expanduser()}\n"
        f"Comm config: {comm_path}\n\n"
        "Expected zenoh.io:\n"
        f"{json.dumps(expected, indent=2)}\n\n"
        "Actual zenoh.io:\n"
        f"{json.dumps(actual, indent=2)}\n\n"
        "Regenerate it with:\n\n"
        "  python3 -m hakoniwa_pdu_ros.gen_zenoh_io "
        f"{Path(config_path).expanduser()} --comm {comm_path} --write"
    )


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _write_json(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def _resolve_path(base_dir: Path, raw_path: str) -> Path:
    path = Path(raw_path)
    return path.expanduser().resolve() if path.is_absolute() else (base_dir / path).resolve()
