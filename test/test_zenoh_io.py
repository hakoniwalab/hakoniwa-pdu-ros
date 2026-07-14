from __future__ import annotations

import json
from pathlib import Path

from hakoniwa_pdu_ros.config_loader import load_config
from hakoniwa_pdu_ros.zenoh_io import (
    ZenohIoValidationError,
    build_zenoh_io,
    validate_zenoh_io_for_config,
    write_zenoh_io,
)


def test_build_zenoh_io_from_bidirectional_bindings() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    config = load_config(
        repo_root / "examples" / "zenoh" / "config" / "zenoh_bidirectional_binding.json"
    )

    assert build_zenoh_io(config.bindings) == {
        "robots": [
            {
                "name": "demo",
                "pdu": [
                    {"name": "command", "notify_on_recv": True},
                    {"name": "debuginfo", "notify_on_recv": True},
                ],
            }
        ]
    }


def test_validate_zenoh_io_accepts_matching_comm() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    validate_zenoh_io_for_config(
        repo_root / "examples" / "zenoh" / "config" / "zenoh_bidirectional_binding.json"
    )


def test_write_zenoh_io_updates_only_io_section(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parent.parent
    binding_config = repo_root / "examples" / "zenoh" / "config" / "zenoh_bidirectional_binding.json"
    comm_path = tmp_path / "comm.json"
    comm_path.write_text(
        json.dumps(
            {
                "protocol": "zenoh",
                "name": "demo_comm",
                "direction": "inout",
                "zenoh": {
                    "config_path": "zenoh/client.json5",
                    "key_prefix": "hakoniwa",
                    "io": {"robots": []},
                },
            }
        ),
        encoding="utf-8",
    )

    write_zenoh_io(binding_config, comm_path)

    updated = json.loads(comm_path.read_text(encoding="utf-8"))
    assert updated["name"] == "demo_comm"
    assert updated["zenoh"]["key_prefix"] == "hakoniwa"
    assert updated["zenoh"]["io"]["robots"][0]["name"] == "demo"
    assert updated["zenoh"]["io"]["robots"][0]["pdu"] == [
        {"name": "command", "notify_on_recv": True},
        {"name": "debuginfo", "notify_on_recv": True},
    ]


def test_validate_zenoh_io_reports_generation_command(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parent.parent
    src_config = repo_root / "examples" / "zenoh" / "config"
    comm_path = tmp_path / "comm.json"
    binding_path = tmp_path / "binding.json"

    comm_path.write_text(
        json.dumps(
            {
                "protocol": "zenoh",
                "name": "demo_comm",
                "direction": "inout",
                "zenoh": {
                    "config_path": "zenoh/client.json5",
                    "key_prefix": "hakoniwa",
                    "io": {"robots": []},
                },
            }
        ),
        encoding="utf-8",
    )
    binding_path.write_text(
        json.dumps(
            {
                "endpoint_config": str(src_config / "endpoint_ros_bridge.json"),
                "bindings": [
                    {"pdu_key": {"robot_name": "demo", "pdu_name": "command"}},
                ],
            }
        ),
        encoding="utf-8",
    )

    endpoint_path = src_config / "endpoint_ros_bridge.json"
    endpoint = json.loads(endpoint_path.read_text(encoding="utf-8"))
    endpoint["comm"] = str(comm_path)
    endpoint["pdu_def_path"] = str(src_config / "pdudef.json")
    endpoint["cache"] = str(src_config / "cache" / "buffer.json")
    endpoint_copy = tmp_path / "endpoint.json"
    endpoint_copy.write_text(json.dumps(endpoint), encoding="utf-8")

    binding = json.loads(binding_path.read_text(encoding="utf-8"))
    binding["endpoint_config"] = str(endpoint_copy)
    binding_path.write_text(json.dumps(binding), encoding="utf-8")

    try:
        validate_zenoh_io_for_config(binding_path)
    except ZenohIoValidationError as err:
        message = str(err)
        assert "gen_zenoh_io" in message
        assert "--write" in message
        assert str(comm_path) in message
    else:
        assert False, "expected ZenohIoValidationError"
