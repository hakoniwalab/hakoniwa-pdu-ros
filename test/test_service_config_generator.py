from __future__ import annotations

import json
from pathlib import Path

import pytest

from hakoniwa_pdu_ros.service_binding import load_service_binding, service_key
from hakoniwa_pdu_ros.service_config_generator import (
    generate_service_configs,
    resolve_offset_size,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
BINDING = REPO_ROOT / "config" / "service" / "add_two_ints.json"
FIXTURES = REPO_ROOT / "test" / "fixtures"
OFFSETS = FIXTURES / "offset"


def test_load_add_two_ints_service_binding() -> None:
    config = load_service_binding(BINDING)

    assert config.version == 1
    assert config.kind == "ros_service_server"
    assert config.rpc.client_endpoint.node_id == "hakoniwa-pdu-ros-service"
    assert config.rpc.client_endpoint.endpoint_id == "client_ep_id"
    assert config.rpc.endpoint_config.name == "rpc-endpoints.json"
    assert len(config.bindings) == 1
    binding = config.bindings[0]
    assert binding.ros_type == "example_interfaces/srv/AddTwoInts"
    assert binding.max_clients == 4
    assert binding.heap.request_bytes == 0
    assert binding.heap.response_bytes == 0


def test_generate_add_two_ints_server_and_client_configs(tmp_path: Path) -> None:
    resolved_ros_types = []

    generated = generate_service_configs(
        BINDING,
        output_dir=tmp_path,
        offset_dir=OFFSETS,
        ros_interface_resolver=resolved_ros_types.append,
        pdu_type_resolver=lambda _ros_type, _override: "hako_srv_msgs/AddTwoInts",
    )

    assert resolved_ros_types == ["example_interfaces/srv/AddTwoInts"]
    assert _load(generated.server_config) == _load(
        FIXTURES / "add_two_ints-rpc-server-services.json"
    )
    assert _load(generated.client_config) == _load(
        FIXTURES / "add_two_ints-rpc-client-services.json"
    )


def test_generation_is_idempotent(tmp_path: Path) -> None:
    kwargs = {
        "output_dir": tmp_path,
        "offset_dir": OFFSETS,
        "ros_interface_resolver": lambda _ros_type: None,
        "pdu_type_resolver": lambda _ros_type, _override: "hako_srv_msgs/AddTwoInts",
    }
    first = generate_service_configs(BINDING, **kwargs)
    first_server = first.server_config.read_bytes()
    first_client = first.client_config.read_bytes()

    second = generate_service_configs(BINDING, **kwargs)

    assert second.server_config.read_bytes() == first_server
    assert second.client_config.read_bytes() == first_client
    assert not list(tmp_path.glob("*.tmp"))


def test_default_output_uses_cwd_and_binding_stem(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    generated = generate_service_configs(
        BINDING,
        offset_dir=OFFSETS,
        ros_interface_resolver=lambda _ros_type: None,
        pdu_type_resolver=lambda _ros_type, _override: "hako_srv_msgs/AddTwoInts",
    )

    assert generated.output_dir == (
        tmp_path / "build" / "generated" / "service" / "add_two_ints"
    )


def test_offset_sizes_match_existing_add_two_ints_contract() -> None:
    assert resolve_offset_size(OFFSETS, "hako_srv_msgs/AddTwoIntsRequestPacket") == 296
    assert resolve_offset_size(OFFSETS, "hako_srv_msgs/AddTwoIntsResponsePacket") == 288


def test_heap_mapping_and_channel_ids_are_scoped_per_service(tmp_path: Path) -> None:
    config_path = _copy_binding(tmp_path)
    data = _load(config_path)
    first = data["bindings"][0]
    first["max_clients"] = 1
    first["heap"] = {"request_bytes": 64, "response_bytes": 128}
    second = json.loads(json.dumps(first))
    second["ros_name"] = "/subtract_two_ints"
    second["hakoniwa_service"] = "Service/Subtract"
    data["bindings"].append(second)
    _write(config_path, data)

    generated = generate_service_configs(
        config_path,
        output_dir=tmp_path / "generated",
        offset_dir=OFFSETS,
        ros_interface_resolver=lambda _ros_type: None,
        pdu_type_resolver=lambda _ros_type, _override: "hako_srv_msgs/AddTwoInts",
    )
    services = _load(generated.server_config)["services"]

    assert services[0]["pduSize"]["server"]["heapSize"] == 64
    assert services[0]["pduSize"]["client"]["heapSize"] == 128
    assert [
        (
            service["clients"][0]["requestChannelId"],
            service["clients"][0]["responseChannelId"],
        )
        for service in services
    ] == [(0, 1), (0, 1)]


def test_service_key_uses_hakoniwa_service_tail() -> None:
    assert service_key("Service/AddTwoInts") == "add_two_ints"


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("max_clients", 0, "max_clients must be a positive integer"),
        ("timeout_msec", True, "timeout_msec must be a positive integer"),
    ],
)
def test_binding_rejects_invalid_positive_integer(
    tmp_path: Path, field: str, value: object, message: str
) -> None:
    config_path = _copy_binding(tmp_path)
    data = _load(config_path)
    data["bindings"][0][field] = value
    _write(config_path, data)

    with pytest.raises(ValueError, match=message):
        load_service_binding(config_path)


def test_binding_rejects_unknown_field(tmp_path: Path) -> None:
    config_path = _copy_binding(tmp_path)
    data = _load(config_path)
    data["bindings"][0]["rpc_client_names"] = ["must-not-be-user-input"]
    _write(config_path, data)

    with pytest.raises(ValueError, match="Unsupported bindings\\[0\\] fields"):
        load_service_binding(config_path)


@pytest.mark.parametrize("kind", [None, "server", "ros_service_client"])
def test_binding_rejects_missing_or_unsupported_kind(
    tmp_path: Path, kind: str | None
) -> None:
    config_path = _copy_binding(tmp_path)
    data = _load(config_path)
    if kind is None:
        del data["kind"]
    else:
        data["kind"] = kind
    _write(config_path, data)

    with pytest.raises(ValueError, match="kind must be 'ros_service_server'"):
        load_service_binding(config_path)


def test_binding_rejects_missing_endpoint_reference(tmp_path: Path) -> None:
    config_path = _copy_binding(tmp_path)
    data = _load(config_path)
    data["rpc"]["client_endpoint"]["endpoint_id"] = "missing"
    _write(config_path, data)

    with pytest.raises(ValueError, match="not present in rpc.endpoint_config"):
        load_service_binding(config_path)


def _copy_binding(tmp_path: Path) -> Path:
    config_path = tmp_path / "binding.json"
    endpoint_path = tmp_path / "rpc-endpoints.json"
    config_path.write_text(BINDING.read_text(encoding="utf-8"), encoding="utf-8")
    endpoint_path.write_text(
        (BINDING.parent / "rpc-endpoints.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    return config_path


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _write(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data), encoding="utf-8")
