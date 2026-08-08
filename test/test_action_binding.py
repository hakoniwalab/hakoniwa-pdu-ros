from __future__ import annotations

import json
from pathlib import Path

import pytest

from hakoniwa_pdu_ros.action_binding import load_action_binding


REPO_ROOT = Path(__file__).resolve().parents[1]
ACTION_CONFIG = REPO_ROOT / "config" / "action"


def test_loads_direction_neutral_action_binding() -> None:
    config = load_action_binding(ACTION_CONFIG / "fibonacci.json")

    assert config.version == 1
    assert config.action.transport_config.is_file()
    assert config.action.delta_time_usec == 1000
    assert len(config.bindings) == 1
    binding = config.bindings[0]
    assert binding.ros_type == "action_tutorials_interfaces/action/Fibonacci"
    assert binding.hakoniwa_action == "fibonacci"
    assert binding.pdu_action_type == "sample_action_msgs/Fibonacci"
    assert binding.slot_count == 4
    assert binding.heap.goal_bytes == 1048576
    assert binding.client_endpoint.node_id == "fibonacci-client"
    assert binding.server_endpoint.node_id == "fibonacci-server"


def test_omitted_heap_values_remain_unresolved_for_rpc_generator(
    tmp_path: Path,
) -> None:
    path = _copy_binding(tmp_path)
    data = _load(path)
    del data["bindings"][0]["heap"]
    _write(path, data)

    heap = load_action_binding(path).bindings[0].heap

    assert heap.goal_bytes is None
    assert heap.result_bytes is None
    assert heap.feedback_bytes is None


def test_rejects_runtime_direction_in_binding(tmp_path: Path) -> None:
    path = _copy_binding(tmp_path)
    data = _load(path)
    data["kind"] = "ros_action_server"
    _write(path, data)

    with pytest.raises(ValueError, match="Unsupported root fields: kind"):
        load_action_binding(path)


def test_rejects_unknown_binding_field(tmp_path: Path) -> None:
    path = _copy_binding(tmp_path)
    data = _load(path)
    data["bindings"][0]["channel_ids"] = [0, 1, 2]
    _write(path, data)

    with pytest.raises(ValueError, match=r"Unsupported bindings\[0\] fields"):
        load_action_binding(path)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("slot_count", 0, "slot_count must be a positive integer"),
        (
            "goal_response_timeout_msec",
            True,
            "goal_response_timeout_msec must be a positive integer",
        ),
    ],
)
def test_rejects_invalid_positive_integer(
    tmp_path: Path, field: str, value: object, message: str
) -> None:
    path = _copy_binding(tmp_path)
    data = _load(path)
    data["bindings"][0][field] = value
    _write(path, data)

    with pytest.raises(ValueError, match=message):
        load_action_binding(path)


def test_rejects_invalid_heap(tmp_path: Path) -> None:
    path = _copy_binding(tmp_path)
    data = _load(path)
    data["bindings"][0]["heap"]["feedback_bytes"] = -1
    _write(path, data)

    with pytest.raises(
        ValueError, match="feedback_bytes must be a non-negative integer"
    ):
        load_action_binding(path)


def test_rejects_endpoint_missing_from_transport(tmp_path: Path) -> None:
    path = _copy_binding(tmp_path)
    data = _load(path)
    data["bindings"][0]["server_endpoint"]["node_id"] = "missing-server"
    _write(path, data)

    with pytest.raises(ValueError, match="is not present in action.transport_config"):
        load_action_binding(path)


def test_rejects_duplicate_ros_and_hakoniwa_names(tmp_path: Path) -> None:
    path = _copy_binding(tmp_path)
    data = _load(path)
    data["bindings"].append(json.loads(json.dumps(data["bindings"][0])))
    _write(path, data)

    with pytest.raises(ValueError, match="Duplicate ROS action name"):
        load_action_binding(path)


def test_rejects_pdu_action_type_with_different_basename(tmp_path: Path) -> None:
    path = _copy_binding(tmp_path)
    data = _load(path)
    data["bindings"][0]["pdu_action_type"] = "sample_action_msgs/Other"
    _write(path, data)

    with pytest.raises(ValueError, match="basename must match ros_type"):
        load_action_binding(path)


def _copy_binding(tmp_path: Path) -> Path:
    source = ACTION_CONFIG / "fibonacci.json"
    transport_source = ACTION_CONFIG / "fibonacci-transport.json"
    path = tmp_path / "binding.json"
    transport_path = tmp_path / transport_source.name
    path.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    transport_path.write_text(
        transport_source.read_text(encoding="utf-8"), encoding="utf-8"
    )
    return path


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _write(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data), encoding="utf-8")
