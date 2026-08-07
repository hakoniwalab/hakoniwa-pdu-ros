from __future__ import annotations

import json
from pathlib import Path

import pytest

from hakoniwa_pdu_ros.action_binding import load_action_binding


def _write(tmp_path: Path, data: dict) -> Path:
    path = tmp_path / "action-binding.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def _config() -> dict:
    return {
        "version": 1,
        "runtime": {
            "node_id": "ros-action-bridge",
            "client_name": "ros-action-client",
            "action_config": "resolved-action.json",
            "endpoint_config": "endpoints.json",
        },
        "actions": [
            {
                "ros_name": "/fibonacci",
                "ros_type": "example_interfaces/action/Fibonacci",
                "hakoniwa_action": "fibonacci",
                "pdu_action_type": "sample_action_msgs/Fibonacci",
            }
        ],
    }


def test_load_action_binding_resolves_runtime_paths(tmp_path: Path):
    config = load_action_binding(_write(tmp_path, _config()))
    assert config.runtime.node_id == "ros-action-bridge"
    assert config.runtime.action_config == (tmp_path / "resolved-action.json").resolve()
    assert config.runtime.endpoint_config == (tmp_path / "endpoints.json").resolve()
    assert config.actions[0].goal_response_timeout_msec == 5000


def test_load_action_binding_rejects_duplicate_ros_names(tmp_path: Path):
    data = _config()
    duplicate = dict(data["actions"][0])
    duplicate["hakoniwa_action"] = "other"
    data["actions"].append(duplicate)
    with pytest.raises(ValueError, match="duplicate ROS Action"):
        load_action_binding(_write(tmp_path, data))


def test_load_action_binding_rejects_duplicate_hakoniwa_actions(tmp_path: Path):
    data = _config()
    duplicate = dict(data["actions"][0])
    duplicate["ros_name"] = "/other"
    data["actions"].append(duplicate)
    with pytest.raises(ValueError, match="duplicate Hakoniwa Action"):
        load_action_binding(_write(tmp_path, data))


def test_load_action_binding_rejects_non_positive_timeout(tmp_path: Path):
    data = _config()
    data["actions"][0]["goal_response_timeout_msec"] = 0
    with pytest.raises(ValueError, match="positive integer"):
        load_action_binding(_write(tmp_path, data))
