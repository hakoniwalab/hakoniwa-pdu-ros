from pathlib import Path
import json

from hakoniwa_pdu_ros.config_loader import load_config


def test_load_sample_bridge_config() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    config = load_config(repo_root / "config" / "sample" / "sample_binding.json")

    assert config.endpoint_config.name == "endpoint_zenoh.json"
    assert config.pdu_def_path.name == "pdudef.json"
    assert len(config.bindings) == 2

    pos_binding = config.bindings[0]
    cmd_binding = config.bindings[1]

    assert pos_binding.direction == "pdu_to_ros"
    assert pos_binding.pdu_key.robot_name == "Drone"
    assert pos_binding.pdu_key.pdu_name == "pos"
    assert pos_binding.topic == "/hakoniwa/drone/pos"
    assert pos_binding.channel_id == 0
    assert pos_binding.pdu_size == 72
    assert pos_binding.pdu_type == "geometry_msgs/Twist"

    assert cmd_binding.direction == "ros_to_pdu"
    assert cmd_binding.pdu_key.robot_name == "Drone"
    assert cmd_binding.pdu_key.pdu_name == "cmd"
    assert cmd_binding.topic == "/hakoniwa/drone/cmd"
    assert cmd_binding.channel_id == 1
    assert cmd_binding.pdu_size == 72
    assert cmd_binding.pdu_type == "geometry_msgs/Twist"


def test_load_bidirectional_binding_when_direction_is_omitted() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    config = load_config(
        repo_root / "examples" / "zenoh" / "config" / "zenoh_bidirectional_binding.json"
    )

    assert len(config.bindings) == 4

    command_binding = config.bindings[0]
    command_send_binding = config.bindings[1]
    debuginfo_binding = config.bindings[2]
    debuginfo_send_binding = config.bindings[3]

    assert command_binding.direction == "pdu_to_ros"
    assert command_binding.pdu_key.robot_name == "demo"
    assert command_binding.pdu_key.pdu_name == "command"
    assert command_binding.topic == "/from_pdu/demo/command"
    assert command_binding.channel_id == 0
    assert command_binding.pdu_type == "std_msgs/UInt16"

    assert command_send_binding.direction == "ros_to_pdu"
    assert command_send_binding.pdu_key.pdu_name == "command"
    assert command_send_binding.topic == "/to_pdu/demo/command"
    assert command_send_binding.channel_id == 0
    assert command_send_binding.pdu_type == "std_msgs/UInt16"

    assert debuginfo_binding.direction == "pdu_to_ros"
    assert debuginfo_binding.pdu_key.robot_name == "demo"
    assert debuginfo_binding.pdu_key.pdu_name == "debuginfo"
    assert debuginfo_binding.topic == "/from_pdu/demo/debuginfo"
    assert debuginfo_binding.channel_id == 1
    assert debuginfo_binding.pdu_type == "std_msgs/UInt16"

    assert debuginfo_send_binding.direction == "ros_to_pdu"
    assert debuginfo_send_binding.pdu_key.pdu_name == "debuginfo"
    assert debuginfo_send_binding.topic == "/to_pdu/demo/debuginfo"
    assert debuginfo_send_binding.channel_id == 1
    assert debuginfo_send_binding.pdu_type == "std_msgs/UInt16"


def test_reject_same_topic_for_both_directions(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parent.parent
    src_config = repo_root / "examples" / "zenoh" / "config"

    config = {
        "endpoint_config": str(src_config / "endpoint_ros_bridge.json"),
        "bindings": [
            {
                "pdu_key": {"robot_name": "demo", "pdu_name": "command"},
                "direction": "pdu_to_ros",
                "topic": "/demo/command",
            },
            {
                "pdu_key": {"robot_name": "demo", "pdu_name": "command"},
                "direction": "ros_to_pdu",
                "topic": "/demo/command",
            },
        ],
    }
    config_path = tmp_path / "conflicting_binding.json"
    config_path.write_text(json.dumps(config), encoding="utf-8")

    try:
        load_config(config_path)
    except ValueError as err:
        assert "same ROS topic" in str(err)
    else:
        assert False, "expected ValueError for bidirectional bindings on the same topic"
