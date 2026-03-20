from pathlib import Path

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
