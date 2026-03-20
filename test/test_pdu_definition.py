from pathlib import Path

from hakoniwa_pdu_ros.pdu_definition import PduDefinition


def test_load_compact_pdudef() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    pdudef_path = repo_root / "config" / "sample" / "pdu" / "pdudef.json"

    definition = PduDefinition()
    definition.load(pdudef_path)

    pos = definition.get("Drone", "pos")
    cmd = definition.get("Drone2", "cmd")

    assert pos.channel_id == 0
    assert pos.pdu_size == 72
    assert pos.type == "geometry_msgs/Twist"
    assert cmd.channel_id == 1
    assert cmd.pdu_size == 72
    assert cmd.type == "geometry_msgs/Twist"
