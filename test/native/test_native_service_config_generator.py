from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from hakoniwa_pdu_ros.service_config_generator import generate_service_configs


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_add_two_ints_resolves_installed_ros_and_pdu_types(tmp_path: Path) -> None:
    generated = generate_service_configs(
        REPO_ROOT / "config" / "service" / "add_two_ints.json",
        output_dir=tmp_path,
        offset_dir=REPO_ROOT / "test" / "fixtures" / "offset",
    )

    server = json.loads(generated.server_config.read_text(encoding="utf-8"))
    client = json.loads(generated.client_config.read_text(encoding="utf-8"))

    assert server["services"][0]["type"] == "hako_srv_msgs/AddTwoInts"
    assert server["services"][0]["pduSize"]["server"]["baseSize"] == 296
    assert client["services"][0]["clients"][3] == {
        "name": "hakoniwa_pdu_ros_add_3",
        "requestChannelId": 6,
        "responseChannelId": 7,
        "client_endpoint": {
            "nodeId": "hakoniwa-pdu-ros-service",
            "endpointId": "hakoniwa-pdu-ros-service-service-tcp",
        },
    }


def test_add_two_ints_generator_cli(tmp_path: Path) -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "hakoniwa_pdu_ros.generate_service_config",
            "--config",
            str(REPO_ROOT / "config" / "service" / "add_two_ints.json"),
            "--offset-dir",
            str(REPO_ROOT / "test" / "fixtures" / "offset"),
            "--output-dir",
            str(tmp_path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "rpc-server-services.json" in result.stdout
    assert "rpc-client-services.json" in result.stdout
    assert (tmp_path / "rpc-server-services.json").is_file()
    assert (tmp_path / "rpc-client-services.json").is_file()
