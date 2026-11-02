import importlib.util
import sys
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "hakoniwa_pdu_ros"


def load_entrypoint(filename):
    spec = importlib.util.spec_from_file_location("test_ros_cli", PACKAGE_ROOT / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_topic_bridge_accepts_ros_remapping(monkeypatch):
    pytest.importorskip("rclpy")
    bridge = ModuleType("hakoniwa_pdu_ros.bridge_node")
    bridge.run = Mock()
    zenoh = ModuleType("hakoniwa_pdu_ros.zenoh_io")
    zenoh.ZenohIoValidationError = ValueError
    monkeypatch.setitem(sys.modules, bridge.__name__, bridge)
    monkeypatch.setitem(sys.modules, zenoh.__name__, zenoh)
    module = load_entrypoint("__main__.py")
    monkeypatch.setattr(module, "configure_import_paths", lambda: None)
    monkeypatch.setattr(
        sys, "argv",
        ["bridge", "--config", "topics.json", "--ros-args", "-r", "/pdu/joint_states:=/joint_states_robot"],
    )

    module.main()

    bridge.run.assert_called_once_with("topics.json")


def test_service_bridge_accepts_ros_remapping(monkeypatch):
    pytest.importorskip("rclpy")
    module = load_entrypoint("service_server_node.py")
    module.run = Mock()
    monkeypatch.setattr(module, "configure_import_paths", lambda: None)
    monkeypatch.setattr(
        sys, "argv",
        ["service-server", "--config", "services.json", "--offset-dir", "offset",
         "--ros-args", "-r", "/hakoniwa/service:=/service"],
    )

    module.main()

    module.run.assert_called_once_with(
        "services.json", output_dir=None, offset_dir="offset", rpc_library=None
    )
