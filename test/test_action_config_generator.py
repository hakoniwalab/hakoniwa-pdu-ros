from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

import hakoniwa_pdu_ros.action_config_generator as action_generator
from hakoniwa_pdu_ros.action_config_generator import generate_action_configs


REPO_ROOT = Path(__file__).resolve().parents[1]
PDU_RPC_PYTHON = REPO_ROOT.parent / "hakoniwa-pdu-rpc" / "python"
if str(PDU_RPC_PYTHON) not in sys.path:
    sys.path.insert(0, str(PDU_RPC_PYTHON))

from hakoniwa_pdu_rpc.action_config_generator import generate as generate_rpc


BINDING = REPO_ROOT / "config" / "action" / "fibonacci.json"


def test_resolves_explicit_pdu_action_type(monkeypatch) -> None:
    validated = []
    monkeypatch.setattr(
        action_generator,
        "_validate_pdu_modules",
        lambda package, action: validated.append((package, action)),
    )

    resolved = action_generator.resolve_installed_pdu_action_type(
        "sample_action_msgs/action/Fibonacci",
        "sample_action_msgs/Fibonacci",
    )

    assert resolved == "sample_action_msgs/Fibonacci"
    assert validated == [("sample_action_msgs", "Fibonacci")]


def test_auto_resolves_one_complete_pdu_action_package(monkeypatch) -> None:
    monkeypatch.setattr(
        action_generator.importlib,
        "import_module",
        lambda name: SimpleNamespace(__path__=["unused"]),
    )
    monkeypatch.setattr(
        action_generator.pkgutil,
        "iter_modules",
        lambda _path: [
            SimpleNamespace(name="incomplete_msgs", ispkg=True),
            SimpleNamespace(name="sample_action_msgs", ispkg=True),
        ],
    )

    def validate(package: str, _action: str) -> None:
        if package != "sample_action_msgs":
            raise ValueError("incomplete")

    monkeypatch.setattr(action_generator, "_validate_pdu_modules", validate)

    assert action_generator.resolve_installed_pdu_action_type(
        "sample_action_msgs/action/Fibonacci", None
    ) == "sample_action_msgs/Fibonacci"


def test_rejects_ambiguous_pdu_action_packages(monkeypatch) -> None:
    monkeypatch.setattr(
        action_generator.importlib,
        "import_module",
        lambda name: SimpleNamespace(__path__=["unused"]),
    )
    monkeypatch.setattr(
        action_generator.pkgutil,
        "iter_modules",
        lambda _path: [
            SimpleNamespace(name="first_msgs", ispkg=True),
            SimpleNamespace(name="second_msgs", ispkg=True),
        ],
    )
    monkeypatch.setattr(
        action_generator, "_validate_pdu_modules", lambda _package, _action: None
    )

    with pytest.raises(ValueError, match="ambiguous"):
        action_generator.resolve_installed_pdu_action_type(
            "sample_action_msgs/action/Fibonacci", None
        )


def test_maps_binding_to_pdu_rpc_manifest_without_owning_native_rules(
    tmp_path: Path,
) -> None:
    observed = {}

    def capture(manifest_path: Path, output_dir: Path) -> list[Path]:
        observed["manifest"] = _load(manifest_path)
        observed["output"] = output_dir
        marker = output_dir / "delegated.json"
        marker.write_text("{}\n", encoding="utf-8")
        return [marker]

    generated = generate_action_configs(
        BINDING,
        output_dir=tmp_path,
        ros_interface_resolver=lambda _ros_type: None,
        pdu_type_resolver=lambda _ros_type, _override: (
            "sample_action_msgs/Fibonacci"
        ),
        rpc_generator=capture,
    )

    action = observed["manifest"]["actions"][0]
    assert action == {
        "name": "fibonacci",
        "type": "sample_action_msgs/Fibonacci",
        "slotCount": 4,
        "bufferHeap": {
            "requestSize": 1048576,
            "responseSize": 1048576,
            "feedbackSize": 1048576,
        },
        "clientEndpoint": {"nodeId": "fibonacci-client"},
        "serverEndpoint": {"nodeId": "fibonacci-server"},
    }
    assert "channels" not in action
    assert "endpointId" not in json.dumps(action)
    assert observed["output"] == tmp_path
    assert generated.generated_files == (tmp_path / "delegated.json",)


def test_public_pdu_rpc_generator_creates_runtime_files(tmp_path: Path) -> None:
    generated = generate_action_configs(
        BINDING,
        output_dir=tmp_path,
        ros_interface_resolver=lambda _ros_type: None,
        pdu_type_resolver=lambda _ros_type, _override: (
            "sample_action_msgs/Fibonacci"
        ),
        rpc_generator=generate_rpc,
    )

    assert generated.manifest == tmp_path / "hakoniwa-action.json"
    assert (tmp_path / "resolved-action.json").is_file()
    assert (tmp_path / "endpoints.json").is_file()
    assert (tmp_path / "queue.json").is_file()
    resolved = _load(tmp_path / "resolved-action.json")
    channels = resolved["actions"][0]["channels"]
    assert len(channels) == 12
    assert channels[0]["channelId"] == 0
    assert channels[-1]["channelId"] == 11
    assert channels[-1]["channelName"] == "Slot3Feedback"


def test_generation_is_idempotent(tmp_path: Path) -> None:
    kwargs = {
        "output_dir": tmp_path,
        "ros_interface_resolver": lambda _ros_type: None,
        "pdu_type_resolver": lambda _ros_type, _override: (
            "sample_action_msgs/Fibonacci"
        ),
        "rpc_generator": generate_rpc,
    }
    generate_action_configs(BINDING, **kwargs)
    first = {
        path.relative_to(tmp_path): path.read_bytes()
        for path in tmp_path.rglob("*.json")
    }
    generate_action_configs(BINDING, **kwargs)
    second = {
        path.relative_to(tmp_path): path.read_bytes()
        for path in tmp_path.rglob("*.json")
    }
    assert second == first
    assert not list(tmp_path.rglob("*.tmp"))


def test_default_output_uses_cwd_and_binding_stem(
    tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.chdir(tmp_path)
    generated = generate_action_configs(
        BINDING,
        ros_interface_resolver=lambda _ros_type: None,
        pdu_type_resolver=lambda _ros_type, _override: (
            "sample_action_msgs/Fibonacci"
        ),
        rpc_generator=generate_rpc,
    )
    assert generated.output_dir == (
        tmp_path / "build" / "generated" / "action" / "fibonacci"
    )


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))
