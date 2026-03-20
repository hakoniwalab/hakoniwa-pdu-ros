from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from hakoniwa_pdu_ros.env_setup import configure_import_paths

configure_import_paths()

try:
    from hakoniwa_pdu.impl.pdu_channel_config import PduChannelConfig as _ExternalPduChannelConfig
except ImportError:  # pragma: no cover - exercised only when external package is absent
    _ExternalPduChannelConfig = None


@dataclass(frozen=True)
class PduType:
    channel_id: int
    pdu_size: int
    name: str
    type: str


class PduDefinition:
    """Resolves ROS-side PDU metadata, preferring hakoniwa-pdu-python when available."""

    def __init__(self) -> None:
        self._definitions: dict[str, dict[str, PduType]] = {}
        self._channel_config = None

    def load(self, pdudef_path: str | Path) -> None:
        path = Path(pdudef_path).expanduser().resolve()
        if _ExternalPduChannelConfig is not None:
            self._load_external(path)
            return

        with path.open("r", encoding="utf-8") as f:
            config = json.load(f)

        if "paths" in config:
            self._load_compact(config, path.parent)
            return
        self._load_legacy(config)

    def get(self, robot_name: str, pdu_name: str) -> PduType:
        try:
            return self._definitions[robot_name][pdu_name]
        except KeyError as exc:
            raise KeyError(f"PDU not found: robot={robot_name} pdu={pdu_name}") from exc

    def get_pdu_channel_id(self, robot_name: str, pdu_name: str) -> int:
        return self.get(robot_name, pdu_name).channel_id

    def get_pdu_size(self, robot_name: str, pdu_name: str) -> int:
        return self.get(robot_name, pdu_name).pdu_size

    def get_pdu_type(self, robot_name: str, pdu_name: str) -> str:
        return self.get(robot_name, pdu_name).type

    def list_robot_pdus(self, robot_name: str) -> list[PduType]:
        return list(self._definitions.get(robot_name, {}).values())

    def _load_external(self, pdudef_path: Path) -> None:
        channel_config = _ExternalPduChannelConfig(str(pdudef_path))
        self._channel_config = channel_config
        if hasattr(channel_config, "get_pdudef_compact"):
            definitions: dict[str, dict[str, PduType]] = {}
            compact = channel_config.get_pdudef_compact()
            for robot in compact.get("robots", []):
                robot_name = robot["name"]
                definitions[robot_name] = {
                    pdu["name"]: PduType(
                        channel_id=pdu["channel_id"],
                        pdu_size=pdu["pdu_size"],
                        name=pdu["name"],
                        type=pdu["type"],
                    )
                    for pdu in robot.get("pdus", [])
                }
            self._definitions = definitions
            return

        raw_pdudef = channel_config.get_pdudef()
        if "paths" in raw_pdudef:
            self._load_compact(raw_pdudef, pdudef_path.parent)
            return
        self._load_legacy(raw_pdudef)

    def _load_compact(self, config: dict, base_dir: Path) -> None:
        pdutype_sets: dict[str, list[PduType]] = {}
        for entry in config["paths"]:
            set_id = entry["id"]
            raw_path = Path(entry["path"])
            resolved = raw_path if raw_path.is_absolute() else (base_dir / raw_path)
            with resolved.open("r", encoding="utf-8") as f:
                pdutypes = json.load(f)
            pdutype_sets[set_id] = [
                PduType(
                    channel_id=item["channel_id"],
                    pdu_size=item["pdu_size"],
                    name=item["name"],
                    type=item["type"],
                )
                for item in pdutypes
            ]

        definitions: dict[str, dict[str, PduType]] = {}
        for robot_def in config["robots"]:
            robot_name = robot_def["name"]
            set_id = robot_def["pdutypes_id"]
            if set_id not in pdutype_sets:
                raise ValueError(f"Unknown pdutypes_id: {set_id}")
            definitions[robot_name] = {item.name: item for item in pdutype_sets[set_id]}
        self._definitions = definitions

    def _load_legacy(self, config: dict) -> None:
        definitions: dict[str, dict[str, PduType]] = {}
        for robot_def in config["robots"]:
            robot_name = robot_def["name"]
            robot_pdus: dict[str, PduType] = {}
            for section in (
                "shm_pdu_readers",
                "shm_pdu_writers",
                "rpc_pdu_readers",
                "rpc_pdu_writers",
            ):
                for item in robot_def.get(section, []):
                    name = item["org_name"]
                    robot_pdus.setdefault(
                        name,
                        PduType(
                            channel_id=item["channel_id"],
                            pdu_size=item["pdu_size"],
                            name=name,
                            type=item["type"],
                        ),
                    )
            definitions[robot_name] = robot_pdus
        self._definitions = definitions
