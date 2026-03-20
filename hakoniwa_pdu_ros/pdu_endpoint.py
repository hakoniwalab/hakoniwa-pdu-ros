from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from hakoniwa_pdu_ros.env_setup import configure_import_paths

configure_import_paths()

from hakoniwa_pdu_endpoint.c_endpoint import PduKey  # noqa: E402
from hakoniwa_pdu_endpoint.c_endpoint_async import EndpointAsync, PduEvent  # noqa: E402
from hakoniwa_pdu_endpoint.c_endpoint import Endpoint  # noqa: E402


class PduEndpointManager:
    """Thin ROS-side wrapper around hakoniwa-pdu-endpoint Python bindings."""

    def __init__(self, endpoint_config_path: str | Path, direction: str = "inout") -> None:
        self._endpoint_config_path = str(Path(endpoint_config_path).expanduser().resolve())
        self._endpoint = Endpoint("hakoniwa_pdu_ros", direction)
        self._async = EndpointAsync(self._endpoint)
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._async.open(self._endpoint_config_path)
        self._async.start()
        self._async.post_start()
        self._async.start_dispatch()
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        self._async.stop_dispatch()
        self._async.stop()
        self._async.close()
        self._started = False

    def subscribe_recv(
        self,
        robot_name: str,
        pdu_name: str,
        callback: Callable[[bytes], None],
    ) -> None:
        key = PduKey(robot=robot_name, pdu=pdu_name)

        def _on_event(event: PduEvent) -> None:
            callback(event.payload)

        self._async.on_recv_by_name(key, _on_event)

    def send(
        self,
        robot_name: str,
        pdu_name: str,
        data: bytes,
    ) -> None:
        key = PduKey(robot=robot_name, pdu=pdu_name)
        self._async.send_by_name(key, data)
