from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hakoniwa_pdu_rpc import RpcClient, RpcMuxServer, ServerEvent, load_service_wire
from hakoniwa_pdu_rpc import make_typed_client


SERVICE_NAME = "Service/Add"
SERVICE_TYPE = "AddTwoInts"
SERVER_NODE_ID = "server_node"
CLIENT_NODE_ID = "hakoniwa-pdu-ros-service"
CLIENT_NAME = "hakoniwa_pdu_ros_add_0"
STATUS_DONE = 3
RESULT_OK = 0
RESULT_CANCELED = 2


@dataclass(frozen=True)
class AddTwoIntsRequest:
    request_token: int
    left: int
    right: int


class AddTwoIntsRpcServer:
    """Small ROS-independent RPC server reusable by later Service Node tests."""

    def __init__(
        self,
        library_path: str | Path,
        service_config_path: str | Path,
        endpoint_mux_config_path: str | Path,
    ) -> None:
        self._server = RpcMuxServer(
            library_path,
            SERVER_NODE_ID,
            service_config_path,
            endpoint_mux_config_path,
        )
        self._wire = load_service_wire(SERVICE_TYPE)

    def start(self) -> None:
        self._server.start()

    def wait_connected(self, expected: int, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._server.poll()
            if self._server.connected_count() == expected:
                return
            time.sleep(0.001)
        raise TimeoutError(
            "RPC mux connection timed out: "
            f"{self._server.connected_count()} of {expected}"
        )

    def serve_once(self, timeout_sec: float = 3.0) -> tuple[int, int, int]:
        request = self.receive_request(timeout_sec=timeout_sec)
        self.send_sum(request)
        return request.left, request.right, request.left + request.right

    def receive_request(self, timeout_sec: float = 3.0) -> AddTwoIntsRequest:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            incoming = self._server.poll()
            if incoming.event == ServerEvent.NONE:
                time.sleep(0.001)
                continue
            if incoming.event == ServerEvent.REQUEST_CANCEL:
                # A late cancel may remain queued after a normal reply won the
                # terminal race. It must not be mistaken for a new request.
                continue
            if incoming.event != ServerEvent.REQUEST_IN:
                raise RuntimeError(f"unexpected RPC server event: {incoming.event}")
            if incoming.service_name != SERVICE_NAME:
                raise RuntimeError(
                    f"unexpected RPC service: {incoming.service_name!r}"
                )

            request_packet = self._wire.request_decode(incoming.pdu)
            return AddTwoIntsRequest(
                request_token=incoming.request_token,
                left=int(request_packet.body.a),
                right=int(request_packet.body.b),
            )

        raise TimeoutError("AddTwoInts RPC request was not received")

    def send_sum(self, request: AddTwoIntsRequest) -> None:
        reply = self._server.create_reply_buffer(
            request.request_token,
            status=STATUS_DONE,
            result_code=RESULT_OK,
        )
        response_packet = self._wire.response_decode(reply)
        response_packet.body.sum = request.left + request.right
        self._server.send_reply(
            request.request_token,
            self._wire.response_encode(response_packet),
        )

    def receive_cancel(self, timeout_sec: float = 3.0) -> int:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            incoming = self._server.poll()
            if incoming.event == ServerEvent.NONE:
                time.sleep(0.001)
                continue
            if incoming.event != ServerEvent.REQUEST_CANCEL:
                raise RuntimeError(f"unexpected RPC server event: {incoming.event}")
            if incoming.service_name != SERVICE_NAME:
                raise RuntimeError(
                    f"unexpected canceled service: {incoming.service_name}"
                )
            return incoming.request_token
        raise TimeoutError("AddTwoInts RPC cancel was not received")

    def send_cancel(self, request_token: int) -> None:
        reply = self._server.create_reply_buffer(
            request_token,
            status=STATUS_DONE,
            result_code=RESULT_CANCELED,
        )
        self._server.send_cancel_reply(request_token, reply)

    def close(self) -> None:
        self._server.close()

    def __enter__(self) -> "AddTwoIntsRpcServer":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()


def create_add_two_ints_client(
    library_path: str | Path,
    service_config_path: str | Path,
    endpoint_config_path: str | Path,
):
    rpc_client = RpcClient(
        library_path,
        CLIENT_NODE_ID,
        CLIENT_NAME,
        service_config_path,
        endpoint_config_path,
    )
    return rpc_client, make_typed_client(
        rpc_client,
        service_name=SERVICE_NAME,
        service_type=SERVICE_TYPE,
    )
