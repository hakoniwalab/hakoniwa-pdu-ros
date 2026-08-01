from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from hakoniwa_pdu_rpc import RpcClient, RpcServer, ServerEvent, load_service_wire
from hakoniwa_pdu_rpc import make_typed_client


SERVICE_NAME = "Service/Add"
SERVICE_TYPE = "AddTwoInts"
SERVER_NODE_ID = "server_node"
CLIENT_NODE_ID = "hakoniwa-pdu-ros-service"
CLIENT_NAME = "hakoniwa_pdu_ros_add_0"
STATUS_DONE = 3
RESULT_OK = 0


class AddTwoIntsRpcServer:
    """Small ROS-independent RPC server reusable by later Service Node tests."""

    def __init__(
        self,
        library_path: str | Path,
        service_config_path: str | Path,
        endpoint_config_path: str | Path,
    ) -> None:
        self._server = RpcServer(
            library_path,
            SERVER_NODE_ID,
            service_config_path,
            endpoint_config_path,
        )
        self._wire = load_service_wire(SERVICE_TYPE)

    def start(self) -> None:
        self._server.start()

    def serve_once(self, timeout_sec: float = 3.0) -> tuple[int, int, int]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            incoming = self._server.poll()
            if incoming.event == ServerEvent.NONE:
                time.sleep(0.001)
                continue
            if incoming.event != ServerEvent.REQUEST_IN:
                raise RuntimeError(f"unexpected RPC server event: {incoming.event}")
            if incoming.service_name != SERVICE_NAME:
                raise RuntimeError(
                    f"unexpected RPC service: {incoming.service_name!r}"
                )

            request_packet = self._wire.request_decode(incoming.pdu)
            left = int(request_packet.body.a)
            right = int(request_packet.body.b)

            reply = self._server.create_reply_buffer(
                incoming.request_token,
                status=STATUS_DONE,
                result_code=RESULT_OK,
            )
            response_packet = self._wire.response_decode(reply)
            response_packet.body.sum = left + right
            self._server.send_reply(
                incoming.request_token,
                self._wire.response_encode(response_packet),
            )
            return left, right, left + right

        raise TimeoutError("AddTwoInts RPC request was not received")

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
