#!/usr/bin/env python3
"""Minimal Hakoniwa AddTwoInts RPC server for the manual ROS Service demo."""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

from hakoniwa_pdu_rpc import RpcMuxServer, ServerEvent, load_service_wire


SERVICE_NAME = "Service/Add"
SERVICE_TYPE = "AddTwoInts"
STATUS_DONE = 3
RESULT_OK = 0
RESULT_CANCELED = 2


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service-config", required=True)
    parser.add_argument("--endpoint-config", required=True)
    parser.add_argument("--rpc-library", default=os.environ.get("HAKO_PDU_RPC_LIBRARY"))
    parser.add_argument("--node-id", default="server_node")
    args = parser.parse_args()
    if not args.rpc_library:
        parser.error("Specify --rpc-library or set HAKO_PDU_RPC_LIBRARY")

    wire = load_service_wire(SERVICE_TYPE)
    with RpcMuxServer(
        Path(args.rpc_library),
        args.node_id,
        Path(args.service_config),
        Path(args.endpoint_config),
    ) as server:
        server.start()
        print(
            f"AddTwoInts RPC Server started: service={SERVICE_NAME} "
            f"endpoint={args.endpoint_config}",
            flush=True,
        )
        print("Press Ctrl+C to stop.", flush=True)
        try:
            while True:
                incoming = server.poll()
                if incoming.event == ServerEvent.NONE:
                    time.sleep(0.001)
                    continue
                if incoming.service_name != SERVICE_NAME:
                    raise RuntimeError(
                        f"unexpected RPC service: {incoming.service_name!r}"
                    )
                if incoming.event == ServerEvent.REQUEST_CANCEL:
                    reply = server.create_reply_buffer(
                        incoming.request_token,
                        status=STATUS_DONE,
                        result_code=RESULT_CANCELED,
                    )
                    server.send_cancel_reply(incoming.request_token, reply)
                    print(
                        f"canceled: client={incoming.client_name}",
                        flush=True,
                    )
                    continue
                if incoming.event != ServerEvent.REQUEST_IN:
                    raise RuntimeError(f"unexpected RPC event: {incoming.event}")

                request = wire.request_decode(incoming.pdu)
                left = int(request.body.a)
                right = int(request.body.b)
                reply = server.create_reply_buffer(
                    incoming.request_token,
                    status=STATUS_DONE,
                    result_code=RESULT_OK,
                )
                response = wire.response_decode(reply)
                response.body.sum = left + right
                server.send_reply(
                    incoming.request_token,
                    wire.response_encode(response),
                )
                print(
                    f"call: client={incoming.client_name} "
                    f"{left} + {right} = {left + right}",
                    flush=True,
                )
        except KeyboardInterrupt:
            print("Stopping AddTwoInts RPC Server.", flush=True)


if __name__ == "__main__":
    main()
