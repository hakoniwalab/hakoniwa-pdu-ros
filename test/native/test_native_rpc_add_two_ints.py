from __future__ import annotations

import os
import threading
from pathlib import Path

from hakoniwa_pdu_ros.service_config_generator import generate_service_configs

from add_two_ints_rpc_fixture import (
    AddTwoIntsRpcServer,
    create_add_two_ints_client,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
BINDING = REPO_ROOT / "config" / "service" / "add_two_ints.json"
ENDPOINT_CONFIG = REPO_ROOT / "config" / "service" / "rpc-endpoints.json"
OFFSETS = REPO_ROOT / "test" / "fixtures" / "offset"


def test_generated_configs_support_typed_async_add_two_ints(
    tmp_path: Path,
) -> None:
    generated = generate_service_configs(
        BINDING,
        output_dir=tmp_path,
        offset_dir=OFFSETS,
    )
    library_path = os.environ["HAKO_PDU_RPC_LIBRARY"]

    with AddTwoIntsRpcServer(
        library_path,
        generated.server_config,
        ENDPOINT_CONFIG,
    ) as server:
        server.start()
        rpc_client, client = create_add_two_ints_client(
            library_path,
            generated.client_config,
            ENDPOINT_CONFIG,
        )
        worker: threading.Thread | None = None
        try:
            rpc_client.start()
            served: list[tuple[int, int, int]] = []
            errors: list[BaseException] = []

            def serve() -> None:
                try:
                    served.append(server.serve_once())
                except BaseException as error:
                    errors.append(error)

            worker = threading.Thread(target=serve, name="add-two-ints-server")
            worker.start()

            request = client.create_request()
            request.a = 19
            request.b = 23
            future = client.call_async(request, timeout_usec=1_000_000)
            response = future.result(timeout=5.0)

            worker.join(timeout=5.0)
            assert not worker.is_alive()
            assert not errors
            assert served == [(19, 23, 42)]
            assert response.sum == 42
        finally:
            if worker is not None and worker.is_alive():
                worker.join(timeout=5.0)
            rpc_client.close()
