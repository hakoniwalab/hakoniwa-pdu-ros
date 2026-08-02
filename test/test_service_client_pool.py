from __future__ import annotations

from concurrent.futures import Future

import pytest

from hakoniwa_pdu_ros.service_client_pool import create_rpc_client_pool


class FakeClient:
    def __init__(self) -> None:
        self.started = False
        self.closed = False

    def start(self) -> None:
        self.started = True

    def close(self) -> None:
        self.closed = True


def test_pool_rejects_when_capacity_is_exhausted_and_reuses_release() -> None:
    raw_clients: list[FakeClient] = []

    def factory(_name: str):
        raw = FakeClient()
        raw_clients.append(raw)
        return raw, object()

    pool = create_rpc_client_pool(
        max_clients=2,
        client_name=lambda index: f"client_{index}",
        client_factory=factory,
    )
    first = pool.acquire()
    second = pool.acquire()

    assert first is not None
    assert second is not None
    assert pool.acquire() is None
    assert pool.active_count == 2

    pool.release(first)
    assert pool.acquire() is first
    assert all(client.started for client in raw_clients)
    pool.close()
    assert all(client.closed for client in raw_clients)


def test_serial_calls_keep_other_clients_available_for_concurrency() -> None:
    pool = create_rpc_client_pool(
        max_clients=3,
        client_name=lambda index: f"client_{index}",
        client_factory=lambda _name: (FakeClient(), object()),
    )

    first = pool.acquire()
    assert first is not None
    pool.release(first)
    reused = pool.acquire()
    second = pool.acquire()

    assert reused is first
    assert second is not None
    assert second.name == "client_1"
    pool.release(reused)
    pool.release(second)
    pool.close()


def test_close_cancels_active_future_before_closing_clients() -> None:
    raw = FakeClient()
    pool = create_rpc_client_pool(
        max_clients=1,
        client_name=lambda _index: "client_0",
        client_factory=lambda _name: (raw, object()),
    )
    lease = pool.acquire()
    assert lease is not None
    future: Future[object] = Future()
    pool.set_future(lease, future)

    pool.close(timeout_sec=0.01)

    assert future.cancelled()
    assert raw.closed
    assert pool.acquire() is None


def test_pool_closes_all_clients_when_startup_fails() -> None:
    clients: list[FakeClient] = []

    class FailingClient(FakeClient):
        def start(self) -> None:
            raise RuntimeError("start failed")

    def factory(name: str):
        client = FailingClient() if name == "client_1" else FakeClient()
        clients.append(client)
        return client, object()

    with pytest.raises(RuntimeError, match="start failed"):
        create_rpc_client_pool(
            max_clients=2,
            client_name=lambda index: f"client_{index}",
            client_factory=factory,
        )

    assert all(client.closed for client in clients)
