from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable


@dataclass
class RpcClientLease:
    name: str
    rpc_client: Any
    typed_client: Any
    future: Any | None = None


class RpcClientPool:
    """Thread-safe, non-queuing pool of one-in-flight RPC clients."""

    def __init__(self, clients: list[RpcClientLease]) -> None:
        if not clients:
            raise ValueError("RPC client pool requires at least one client")
        self._clients = tuple(clients)
        self._available = deque(clients)
        self._leased: set[str] = set()
        self._closed = False
        self._lock = threading.Lock()

    @property
    def capacity(self) -> int:
        return len(self._clients)

    @property
    def active_count(self) -> int:
        with self._lock:
            return len(self._leased)

    def acquire(self) -> RpcClientLease | None:
        with self._lock:
            if self._closed or not self._available:
                return None
            lease = self._available.popleft()
            self._leased.add(lease.name)
            return lease

    def set_future(self, lease: RpcClientLease, future: Any) -> None:
        with self._lock:
            if lease.name not in self._leased:
                raise RuntimeError(f"RPC client is not leased: {lease.name}")
            lease.future = future

    def release(self, lease: RpcClientLease) -> None:
        with self._lock:
            if lease.name not in self._leased:
                raise RuntimeError(f"RPC client was already released: {lease.name}")
            self._leased.remove(lease.name)
            lease.future = None
            if not self._closed:
                # Prefer the already-connected client for the next serial call.
                # Other clients remain available for actual concurrency.
                self._available.appendleft(lease)

    def close(self, timeout_sec: float = 5.0) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            active = [client for client in self._clients if client.future is not None]

        for client in active:
            try:
                client.future.cancel()
            except BaseException:
                pass

        deadline = time.monotonic() + timeout_sec
        for client in active:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                client.future.result(timeout=remaining)
            except BaseException:
                pass

        for client in self._clients:
            client.rpc_client.close()


def create_rpc_client_pool(
    *,
    max_clients: int,
    client_name: Callable[[int], str],
    client_factory: Callable[[str], tuple[Any, Any]],
) -> RpcClientPool:
    clients: list[RpcClientLease] = []
    try:
        for index in range(max_clients):
            name = client_name(index)
            rpc_client, typed_client = client_factory(name)
            try:
                rpc_client.start()
            except BaseException:
                rpc_client.close()
                raise
            clients.append(RpcClientLease(name, rpc_client, typed_client))
    except BaseException:
        for client in clients:
            client.rpc_client.close()
        raise
    return RpcClientPool(clients)
