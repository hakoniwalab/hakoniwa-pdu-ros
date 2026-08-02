from __future__ import annotations

import threading
from concurrent.futures import Future

from hakoniwa_pdu_ros.service_call_lifecycle import (
    BridgeCallLifecycle,
    BridgeTimeoutError,
)


class LateSuccessFuture(Future[int]):
    def cancel(self) -> bool:
        # Model PDU-RPC's documented timeout/cancel race: cancellation is sent,
        # but a normal response may still win the native terminal transition.
        return False


def test_result_before_deadline_is_forwarded() -> None:
    rpc_future: Future[int] = Future()
    completed = threading.Event()
    results: list[int] = []
    errors: list[BaseException] = []
    BridgeCallLifecycle(
        rpc_future,
        timeout_msec=1000,
        on_result=lambda value: (results.append(value), completed.set()),
        on_error=lambda error: (errors.append(error), completed.set()),
    )

    rpc_future.set_result(42)

    assert completed.wait(1.0)
    assert results == [42]
    assert errors == []


def test_normal_result_after_bridge_deadline_is_rejected() -> None:
    rpc_future = LateSuccessFuture()
    completed = threading.Event()
    results: list[int] = []
    errors: list[BaseException] = []
    BridgeCallLifecycle(
        rpc_future,
        timeout_msec=10,
        on_result=lambda value: (results.append(value), completed.set()),
        on_error=lambda error: (errors.append(error), completed.set()),
    )

    assert not completed.wait(0.05)
    rpc_future.set_result(42)

    assert completed.wait(1.0)
    assert results == []
    assert len(errors) == 1
    assert isinstance(errors[0], BridgeTimeoutError)


def test_deadline_waits_for_rpc_terminal_completion() -> None:
    rpc_future = LateSuccessFuture()
    completed = threading.Event()
    BridgeCallLifecycle(
        rpc_future,
        timeout_msec=10,
        on_result=lambda _value: completed.set(),
        on_error=lambda _error: completed.set(),
    )

    assert not completed.wait(0.05)
    assert not rpc_future.done()
    rpc_future.set_exception(RuntimeError("native terminal cleanup finished"))
    assert completed.wait(1.0)
