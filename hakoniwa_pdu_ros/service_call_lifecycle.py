from __future__ import annotations

import threading
from typing import Any, Callable


class BridgeTimeoutError(TimeoutError):
    """The ROS-to-RPC bridge deadline expired before RPC completion."""


class BridgeCallLifecycle:
    """Apply a bridge-owned deadline to one asynchronous RPC call.

    The bridge is the sole deadline owner; callers must start the underlying
    PDU-RPC call with ``timeout_usec=0`` (infinite wait).  The RPC future remains
    the authority for terminal cleanup.  Expiration asks it to cancel, but
    completion is not reported until that future becomes terminal.  A normal
    result arriving after expiration is deliberately converted into
    ``BridgeTimeoutError``.
    """

    def __init__(
        self,
        rpc_future: Any,
        *,
        timeout_msec: int,
        on_result: Callable[[Any], None],
        on_error: Callable[[BaseException], None],
    ) -> None:
        self._rpc_future = rpc_future
        self._on_result = on_result
        self._on_error = on_error
        self._lock = threading.Lock()
        self._expired = False
        self._completed = False
        self._timer = threading.Timer(timeout_msec / 1000.0, self._expire)
        self._timer.daemon = True

        rpc_future.add_done_callback(self._complete)
        self._timer.start()

    def _expire(self) -> None:
        with self._lock:
            if self._completed:
                return
            self._expired = True
        try:
            self._rpc_future.cancel()
        except BaseException:
            # The terminal RPC callback still owns final completion and cleanup.
            pass

    def _complete(self, future: Any) -> None:
        with self._lock:
            if self._completed:
                return
            self._completed = True
            expired = self._expired
        self._timer.cancel()

        if expired:
            self._on_error(BridgeTimeoutError("Bridge RPC deadline expired"))
            return
        try:
            self._on_result(future.result())
        except BaseException as error:
            self._on_error(error)
