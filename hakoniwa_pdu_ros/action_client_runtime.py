from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class ActionGoalSession:
    action_name: str
    goal: object
    events: queue.Queue


class ActionClientRuntime:
    """Serialize native Action polling and dispatch events to Goal-local queues."""

    def __init__(self, client: object, *, idle_sleep_sec: float = 0.001) -> None:
        self._client = client
        self._idle_sleep_sec = idle_sleep_sec
        self._lock = threading.Lock()
        self._sessions: dict[bytes, ActionGoalSession] = {}
        self._closed = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._client.start()
        self._thread = threading.Thread(
            target=self._pump,
            name="hakoniwa-action-client-pump",
            daemon=True,
        )
        self._thread.start()

    def is_running(self) -> bool:
        return bool(self._client.is_running())

    def submit_goal(
        self,
        action_name: str,
        pdu: bytes,
        goal_id: bytes,
        *,
        timeout_usec: int,
    ) -> tuple[ActionGoalSession, object]:
        events: queue.Queue = queue.Queue()
        provisional = ActionGoalSession(action_name, None, events)
        with self._lock:
            if goal_id in self._sessions:
                raise ValueError("duplicate Action Goal ID")
            self._sessions[goal_id] = provisional
        try:
            goal = self._client.send_goal(
                action_name,
                pdu,
                goal_id,
                timeout_usec=timeout_usec,
            )
            session = ActionGoalSession(action_name, goal, events)
            with self._lock:
                self._sessions[goal_id] = session
            response = self.wait_for(
                session,
                {"GOAL_RESPONSE", "TIMEOUT", "ERROR"},
                timeout_sec=max(timeout_usec / 1_000_000.0 + 1.0, 1.0),
            )
            return session, response
        except BaseException:
            self.release(goal_id)
            raise

    def cancel(self, session: ActionGoalSession, *, timeout_sec: float = 5.0):
        self._client.cancel_goal(session.action_name, session.goal)
        return self.wait_for(
            session,
            {"CANCEL_RESPONSE", "RESULT", "ERROR"},
            timeout_sec=timeout_sec,
        )

    def wait_for(
        self,
        session: ActionGoalSession,
        event_names: set[str],
        *,
        timeout_sec: float | None = None,
    ):
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
        deferred = []
        try:
            while True:
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    raise TimeoutError(
                        f"timed out waiting for Action event: {sorted(event_names)}"
                    )
                event = session.events.get(timeout=remaining)
                if event.event.name in event_names:
                    return event
                deferred.append(event)
        except queue.Empty as error:
            raise TimeoutError(
                f"timed out waiting for Action event: {sorted(event_names)}"
            ) from error
        finally:
            for event in deferred:
                session.events.put(event)

    def next_event(
        self, session: ActionGoalSession, *, timeout_sec: float | None = None
    ):
        try:
            return session.events.get(timeout=timeout_sec)
        except queue.Empty as error:
            raise TimeoutError("timed out waiting for Action event") from error

    def release(self, goal_id: bytes) -> None:
        with self._lock:
            self._sessions.pop(bytes(goal_id), None)

    def close(self) -> None:
        if self._closed.is_set():
            return
        self._closed.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            self._client.stop()
        finally:
            self._client.close()
        with self._lock:
            self._sessions.clear()

    def _pump(self) -> None:
        while not self._closed.is_set():
            try:
                event = self._client.poll()
            except BaseException as error:
                self._broadcast_error(error)
                return
            if event.event.name == "NONE":
                time.sleep(self._idle_sleep_sec)
                continue
            goal = getattr(event, "goal", None)
            if goal is None:
                continue
            goal_id = bytes(goal.goal_id)
            with self._lock:
                session = self._sessions.get(goal_id)
            if session is not None:
                session.events.put(event)

    def _broadcast_error(self, error: BaseException) -> None:
        with self._lock:
            sessions = tuple(self._sessions.values())
        for session in sessions:
            session.events.put(_PumpError(error))


class _PumpErrorEvent:
    name = "ERROR"


@dataclass(frozen=True)
class _PumpError:
    error: BaseException
    event: _PumpErrorEvent = _PumpErrorEvent()
