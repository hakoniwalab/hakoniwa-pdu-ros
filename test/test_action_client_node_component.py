from __future__ import annotations

from enum import IntEnum
import importlib
import sys
from types import ModuleType, SimpleNamespace


class _Logger:
    def __init__(self) -> None:
        self.messages = []

    def info(self, message) -> None:
        self.messages.append(("info", message))

    def warning(self, message) -> None:
        self.messages.append(("warning", message))

    def error(self, message) -> None:
        self.messages.append(("error", message))


class _Node:
    def get_logger(self):
        return self.logger


class _Future:
    def __init__(self) -> None:
        self._done = False
        self._result = None
        self._error = None
        self._callbacks = []

    def add_done_callback(self, callback) -> None:
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

    def complete(self, result) -> None:
        self._result = result
        self._done = True
        callbacks = self._callbacks
        self._callbacks = []
        for callback in callbacks:
            callback(self)

    def fail(self, error: BaseException) -> None:
        self._error = error
        self._done = True
        callbacks = self._callbacks
        self._callbacks = []
        for callback in callbacks:
            callback(self)


class _GoalStatus:
    STATUS_UNKNOWN = 0
    STATUS_ACCEPTED = 1
    STATUS_EXECUTING = 2
    STATUS_CANCELING = 3
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


class _ActionTerminalStatus(IntEnum):
    UNSPECIFIED = 0
    SUCCEEDED = 1
    CANCELED = 2
    ABORTED = 3


def _load_node_module(monkeypatch):
    rclpy = ModuleType("rclpy")
    action_msgs = ModuleType("action_msgs")
    action_msgs_msg = ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatus = _GoalStatus
    rclpy_action = ModuleType("rclpy.action")
    rclpy_action.ActionClient = object
    callback_groups = ModuleType("rclpy.callback_groups")
    callback_groups.ReentrantCallbackGroup = object
    executors = ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = object
    node = ModuleType("rclpy.node")
    node.Node = _Node
    rpc = ModuleType("hakoniwa_pdu_rpc")
    rpc.ActionTerminalStatus = _ActionTerminalStatus
    for name, module in (
        ("rclpy", rclpy),
        ("rclpy.action", rclpy_action),
        ("rclpy.callback_groups", callback_groups),
        ("rclpy.executors", executors),
        ("rclpy.node", node),
        ("action_msgs", action_msgs),
        ("action_msgs.msg", action_msgs_msg),
        ("hakoniwa_pdu_rpc", rpc),
    ):
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("hakoniwa_pdu_ros.action_client_node", None)
    return importlib.import_module("hakoniwa_pdu_ros.action_client_node")


class _TypedAction:
    def __init__(self) -> None:
        self.calls = []

    def accept_goal(self, goal) -> None:
        self.calls.append(("accept_goal", goal.goal_id))

    def reject_goal(self, goal) -> None:
        self.calls.append(("reject_goal", goal.goal_id))

    def accept_cancel(self, goal) -> None:
        self.calls.append(("accept_cancel", goal.goal_id))

    def reject_cancel(self, goal) -> None:
        self.calls.append(("reject_cancel", goal.goal_id))

    def create_feedback(self):
        return SimpleNamespace(partial_sequence=[])

    def send_feedback(self, goal, feedback) -> None:
        self.calls.append(
            ("feedback", goal.goal_id, list(feedback.partial_sequence))
        )

    def create_result(self):
        return SimpleNamespace(sequence=[])

    def complete(self, goal, status, result) -> None:
        self.calls.append(
            ("complete", goal.goal_id, status.name, list(result.sequence))
        )


class _Mapper:
    def goal_to_ros(self, goal):
        return SimpleNamespace(order=goal.order)

    def feedback_to_typed(self, feedback, target):
        target.partial_sequence = list(feedback.partial_sequence)
        return target

    def result_to_typed(self, result, target):
        target.sequence = list(result.sequence)
        return target


class _RosGoalHandle:
    def __init__(self, goal_id: bytes, accepted: bool = True) -> None:
        self.goal_id = SimpleNamespace(uuid=goal_id)
        self.accepted = accepted
        self.result_future = _Future()
        self.cancel_future = _Future()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


class _RosClient:
    def __init__(self) -> None:
        self.ready = True
        self.goal_future = None
        self.goal_futures = []
        self.feedback_callback = None
        self.feedback_callbacks = []
        self.sent_goal = None
        self.sent_goals = []
        self.destroyed = False

    def server_is_ready(self) -> bool:
        return self.ready

    def send_goal_async(self, goal, *, feedback_callback):
        self.sent_goal = goal
        self.sent_goals.append(goal)
        self.feedback_callback = feedback_callback
        self.feedback_callbacks.append(feedback_callback)
        self.goal_future = _Future()
        self.goal_futures.append(self.goal_future)
        return self.goal_future

    def destroy(self) -> None:
        self.destroyed = True


class _RawServer:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


def _make_node_and_runtime(monkeypatch):
    module = _load_node_module(monkeypatch)
    node = module.HakoniwaRosActionClientNode.__new__(
        module.HakoniwaRosActionClientNode
    )
    node.logger = _Logger()
    node._lock = module.Lock()
    node._contexts = {}
    typed_action = _TypedAction()
    ros_client = _RosClient()
    binding = SimpleNamespace(
        hakoniwa_action="fibonacci",
        ros_name="/fibonacci",
    )
    runtime = module._BindingRuntime(
        binding,
        _Mapper(),
        typed_action,
        ros_client,
    )
    node._bindings = {"fibonacci": runtime}
    return module, node, runtime, typed_action, ros_client


def _goal_event(
    goal_id: bytes,
    order: int = 10,
    action_name: str = "fibonacci",
):
    return SimpleNamespace(
        event=SimpleNamespace(name="GOAL_REQUEST"),
        action_name=action_name,
        goal=SimpleNamespace(goal_id=goal_id),
        goal_body=SimpleNamespace(order=order),
    )


def test_goal_accept_feedback_and_succeeded_result(monkeypatch) -> None:
    module, node, runtime, typed_action, ros_client = _make_node_and_runtime(
        monkeypatch
    )
    goal_id = bytes(range(1, 17))

    node._handle_goal_request(runtime, _goal_event(goal_id))
    assert ros_client.sent_goal.order == 10

    ros_handle = _RosGoalHandle(bytes(range(17, 33)))
    ros_client.goal_future.complete(ros_handle)
    assert typed_action.calls == [("accept_goal", goal_id)]

    ros_client.feedback_callback(
        SimpleNamespace(
            feedback=SimpleNamespace(partial_sequence=[0, 1, 1, 2])
        )
    )
    ros_handle.result_future.complete(
        SimpleNamespace(
            status=_GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(sequence=[0, 1, 1, 2, 3]),
        )
    )

    assert typed_action.calls == [
        ("accept_goal", goal_id),
        ("feedback", goal_id, [0, 1, 1, 2]),
        ("complete", goal_id, "SUCCEEDED", [0, 1, 1, 2, 3]),
    ]
    assert node._contexts == {}


def test_ros_goal_rejection_rejects_hakoniwa_goal(monkeypatch) -> None:
    _, node, runtime, typed_action, ros_client = _make_node_and_runtime(
        monkeypatch
    )
    goal_id = bytes([0x21]) * 16

    node._handle_goal_request(runtime, _goal_event(goal_id))
    ros_client.goal_future.complete(
        _RosGoalHandle(bytes([0x31]) * 16, accepted=False)
    )

    assert typed_action.calls == [("reject_goal", goal_id)]
    assert node._contexts == {}


def test_canceled_result_waits_for_cancel_accept_response(monkeypatch) -> None:
    _, node, runtime, typed_action, ros_client = _make_node_and_runtime(
        monkeypatch
    )
    goal_id = bytes([0x41]) * 16
    ros_goal_id = bytes([0x51]) * 16

    node._handle_goal_request(runtime, _goal_event(goal_id))
    ros_handle = _RosGoalHandle(ros_goal_id)
    ros_client.goal_future.complete(ros_handle)
    node._handle_cancel_request(runtime, SimpleNamespace(goal_id=goal_id))

    ros_handle.result_future.complete(
        SimpleNamespace(
            status=_GoalStatus.STATUS_CANCELED,
            result=SimpleNamespace(sequence=[0, 1]),
        )
    )
    assert [call[0] for call in typed_action.calls] == ["accept_goal"]

    ros_handle.cancel_future.complete(
        SimpleNamespace(
            goals_canceling=[
                SimpleNamespace(goal_id=SimpleNamespace(uuid=ros_goal_id))
            ]
        )
    )

    assert typed_action.calls == [
        ("accept_goal", goal_id),
        ("accept_cancel", goal_id),
        ("complete", goal_id, "CANCELED", [0, 1]),
    ]


def test_cancel_reject_keeps_goal_available_for_result(monkeypatch) -> None:
    _, node, runtime, typed_action, ros_client = _make_node_and_runtime(
        monkeypatch
    )
    goal_id = bytes([0x61]) * 16

    node._handle_goal_request(runtime, _goal_event(goal_id))
    ros_handle = _RosGoalHandle(bytes([0x71]) * 16)
    ros_client.goal_future.complete(ros_handle)
    node._handle_cancel_request(runtime, SimpleNamespace(goal_id=goal_id))
    ros_handle.cancel_future.complete(SimpleNamespace(goals_canceling=[]))
    ros_handle.result_future.complete(
        SimpleNamespace(
            status=_GoalStatus.STATUS_ABORTED,
            result=SimpleNamespace(sequence=[0, 1, 1]),
        )
    )

    assert typed_action.calls == [
        ("accept_goal", goal_id),
        ("reject_cancel", goal_id),
        ("complete", goal_id, "ABORTED", [0, 1, 1]),
    ]


def test_unavailable_ros_server_rejects_goal_without_context(monkeypatch) -> None:
    _, node, runtime, typed_action, ros_client = _make_node_and_runtime(
        monkeypatch
    )
    ros_client.ready = False
    goal_id = bytes([0x81]) * 16

    node._handle_goal_request(runtime, _goal_event(goal_id))

    assert typed_action.calls == [("reject_goal", goal_id)]
    assert node._contexts == {}


def test_early_result_failure_waits_for_goal_accept_response(monkeypatch) -> None:
    _, node, runtime, typed_action, ros_client = _make_node_and_runtime(
        monkeypatch
    )
    goal_id = bytes([0x91]) * 16
    ros_handle = _RosGoalHandle(bytes([0xA1]) * 16)
    ros_handle.result_future.fail(RuntimeError("result transport failed"))

    node._handle_goal_request(runtime, _goal_event(goal_id))
    ros_client.goal_future.complete(ros_handle)

    assert typed_action.calls == [
        ("accept_goal", goal_id),
        ("complete", goal_id, "ABORTED", []),
    ]
    assert node._contexts == {}
    assert any(
        level == "error" and "result transport failed" in message
        for level, message in node.logger.messages
    )


def test_two_goals_keep_independent_contexts(monkeypatch) -> None:
    _, node, runtime, typed_action, ros_client = _make_node_and_runtime(
        monkeypatch
    )
    first_id = bytes([0xB1]) * 16
    second_id = bytes([0xB2]) * 16

    node._handle_goal_request(runtime, _goal_event(first_id, order=5))
    node._handle_goal_request(runtime, _goal_event(second_id, order=7))

    first_handle = _RosGoalHandle(bytes([0xC1]) * 16)
    second_handle = _RosGoalHandle(bytes([0xC2]) * 16)
    ros_client.goal_futures[0].complete(first_handle)
    ros_client.goal_futures[1].complete(second_handle)

    assert typed_action.calls == [
        ("accept_goal", first_id),
        ("accept_goal", second_id),
    ]
    assert set(node._contexts) == {
        ("fibonacci", first_id),
        ("fibonacci", second_id),
    }

    second_handle.result_future.complete(
        SimpleNamespace(
            status=_GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(sequence=[0, 1, 1, 2, 3, 5, 8]),
        )
    )
    first_handle.result_future.complete(
        SimpleNamespace(
            status=_GoalStatus.STATUS_ABORTED,
            result=SimpleNamespace(sequence=[0, 1, 1, 2, 3]),
        )
    )

    assert typed_action.calls[-2:] == [
        ("complete", second_id, "SUCCEEDED", [0, 1, 1, 2, 3, 5, 8]),
        ("complete", first_id, "ABORTED", [0, 1, 1, 2, 3]),
    ]
    assert node._contexts == {}


def test_multiple_action_bindings_route_to_their_own_runtime(monkeypatch) -> None:
    module, node, first_runtime, first_action, first_client = (
        _make_node_and_runtime(monkeypatch)
    )
    second_action = _TypedAction()
    second_client = _RosClient()
    second_runtime = module._BindingRuntime(
        SimpleNamespace(
            hakoniwa_action="fibonacci_alt",
            ros_name="/fibonacci_alt",
        ),
        _Mapper(),
        second_action,
        second_client,
    )
    node._bindings["fibonacci_alt"] = second_runtime
    first_id = bytes([0xD1]) * 16
    second_id = bytes([0xD2]) * 16

    node._dispatch_event(_goal_event(first_id, action_name="fibonacci"))
    node._dispatch_event(
        _goal_event(second_id, action_name="fibonacci_alt")
    )
    first_client.goal_future.complete(_RosGoalHandle(bytes([0xE1]) * 16))
    second_client.goal_future.complete(_RosGoalHandle(bytes([0xE2]) * 16))

    assert first_action.calls == [("accept_goal", first_id)]
    assert second_action.calls == [("accept_goal", second_id)]
    assert set(node._contexts) == {
        ("fibonacci", first_id),
        ("fibonacci_alt", second_id),
    }
    assert first_runtime is node._bindings["fibonacci"]


def test_close_runtime_cancels_active_goals_and_closes_resources(
    monkeypatch,
) -> None:
    module, node, runtime, _typed_action, ros_client = _make_node_and_runtime(
        monkeypatch
    )
    goal_id = bytes([0xF1]) * 16
    ros_handle = _RosGoalHandle(bytes([0xF2]) * 16)
    context = module._GoalContext(
        "fibonacci",
        SimpleNamespace(goal_id=goal_id),
        runtime,
        ros_goal_handle=ros_handle,
        hakoniwa_accepted=True,
    )
    raw_server = _RawServer()
    node._contexts[("fibonacci", goal_id)] = context
    node._closed = False
    node._stop_requested = module.Event()
    node._poll_thread = None
    node._ros_clients = [ros_client]
    node._raw_servers = [raw_server]
    node._typed_servers = [object()]

    node._close_runtime()

    assert context.terminal
    assert node._contexts == {}
    assert ros_handle.cancel_calls == 1
    assert ros_client.destroyed
    assert raw_server.closed
    assert node._raw_servers == []
    assert node._typed_servers == []
