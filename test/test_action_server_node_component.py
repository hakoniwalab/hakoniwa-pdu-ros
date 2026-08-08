from __future__ import annotations

import importlib
import sys
from threading import Thread
import time
from types import ModuleType, SimpleNamespace

from hakoniwa_pdu_ros.action_goal_context import ActionGoalContextRegistry


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
    executor = None

    def get_logger(self):
        return self.logger


class _Future:
    def __init__(self, *, executor=None) -> None:
        self.executor = executor
        self.result_value = None

    def set_result(self, value) -> None:
        self.result_value = value


class _RosGoalHandle:
    def __init__(self, goal_id: bytes) -> None:
        self.goal_id = SimpleNamespace(uuid=goal_id)
        self.executed = False
        self.feedback = []
        self.terminal = None
        self.is_cancel_requested = False

    def execute(self) -> None:
        self.executed = True

    def publish_feedback(self, value) -> None:
        self.feedback.append(value)

    def succeed(self) -> None:
        self.terminal = "succeeded"

    def canceled(self) -> None:
        self.terminal = "canceled"

    def abort(self) -> None:
        self.terminal = "aborted"


def _load_node_module(monkeypatch):
    rclpy = ModuleType("rclpy")
    rclpy_action = ModuleType("rclpy.action")
    rclpy_action.ActionServer = object
    rclpy_action.GoalResponse = SimpleNamespace(ACCEPT="accept", REJECT="reject")
    rclpy_action.CancelResponse = SimpleNamespace(ACCEPT="accept", REJECT="reject")
    callback_groups = ModuleType("rclpy.callback_groups")
    callback_groups.MutuallyExclusiveCallbackGroup = object
    executors = ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = object
    node = ModuleType("rclpy.node")
    node.Node = _Node
    task = ModuleType("rclpy.task")
    task.Future = _Future
    for name, module in (
        ("rclpy", rclpy),
        ("rclpy.action", rclpy_action),
        ("rclpy.callback_groups", callback_groups),
        ("rclpy.executors", executors),
        ("rclpy.node", node),
        ("rclpy.task", task),
    ):
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("hakoniwa_pdu_ros.action_server_node", None)
    return importlib.import_module("hakoniwa_pdu_ros.action_server_node")


def test_goal_accept_feedback_and_result_use_explicit_id_mapping(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    node = module.HakoniwaRosActionServerNode.__new__(
        module.HakoniwaRosActionServerNode
    )
    node.logger = _Logger()
    node._contexts = ActionGoalContextRegistry()
    node._lock = module.Lock()
    node._goal_decisions = {}
    node._cancel_decisions = {}
    node._completion = {}
    node._early_events = module.defaultdict(list)
    node._deferred_cancel_results = {}
    node._bindings = {}

    sent = {}
    hakoniwa_handle = SimpleNamespace(goal_id=None)

    class TypedAction:
        def create_goal(self):
            return SimpleNamespace(order=0)

        def send_goal(self, goal, goal_id, timeout_usec):
            sent.update(goal=goal, goal_id=goal_id, timeout_usec=timeout_usec)
            hakoniwa_handle.goal_id = goal_id
            return hakoniwa_handle

    class Mapper:
        ros_result_type = SimpleNamespace

        def goal_to_typed(self, source, target):
            target.order = source.order

        def feedback_to_ros(self, value):
            return SimpleNamespace(sequence=value.sequence)

        def result_to_ros(self, value):
            return SimpleNamespace(sequence=value.sequence)

    binding = SimpleNamespace(
        hakoniwa_action="fibonacci",
        goal_response_timeout_msec=1000,
    )
    runtime = module._BindingRuntime(binding, Mapper(), TypedAction())
    node._bindings["fibonacci"] = runtime
    callback = node._make_goal_callback(runtime)
    outcome = {}
    callback_thread = Thread(
        target=lambda: outcome.setdefault(
            "response", callback(SimpleNamespace(order=10))
        )
    )
    callback_thread.start()

    deadline = time.monotonic() + 1.0
    while not node._goal_decisions and time.monotonic() < deadline:
        time.sleep(0.001)
    goal_id = next(iter(node._goal_decisions))
    node._dispatch_event(
        SimpleNamespace(
            event=SimpleNamespace(name="GOAL_RESPONSE"),
            action_name="fibonacci",
            goal=SimpleNamespace(goal_id=goal_id),
            decision=SimpleNamespace(name="ACCEPTED"),
        )
    )
    callback_thread.join(timeout=1.0)

    assert outcome["response"] == "accept"
    assert sent["goal"].order == 10
    assert sent["goal_id"] == goal_id
    assert len(goal_id) == 16 and any(goal_id)

    ros_goal_id = bytes([0xA5]) * 16
    ros_handle = _RosGoalHandle(ros_goal_id)
    node._make_accepted_callback(runtime)(ros_handle)
    context = node._contexts.find_by_ros(ros_goal_id)
    assert context is not None
    assert context.hakoniwa_goal_id == goal_id
    assert ros_handle.executed

    node._dispatch_event(
        SimpleNamespace(
            event=SimpleNamespace(name="FEEDBACK"),
            action_name="fibonacci",
            goal=SimpleNamespace(goal_id=goal_id),
            feedback=SimpleNamespace(sequence=[1, 1, 2]),
        )
    )
    node._dispatch_event(
        SimpleNamespace(
            event=SimpleNamespace(name="RESULT"),
            action_name="fibonacci",
            goal=SimpleNamespace(goal_id=goal_id),
            result=SimpleNamespace(sequence=[0, 1, 1, 2, 3]),
            terminal_status=SimpleNamespace(name="SUCCEEDED"),
        )
    )

    assert ros_handle.feedback[0].sequence == [1, 1, 2]
    assert ros_handle.terminal == "succeeded"
    assert node._completion[goal_id].result_value.sequence == [0, 1, 1, 2, 3]


def _make_cancel_test_node(module):
    node = module.HakoniwaRosActionServerNode.__new__(
        module.HakoniwaRosActionServerNode
    )
    node.logger = _Logger()
    node._contexts = ActionGoalContextRegistry()
    node._lock = module.Lock()
    node._goal_decisions = {}
    node._cancel_decisions = {}
    node._completion = {}
    node._early_events = module.defaultdict(list)
    node._deferred_cancel_results = {}
    node._bindings = {}
    node._stop_requested = module.Event()
    return node


def _accepted_cancel_context(node, runtime, goal_id: bytes, ros_goal_id: bytes):
    hakoniwa_handle = SimpleNamespace(goal_id=goal_id)
    node._contexts.register_hakoniwa_accepted(
        "fibonacci", goal_id, hakoniwa_handle
    )
    ros_handle = _RosGoalHandle(ros_goal_id)
    node._contexts.bind_ros_accepted("fibonacci", ros_handle)
    return hakoniwa_handle, ros_handle


def test_cancel_accept_is_correlated_by_hakoniwa_goal_id(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    node = _make_cancel_test_node(module)
    canceled = []

    class TypedAction:
        def cancel_goal(self, goal):
            canceled.append(goal)

    runtime = module._BindingRuntime(
        SimpleNamespace(hakoniwa_action="fibonacci"),
        object(),
        TypedAction(),
    )
    goal_id = bytes(range(1, 17))
    hakoniwa_handle, ros_handle = _accepted_cancel_context(
        node, runtime, goal_id, bytes([0x41]) * 16
    )
    callback = node._make_cancel_callback(runtime)
    outcome = {}
    thread = Thread(
        target=lambda: outcome.setdefault("response", callback(ros_handle))
    )
    thread.start()

    deadline = time.monotonic() + 1.0
    while goal_id not in node._cancel_decisions and time.monotonic() < deadline:
        time.sleep(0.001)
    node._dispatch_event(
        SimpleNamespace(
            event=SimpleNamespace(name="CANCEL_RESPONSE"),
            action_name="fibonacci",
            goal=SimpleNamespace(goal_id=goal_id),
            decision=SimpleNamespace(name="ACCEPTED"),
        )
    )
    thread.join(timeout=1.0)

    assert outcome["response"] == "accept"
    assert canceled == [hakoniwa_handle]
    assert node._contexts.find_by_hakoniwa(goal_id) is not None


def test_cancel_reject_keeps_goal_context(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    node = _make_cancel_test_node(module)
    runtime = module._BindingRuntime(
        SimpleNamespace(hakoniwa_action="fibonacci"),
        object(),
        SimpleNamespace(cancel_goal=lambda _goal: None),
    )
    goal_id = bytes(range(1, 17))
    _, ros_handle = _accepted_cancel_context(
        node, runtime, goal_id, bytes([0x42]) * 16
    )
    callback = node._make_cancel_callback(runtime)
    outcome = {}
    thread = Thread(
        target=lambda: outcome.setdefault("response", callback(ros_handle))
    )
    thread.start()

    deadline = time.monotonic() + 1.0
    while goal_id not in node._cancel_decisions and time.monotonic() < deadline:
        time.sleep(0.001)
    node._dispatch_event(
        SimpleNamespace(
            event=SimpleNamespace(name="CANCEL_RESPONSE"),
            action_name="fibonacci",
            goal=SimpleNamespace(goal_id=goal_id),
            decision=SimpleNamespace(name="REJECTED"),
        )
    )
    thread.join(timeout=1.0)

    assert outcome["response"] == "reject"
    assert node._contexts.find_by_hakoniwa(goal_id) is not None


def test_result_wins_over_pending_cancel(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    node = _make_cancel_test_node(module)

    class Mapper:
        def result_to_ros(self, value):
            return value

    runtime = module._BindingRuntime(
        SimpleNamespace(hakoniwa_action="fibonacci"),
        Mapper(),
        SimpleNamespace(cancel_goal=lambda _goal: None),
    )
    node._bindings["fibonacci"] = runtime
    goal_id = bytes(range(1, 17))
    _, ros_handle = _accepted_cancel_context(
        node, runtime, goal_id, bytes([0x43]) * 16
    )
    node._completion[goal_id] = _Future()
    callback = node._make_cancel_callback(runtime)
    outcome = {}
    thread = Thread(
        target=lambda: outcome.setdefault("response", callback(ros_handle))
    )
    thread.start()

    deadline = time.monotonic() + 1.0
    while goal_id not in node._cancel_decisions and time.monotonic() < deadline:
        time.sleep(0.001)
    result = SimpleNamespace(sequence=[0, 1, 1, 2, 3])
    node._dispatch_event(
        SimpleNamespace(
            event=SimpleNamespace(name="RESULT"),
            action_name="fibonacci",
            goal=SimpleNamespace(goal_id=goal_id),
            result=result,
            terminal_status=SimpleNamespace(name="SUCCEEDED"),
        )
    )
    thread.join(timeout=1.0)

    assert outcome["response"] == "reject"
    assert ros_handle.terminal == "succeeded"
    assert node._completion[goal_id].result_value is result


def test_canceled_result_waits_for_ros_canceling_transition(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    node = _make_cancel_test_node(module)

    class Mapper:
        def result_to_ros(self, value):
            return value

    runtime = module._BindingRuntime(
        SimpleNamespace(hakoniwa_action="fibonacci"),
        Mapper(),
        object(),
    )
    node._bindings["fibonacci"] = runtime
    goal_id = bytes(range(1, 17))
    _, ros_handle = _accepted_cancel_context(
        node, runtime, goal_id, bytes([0x46]) * 16
    )
    node._completion[goal_id] = _Future()
    result = SimpleNamespace(sequence=[0, 1, 1, 2, 3])
    event = SimpleNamespace(
        event=SimpleNamespace(name="RESULT"),
        action_name="fibonacci",
        goal=SimpleNamespace(goal_id=goal_id),
        result=result,
        terminal_status=SimpleNamespace(name="CANCELED"),
    )

    node._dispatch_event(event)

    assert ros_handle.terminal is None
    assert node._completion[goal_id].result_value is None
    assert node._deferred_cancel_results[goal_id] is event

    ros_handle.is_cancel_requested = True
    node._flush_deferred_cancel_results()

    assert ros_handle.terminal == "canceled"
    assert node._completion[goal_id].result_value is result
    assert goal_id not in node._deferred_cancel_results


def test_cancel_send_failure_rejects_without_losing_context(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    node = _make_cancel_test_node(module)

    def fail_cancel(_goal):
        raise RuntimeError("send failed")

    runtime = module._BindingRuntime(
        SimpleNamespace(hakoniwa_action="fibonacci"),
        object(),
        SimpleNamespace(cancel_goal=fail_cancel),
    )
    goal_id = bytes(range(1, 17))
    _, ros_handle = _accepted_cancel_context(
        node, runtime, goal_id, bytes([0x44]) * 16
    )

    response = node._make_cancel_callback(runtime)(ros_handle)

    assert response == "reject"
    assert goal_id not in node._cancel_decisions
    assert node._contexts.find_by_hakoniwa(goal_id) is not None
    assert any(
        level == "error" and "Cancel send failed" in message
        for level, message in node.logger.messages
    )


def test_shutdown_releases_pending_cancel_wait(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    node = _make_cancel_test_node(module)
    runtime = module._BindingRuntime(
        SimpleNamespace(hakoniwa_action="fibonacci"),
        object(),
        SimpleNamespace(cancel_goal=lambda _goal: None),
    )
    goal_id = bytes(range(1, 17))
    _, ros_handle = _accepted_cancel_context(
        node, runtime, goal_id, bytes([0x45]) * 16
    )
    callback = node._make_cancel_callback(runtime)
    outcome = {}
    thread = Thread(
        target=lambda: outcome.setdefault("response", callback(ros_handle))
    )
    thread.start()

    deadline = time.monotonic() + 1.0
    while goal_id not in node._cancel_decisions and time.monotonic() < deadline:
        time.sleep(0.001)
    node._stop_requested.set()
    node._cancel_decisions[goal_id].completed.set()
    thread.join(timeout=1.0)

    assert outcome["response"] == "reject"
    assert not thread.is_alive()
