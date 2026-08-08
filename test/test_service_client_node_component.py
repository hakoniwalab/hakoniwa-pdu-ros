from __future__ import annotations

from enum import IntEnum
import importlib
import sys
import time
from types import ModuleType, SimpleNamespace


class _Logger:
    def __init__(self) -> None:
        self.errors: list[str] = []

    def error(self, message: str) -> None:
        self.errors.append(message)


class _Node:
    def get_logger(self):
        return self.logger


class _Future:
    def __init__(self) -> None:
        self._callbacks = []
        self._result = None
        self._error: BaseException | None = None
        self.cancel_calls = 0

    def add_done_callback(self, callback) -> None:
        self._callbacks.append(callback)

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

    def complete(self, result) -> None:
        self._result = result
        for callback in list(self._callbacks):
            callback(self)

    def fail(self, error: BaseException) -> None:
        self._error = error
        for callback in list(self._callbacks):
            callback(self)

    def cancel(self) -> bool:
        self.cancel_calls += 1
        return True


class _RpcServiceResultCode(IntEnum):
    OK = 0
    ERROR = 1
    CANCELED = 2
    INVALID = 3
    NOT_SUPPORTED = 4


class _RosService:
    class Request:
        def __init__(self) -> None:
            self.a = 0
            self.b = 0


class _RosClient:
    def __init__(self, *, ready: bool = True) -> None:
        self.ready = ready
        self.requests = []
        self.futures: list[_Future] = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request) -> _Future:
        self.requests.append(request)
        future = _Future()
        self.futures.append(future)
        return future


class _TypedService:
    def __init__(self) -> None:
        self.calls = []

    def create_response(self):
        return SimpleNamespace(sum=0)

    def send_reply(self, request, response) -> None:
        self.calls.append(("reply", request.request_token, response.sum))

    def send_error(self, request, result_code) -> None:
        self.calls.append(("error", request.request_token, result_code.name))

    def send_cancel_reply(self, request) -> None:
        self.calls.append(("cancel", request.request_token))


def _load_node_module(monkeypatch):
    rclpy = ModuleType("rclpy")
    callback_groups = ModuleType("rclpy.callback_groups")
    callback_groups.ReentrantCallbackGroup = object
    executors = ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = object
    node = ModuleType("rclpy.node")
    node.Node = _Node
    rpc = ModuleType("hakoniwa_pdu_rpc")
    rpc.RpcServiceResultCode = _RpcServiceResultCode
    for name, module in (
        ("rclpy", rclpy),
        ("rclpy.callback_groups", callback_groups),
        ("rclpy.executors", executors),
        ("rclpy.node", node),
        ("hakoniwa_pdu_rpc", rpc),
    ):
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("hakoniwa_pdu_ros.service_client_node", None)
    module = importlib.import_module("hakoniwa_pdu_ros.service_client_node")

    def copy_fields(source, target) -> None:
        for name, value in vars(source).items():
            if hasattr(target, name):
                setattr(target, name, value)

    monkeypatch.setattr(module, "copy_matching_fields", copy_fields)
    return module


def _runtime(module, service_name: str, *, ready: bool = True, timeout: int = 1000):
    typed_service = _TypedService()
    ros_client = _RosClient(ready=ready)
    binding = SimpleNamespace(
        hakoniwa_service=service_name,
        ros_name=f"/{service_name.lower()}",
        timeout_msec=timeout,
    )
    runtime = module._BindingRuntime(
        binding,
        SimpleNamespace(pdu_service_type=f"test_msgs/{service_name}"),
        typed_service,
        ros_client,
        _RosService,
    )
    return runtime, typed_service, ros_client


def _make_node(monkeypatch, *runtimes):
    module = _load_node_module(monkeypatch)
    node = module.HakoniwaRosServiceClientNode.__new__(
        module.HakoniwaRosServiceClientNode
    )
    node.logger = _Logger()
    node._lock = module.Lock()
    node._contexts = {}
    node._bindings = {
        runtime.binding.hakoniwa_service: runtime for runtime in runtimes
    }
    return module, node


def _request(service_name: str, token: int, left: int = 20, right: int = 22):
    return SimpleNamespace(
        service_name=service_name,
        request_token=token,
        request_body=SimpleNamespace(a=left, b=right),
    )


def test_request_is_forwarded_and_ros_response_is_replied(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    runtime, typed_service, ros_client = _runtime(module, "Add")
    _, node = _make_node(monkeypatch, runtime)
    request = _request("Add", 10)

    node._handle_request(0, request)
    assert vars(ros_client.requests[0]) == {"a": 20, "b": 22}
    ros_client.futures[0].complete(SimpleNamespace(sum=42))

    assert typed_service.calls == [("reply", 10, 42)]
    assert node._contexts == {}


def test_unavailable_service_returns_not_supported(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    runtime, typed_service, _ = _runtime(module, "Add", ready=False)
    _, node = _make_node(monkeypatch, runtime)

    node._handle_request(0, _request("Add", 11))

    assert typed_service.calls == [("error", 11, "NOT_SUPPORTED")]
    assert node._contexts == {}


def test_request_conversion_failure_returns_invalid(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    runtime, typed_service, ros_client = _runtime(module, "Add")
    module, node = _make_node(monkeypatch, runtime)
    monkeypatch.setattr(
        module,
        "copy_matching_fields",
        lambda _source, _target: (_ for _ in ()).throw(ValueError("bad request")),
    )

    node._handle_request(0, _request("Add", 12))

    assert typed_service.calls == [("error", 12, "INVALID")]
    assert ros_client.requests == []


def test_ros_future_failure_returns_error(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    runtime, typed_service, ros_client = _runtime(module, "Add")
    _, node = _make_node(monkeypatch, runtime)

    node._handle_request(0, _request("Add", 13))
    ros_client.futures[0].fail(RuntimeError("ROS failure"))

    assert typed_service.calls == [("error", 13, "ERROR")]
    assert node._contexts == {}


def test_multiple_services_keep_request_routing_separate(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    add, add_typed, add_ros = _runtime(module, "Add")
    reset, reset_typed, reset_ros = _runtime(module, "Reset")
    _, node = _make_node(monkeypatch, add, reset)

    node._handle_request(0, _request("Add", 20, 1, 2))
    node._handle_request(0, _request("Reset", 21, 3, 4))
    reset_ros.futures[0].complete(SimpleNamespace(sum=7))
    add_ros.futures[0].complete(SimpleNamespace(sum=3))

    assert add_typed.calls == [("reply", 20, 3)]
    assert reset_typed.calls == [("reply", 21, 7)]
    assert node._contexts == {}


def test_cancel_wins_and_late_ros_response_is_ignored(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    runtime, typed_service, ros_client = _runtime(module, "Add")
    _, node = _make_node(monkeypatch, runtime)
    request = _request("Add", 30)

    node._handle_request(0, request)
    node._handle_cancel(0, request)
    ros_client.futures[0].complete(SimpleNamespace(sum=42))

    assert typed_service.calls == [("cancel", 30)]
    assert ros_client.futures[0].cancel_calls == 1
    assert node._contexts == {}


def test_timeout_returns_error_and_late_ros_response_is_ignored(monkeypatch) -> None:
    module = _load_node_module(monkeypatch)
    runtime, typed_service, ros_client = _runtime(module, "Add", timeout=1)
    _, node = _make_node(monkeypatch, runtime)
    request = _request("Add", 31)

    node._handle_request(0, request)
    context = node._contexts[(0, 31)]
    context.started_at = time.monotonic() - 1.0
    node._expire_calls()
    ros_client.futures[0].complete(SimpleNamespace(sum=42))

    assert typed_service.calls == [("error", 31, "ERROR")]
    assert ros_client.futures[0].cancel_calls == 1
    assert node._contexts == {}
