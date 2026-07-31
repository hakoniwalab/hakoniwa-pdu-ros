import importlib
import sys
import types
from enum import Enum

from hakoniwa_pdu_ros.config_loader import BindingConfig, PduKeyConfig, QosConfig


class _Policy(Enum):
    KEEP_LAST = 1
    KEEP_ALL = 2
    RELIABLE = 3
    BEST_EFFORT = 4
    VOLATILE = 5
    TRANSIENT_LOCAL = 6


class _QoSProfile:
    def __init__(self, **kwargs: object) -> None:
        self.__dict__.update(kwargs)


class _SubscriptionEventCallbacks:
    def __init__(self, *, incompatible_qos) -> None:
        self.incompatible_qos = incompatible_qos


class _Logger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


def test_subscription_uses_resolved_qos_and_diagnostic_callback(monkeypatch) -> None:
    rclpy = types.ModuleType("rclpy")
    rclpy_event_handler = types.ModuleType("rclpy.event_handler")
    rclpy_event_handler.SubscriptionEventCallbacks = _SubscriptionEventCallbacks
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.HistoryPolicy = _Policy
    rclpy_qos.ReliabilityPolicy = _Policy
    rclpy_qos.DurabilityPolicy = _Policy
    rclpy_qos.QoSProfile = _QoSProfile

    endpoint_module = types.ModuleType("hakoniwa_pdu_ros.pdu_endpoint")
    endpoint_module.PduEndpointManager = object
    mapper_module = types.ModuleType("hakoniwa_pdu_ros.type_mapper")
    mapper_module.import_ros_msg_class = lambda _: object
    mapper_module.pdu_bytes_to_ros_msg = lambda *_: object()
    mapper_module.ros_msg_to_pdu_bytes = lambda *_: b"pdu"
    mapper_module.validate_pdu_converter = lambda _: None
    zenoh_module = types.ModuleType("hakoniwa_pdu_ros.zenoh_io")
    zenoh_module.validate_zenoh_io_for_config = lambda _: None

    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.event_handler", rclpy_event_handler)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)
    monkeypatch.setitem(sys.modules, "rclpy.qos", rclpy_qos)
    monkeypatch.setitem(
        sys.modules,
        "hakoniwa_pdu_ros.pdu_endpoint",
        endpoint_module,
    )
    monkeypatch.setitem(
        sys.modules,
        "hakoniwa_pdu_ros.type_mapper",
        mapper_module,
    )
    monkeypatch.setitem(
        sys.modules,
        "hakoniwa_pdu_ros.zenoh_io",
        zenoh_module,
    )
    monkeypatch.delitem(
        sys.modules,
        "hakoniwa_pdu_ros.bridge_node",
        raising=False,
    )
    bridge_node = importlib.import_module("hakoniwa_pdu_ros.bridge_node")

    node = bridge_node.HakoniwaRosBridgeNode.__new__(
        bridge_node.HakoniwaRosBridgeNode
    )
    logger = _Logger()
    node._manager = types.SimpleNamespace(send=lambda *_: None)
    node._subscriptions = []
    node.get_logger = lambda: logger
    captured: dict[str, object] = {}

    def _create_subscription(
        msg_cls,
        topic,
        callback,
        qos_profile,
        *,
        event_callbacks,
    ):
        captured.update(
            msg_cls=msg_cls,
            topic=topic,
            callback=callback,
            qos_profile=qos_profile,
            event_callbacks=event_callbacks,
        )
        return object()

    node.create_subscription = _create_subscription
    binding = BindingConfig(
        direction="ros_to_pdu",
        pdu_key=PduKeyConfig(robot_name="Tobas", pdu_name="joint_states"),
        topic="/joint_states",
        channel_id=1,
        pdu_size=128,
        pdu_type="sensor_msgs/JointState",
        qos=QosConfig(reliability="best_effort"),
    )

    node._setup_out_binding(binding)

    profile = captured["qos_profile"]
    assert profile.reliability is _Policy.BEST_EFFORT
    assert profile.history is _Policy.KEEP_LAST
    assert captured["topic"] == "/joint_states"
    assert "subscription QoS for /joint_states" in logger.infos[0]

    event_callbacks = captured["event_callbacks"]
    event_callbacks.incompatible_qos(
        types.SimpleNamespace(last_policy_kind=4)
    )
    assert "incompatible publisher QoS" in logger.warnings[0]
