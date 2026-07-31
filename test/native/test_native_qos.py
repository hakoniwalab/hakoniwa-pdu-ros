from __future__ import annotations

import importlib
import sys
import time
import types
import uuid

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState

from hakoniwa_pdu_ros.config_loader import BindingConfig, PduKeyConfig, QosConfig
from hakoniwa_pdu_ros.type_mapper import pdu_bytes_to_ros_msg


# bridge_node normally imports the native endpoint binding. These tests exercise the
# real ROS graph and bridge subscription callback, but capture the produced PDU bytes
# in memory so the Docker image does not need a transport backend.
_endpoint_stub = types.ModuleType("hakoniwa_pdu_ros.pdu_endpoint")
_endpoint_stub.PduEndpointManager = object
sys.modules["hakoniwa_pdu_ros.pdu_endpoint"] = _endpoint_stub

bridge_module = importlib.import_module("hakoniwa_pdu_ros.bridge_node")
HakoniwaRosBridgeNode = bridge_module.HakoniwaRosBridgeNode


class CaptureManager:
    def __init__(self) -> None:
        self.payloads: list[bytes] = []

    def send(self, _robot_name: str, _pdu_name: str, data: bytes) -> None:
        self.payloads.append(bytes(data))

    def stop(self) -> None:
        pass


@pytest.fixture(scope="module", autouse=True)
def ros_runtime() -> None:
    rclpy.init(args=None)
    print(
        "[native-qos] ROS_DISTRO=jazzy "
        "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    )
    yield
    rclpy.shutdown()


def _qos(reliability: ReliabilityPolicy) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )


def _message() -> JointState:
    msg = JointState()
    msg.name = ["joint1", "joint2"]
    msg.position = [0.1, 0.2]
    msg.velocity = [1.0, 2.0]
    msg.effort = [0.01, 0.02]
    return msg


def _create_bridge(topic: str, qos: QosConfig) -> tuple[object, CaptureManager]:
    bridge = HakoniwaRosBridgeNode()
    capture = CaptureManager()
    bridge._manager = capture
    bridge._setup_out_binding(
        BindingConfig(
            direction="ros_to_pdu",
            pdu_key=PduKeyConfig(robot_name="Tobas", pdu_name="joint_states"),
            topic=topic,
            channel_id=1,
            pdu_size=496,
            pdu_type="sensor_msgs/JointState",
            qos=qos,
        )
    )
    return bridge, capture


def _publish_until(
    bridge: object,
    publisher_node: Node,
    publisher: object,
    message: JointState,
    predicate,
    *,
    timeout_sec: float = 8.0,
) -> bool:
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(publisher_node)
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline:
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        return False
    finally:
        executor.remove_node(publisher_node)
        executor.remove_node(bridge)
        publisher_node.destroy_node()
        bridge.destroy_node()
        executor.shutdown(timeout_sec=1.0)


def test_best_effort_binding_receives_best_effort_joint_state() -> None:
    topic = f"/native_test/joint_states_{uuid.uuid4().hex}"
    bridge, capture = _create_bridge(topic, QosConfig(reliability="best_effort"))
    publisher_node = Node(f"native_best_effort_pub_{uuid.uuid4().hex}")
    publisher = publisher_node.create_publisher(
        JointState,
        topic,
        _qos(ReliabilityPolicy.BEST_EFFORT),
    )

    received = _publish_until(
        bridge,
        publisher_node,
        publisher,
        _message(),
        lambda: len(capture.payloads) >= 3,
    )

    assert received, "BEST_EFFORT bridge subscription did not receive JointState"
    restored = pdu_bytes_to_ros_msg(capture.payloads[-1], "sensor_msgs/JointState")
    assert list(restored.name) == ["joint1", "joint2"]
    assert list(restored.position) == pytest.approx([0.1, 0.2])
    assert list(restored.velocity) == pytest.approx([1.0, 2.0])
    assert list(restored.effort) == pytest.approx([0.01, 0.02])
    print(f"[native-qos] BEST_EFFORT positive received={len(capture.payloads)}")


def test_reliable_binding_reports_best_effort_incompatibility(monkeypatch) -> None:
    reported: list[str] = []
    original_callback_factory = bridge_module.make_incompatible_qos_callback

    monkeypatch.setattr(
        bridge_module,
        "make_incompatible_qos_callback",
        lambda _warn, topic, config: original_callback_factory(
            reported.append,
            topic,
            config,
        ),
    )

    topic = f"/native_test/joint_states_{uuid.uuid4().hex}"
    bridge, capture = _create_bridge(topic, QosConfig(reliability="reliable"))
    publisher_node = Node(f"native_incompatible_pub_{uuid.uuid4().hex}")
    publisher = publisher_node.create_publisher(
        JointState,
        topic,
        _qos(ReliabilityPolicy.BEST_EFFORT),
    )

    diagnosed = _publish_until(
        bridge,
        publisher_node,
        publisher,
        _message(),
        lambda: bool(reported),
    )

    assert diagnosed, "CycloneDDS did not report the incompatible QoS event"
    assert capture.payloads == []
    assert "incompatible publisher QoS" in reported[0]
    assert "reliability=reliable" in reported[0]
    print(f"[native-qos] incompatible diagnostic={reported[0]}")


def test_default_binding_receives_reliable_joint_state() -> None:
    topic = f"/native_test/joint_states_{uuid.uuid4().hex}"
    bridge, capture = _create_bridge(topic, QosConfig())
    publisher_node = Node(f"native_reliable_pub_{uuid.uuid4().hex}")
    publisher = publisher_node.create_publisher(
        JointState,
        topic,
        _qos(ReliabilityPolicy.RELIABLE),
    )

    received = _publish_until(
        bridge,
        publisher_node,
        publisher,
        _message(),
        lambda: len(capture.payloads) >= 3,
    )

    assert received, "default RELIABLE bridge subscription did not receive JointState"
    print(f"[native-qos] default RELIABLE received={len(capture.payloads)}")
