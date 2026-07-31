from __future__ import annotations

import array

import pytest
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray

from hakoniwa_pdu_ros.type_mapper import pdu_bytes_to_ros_msg, ros_msg_to_pdu_bytes


def _assert_values(actual: object, expected: list[float]) -> None:
    assert list(actual) == pytest.approx(expected)


def test_native_joint_state_primitive_sequences_round_trip() -> None:
    msg = JointState()
    msg.name = ["joint1", "joint2"]
    msg.position = [0.1, 0.2]
    msg.velocity = [1.0, 2.0]
    msg.effort = [0.01, 0.02]

    assert isinstance(msg.position, array.array)
    assert isinstance(msg.velocity, array.array)
    assert isinstance(msg.effort, array.array)

    payload = ros_msg_to_pdu_bytes(msg, "sensor_msgs/JointState")
    restored = pdu_bytes_to_ros_msg(payload, "sensor_msgs/JointState")

    assert list(restored.name) == ["joint1", "joint2"]
    _assert_values(restored.position, [0.1, 0.2])
    _assert_values(restored.velocity, [1.0, 2.0])
    _assert_values(restored.effort, [0.01, 0.02])


def test_native_laser_scan_primitive_sequences_round_trip() -> None:
    msg = LaserScan()
    msg.angle_min = -1.0
    msg.angle_max = 1.0
    msg.angle_increment = 0.5
    msg.ranges = [1.0, 2.0, 3.0]
    msg.intensities = [0.1, 0.2, 0.3]

    assert isinstance(msg.ranges, array.array)
    assert isinstance(msg.intensities, array.array)

    payload = ros_msg_to_pdu_bytes(msg, "sensor_msgs/LaserScan")
    restored = pdu_bytes_to_ros_msg(payload, "sensor_msgs/LaserScan")

    _assert_values(restored.ranges, [1.0, 2.0, 3.0])
    _assert_values(restored.intensities, [0.1, 0.2, 0.3])


def test_native_float64_multi_array_round_trip() -> None:
    msg = Float64MultiArray()
    msg.data = [1.5, 2.5, 3.5]

    assert isinstance(msg.data, array.array)

    payload = ros_msg_to_pdu_bytes(msg, "std_msgs/Float64MultiArray")
    restored = pdu_bytes_to_ros_msg(payload, "std_msgs/Float64MultiArray")

    _assert_values(restored.data, [1.5, 2.5, 3.5])
