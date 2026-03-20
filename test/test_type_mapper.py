from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
PDU_PYTHON_SRC = REPO_ROOT.parent / "hakoniwa-pdu-python" / "src"
if str(PDU_PYTHON_SRC) not in sys.path:
    sys.path.insert(0, str(PDU_PYTHON_SRC))

from hakoniwa_pdu_ros.type_mapper import (  # noqa: E402
    normalize_ros_msg_type,
    pdu_bytes_to_ros_msg,
    ros_msg_to_pdu_bytes,
)


def _install_fake_ros_modules() -> None:
    builtin_interfaces = types.ModuleType("builtin_interfaces")
    builtin_interfaces_msg = types.ModuleType("builtin_interfaces.msg")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    hako_msgs = types.ModuleType("hako_msgs")
    hako_msgs_msg = types.ModuleType("hako_msgs.msg")

    class Time:
        sec: int
        nanosec: int

        def __init__(self) -> None:
            self.sec = 0
            self.nanosec = 0

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {"sec": "int32", "nanosec": "uint32"}

    class Header:
        stamp: Time
        frame_id: str

        def __init__(self) -> None:
            self.stamp = Time()
            self.frame_id = ""

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {"stamp": "builtin_interfaces/Time", "frame_id": "string"}

    class PointField:
        name: str
        offset: int
        datatype: int
        count: int

        def __init__(self) -> None:
            self.name = ""
            self.offset = 0
            self.datatype = 0
            self.count = 0

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {
                "name": "string",
                "offset": "uint32",
                "datatype": "uint8",
                "count": "uint32",
            }

    class RegionOfInterest:
        x_offset: int
        y_offset: int
        height: int
        width: int
        do_rectify: bool

        def __init__(self) -> None:
            self.x_offset = 0
            self.y_offset = 0
            self.height = 0
            self.width = 0
            self.do_rectify = False

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {
                "x_offset": "uint32",
                "y_offset": "uint32",
                "height": "uint32",
                "width": "uint32",
                "do_rectify": "boolean",
            }

    class PointCloud2:
        header: Header
        height: int
        width: int
        fields: list[PointField]
        is_bigendian: bool
        point_step: int
        row_step: int
        data: list[int]
        is_dense: bool

        def __init__(self) -> None:
            self.header = Header()
            self.height = 0
            self.width = 0
            self.fields = []
            self.is_bigendian = False
            self.point_step = 0
            self.row_step = 0
            self.data = []
            self.is_dense = False

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {
                "header": "std_msgs/Header",
                "height": "uint32",
                "width": "uint32",
                "fields": "sequence<sensor_msgs/PointField>",
                "is_bigendian": "boolean",
                "point_step": "uint32",
                "row_step": "uint32",
                "data": "sequence<uint8>",
                "is_dense": "boolean",
            }

    class JointState:
        header: Header
        name: list[str]
        position: list[float]
        velocity: list[float]
        effort: list[float]

        def __init__(self) -> None:
            self.header = Header()
            self.name = []
            self.position = []
            self.velocity = []
            self.effort = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {
                "header": "std_msgs/Header",
                "name": "sequence<string>",
                "position": "sequence<double>",
                "velocity": "sequence<double>",
                "effort": "sequence<double>",
            }

    class LaserScan:
        header: Header
        angle_min: float
        angle_max: float
        angle_increment: float
        time_increment: float
        scan_time: float
        range_min: float
        range_max: float
        ranges: list[float]
        intensities: list[float]

        def __init__(self) -> None:
            self.header = Header()
            self.angle_min = 0.0
            self.angle_max = 0.0
            self.angle_increment = 0.0
            self.time_increment = 0.0
            self.scan_time = 0.0
            self.range_min = 0.0
            self.range_max = 0.0
            self.ranges = []
            self.intensities = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {
                "header": "std_msgs/Header",
                "angle_min": "float",
                "angle_max": "float",
                "angle_increment": "float",
                "time_increment": "float",
                "scan_time": "float",
                "range_min": "float",
                "range_max": "float",
                "ranges": "sequence<float>",
                "intensities": "sequence<float>",
            }

    class CameraInfo:
        header: Header
        height: int
        width: int
        distortion_model: str
        d: list[float]
        k: list[float]
        r: list[float]
        p: list[float]
        binning_x: int
        binning_y: int
        roi: RegionOfInterest

        def __init__(self) -> None:
            self.header = Header()
            self.height = 0
            self.width = 0
            self.distortion_model = ""
            self.d = []
            self.k = []
            self.r = []
            self.p = []
            self.binning_x = 0
            self.binning_y = 0
            self.roi = RegionOfInterest()

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {
                "header": "std_msgs/Header",
                "height": "uint32",
                "width": "uint32",
                "distortion_model": "string",
                "d": "sequence<double>",
                "k": "double[9]",
                "r": "double[9]",
                "p": "double[12]",
                "binning_x": "uint32",
                "binning_y": "uint32",
                "roi": "sensor_msgs/RegionOfInterest",
            }

    class MultiArrayDimension:
        label: str
        size: int
        stride: int

        def __init__(self) -> None:
            self.label = ""
            self.size = 0
            self.stride = 0

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {"label": "string", "size": "uint32", "stride": "uint32"}

    class MultiArrayLayout:
        dim: list[MultiArrayDimension]
        data_offset: int

        def __init__(self) -> None:
            self.dim = []
            self.data_offset = 0

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {"dim": "sequence<std_msgs/MultiArrayDimension>", "data_offset": "uint32"}

    class Float64MultiArray:
        layout: MultiArrayLayout
        data: list[float]

        def __init__(self) -> None:
            self.layout = MultiArrayLayout()
            self.data = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict[str, str]:
            return {"layout": "std_msgs/MultiArrayLayout", "data": "sequence<double>"}

    class SimpleVarray:
        data: list[int]
        fixed_array: list[int]
        p_mem1: int

        def __init__(self) -> None:
            self.data = []
            self.fixed_array = []
            self.p_mem1 = 0

    class SimpleStructVarray:
        aaa: int
        fixed_str: list[str]
        varray_str: list[str]
        fixed_array: list[SimpleVarray]
        data: list[SimpleVarray]

        def __init__(self) -> None:
            self.aaa = 0
            self.fixed_str = []
            self.varray_str = []
            self.fixed_array = []
            self.data = []

    builtin_interfaces_msg.Time = Time
    builtin_interfaces.msg = builtin_interfaces_msg
    std_msgs_msg.Header = Header
    std_msgs.msg = std_msgs_msg
    sensor_msgs_msg.PointField = PointField
    sensor_msgs_msg.PointCloud2 = PointCloud2
    sensor_msgs_msg.JointState = JointState
    sensor_msgs_msg.LaserScan = LaserScan
    sensor_msgs_msg.CameraInfo = CameraInfo
    sensor_msgs_msg.RegionOfInterest = RegionOfInterest
    sensor_msgs.msg = sensor_msgs_msg
    std_msgs_msg.MultiArrayDimension = MultiArrayDimension
    std_msgs_msg.MultiArrayLayout = MultiArrayLayout
    std_msgs_msg.Float64MultiArray = Float64MultiArray
    hako_msgs_msg.SimpleVarray = SimpleVarray
    hako_msgs_msg.SimpleStructVarray = SimpleStructVarray
    hako_msgs.msg = hako_msgs_msg

    sys.modules["builtin_interfaces"] = builtin_interfaces
    sys.modules["builtin_interfaces.msg"] = builtin_interfaces_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
    sys.modules["hako_msgs"] = hako_msgs
    sys.modules["hako_msgs.msg"] = hako_msgs_msg


def _make_simple_varray(data_values: list[int], fixed_values: list[int], p_mem1: int):
    from hako_msgs.msg import SimpleVarray

    obj = SimpleVarray()
    obj.data = list(data_values)
    obj.fixed_array = list(fixed_values) + [0] * (10 - len(fixed_values))
    obj.p_mem1 = p_mem1
    return obj


class TypeMapperRegistryCaseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_ros_modules()

    def test_normalize_ros_msg_type(self) -> None:
        self.assertEqual(normalize_ros_msg_type("geometry_msgs/Twist"), "geometry_msgs/Twist")
        self.assertEqual(normalize_ros_msg_type("geometry_msgs/msg/Twist"), "geometry_msgs/Twist")

    def test_joint_state_size_0_case(self) -> None:
        self._assert_joint_state_roundtrip([], [], [], [])

    def test_joint_state_size_1_case(self) -> None:
        self._assert_joint_state_roundtrip(["a"], [1.0], [3.0], [4.0])

    def test_joint_state_size_2_case(self) -> None:
        self._assert_joint_state_roundtrip(["a", "b"], [1.0, 2.0], [3.0, 5.0], [4.0, 6.0])

    def test_point_cloud2_size_0_case(self) -> None:
        self._assert_point_cloud2_roundtrip([], [])

    def test_point_cloud2_size_1_case(self) -> None:
        self._assert_point_cloud2_roundtrip(
            [{"name": "x", "offset": 0, "datatype": 7, "count": 1}],
            [1],
        )

    def test_point_cloud2_size_2_case(self) -> None:
        self._assert_point_cloud2_roundtrip(
            [
                {"name": "x", "offset": 0, "datatype": 7, "count": 1},
                {"name": "intensity", "offset": 4, "datatype": 7, "count": 1},
            ],
            [1, 2],
        )

    def test_laser_scan_size_0_case(self) -> None:
        self._assert_laser_scan_roundtrip([], [])

    def test_laser_scan_size_1_case(self) -> None:
        self._assert_laser_scan_roundtrip([1.5], [10.0])

    def test_laser_scan_size_2_case(self) -> None:
        self._assert_laser_scan_roundtrip([1.5, 2.5], [10.0, 20.0])

    def test_camera_info_size_0_case(self) -> None:
        self._assert_camera_info_roundtrip([])

    def test_camera_info_size_1_case(self) -> None:
        self._assert_camera_info_roundtrip([0.1])

    def test_camera_info_size_2_case(self) -> None:
        self._assert_camera_info_roundtrip([0.1, 0.2])

    def test_float64_multi_array_size_0_case(self) -> None:
        self._assert_float64_multi_array_roundtrip([], [])

    def test_float64_multi_array_size_1_case(self) -> None:
        self._assert_float64_multi_array_roundtrip(
            [{"label": "x", "size": 1, "stride": 1}],
            [1.5],
        )

    def test_float64_multi_array_size_2_case(self) -> None:
        self._assert_float64_multi_array_roundtrip(
            [
                {"label": "x", "size": 2, "stride": 2},
                {"label": "y", "size": 3, "stride": 6},
            ],
            [1.5, 2.5],
        )

    def _assert_joint_state_roundtrip(
        self,
        name_values: list[str],
        position_values: list[float],
        velocity_values: list[float],
        effort_values: list[float],
    ) -> None:
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.header.frame_id = "frame"
        msg.name = list(name_values)
        msg.position = list(position_values)
        msg.velocity = list(velocity_values)
        msg.effort = list(effort_values)

        binary = ros_msg_to_pdu_bytes(msg, "sensor_msgs/JointState")
        restored = pdu_bytes_to_ros_msg(binary, "sensor_msgs/JointState")

        self.assertEqual(
            {
                "frame_id": restored.header.frame_id,
                "name": list(restored.name),
                "position": list(restored.position),
                "velocity": list(restored.velocity),
                "effort": list(restored.effort),
            },
            {
                "frame_id": "frame",
                "name": list(name_values),
                "position": list(position_values),
                "velocity": list(velocity_values),
                "effort": list(effort_values),
            },
        )

    def _assert_point_cloud2_roundtrip(self, field_specs: list[dict], data_values: list[int]) -> None:
        from sensor_msgs.msg import PointCloud2, PointField

        msg = PointCloud2()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = 200
        msg.header.frame_id = "pc"
        msg.height = 2
        msg.width = 3
        msg.fields = []
        for spec in field_specs:
            field = PointField()
            field.name = spec["name"]
            field.offset = spec["offset"]
            field.datatype = spec["datatype"]
            field.count = spec["count"]
            msg.fields.append(field)
        msg.is_bigendian = False
        msg.point_step = 8
        msg.row_step = 24
        msg.data = list(data_values)
        msg.is_dense = True

        binary = ros_msg_to_pdu_bytes(msg, "sensor_msgs/PointCloud2")
        restored = pdu_bytes_to_ros_msg(binary, "sensor_msgs/PointCloud2")

        self.assertEqual(
            self._decode_point_cloud2(restored),
            {
                "header": {"stamp": {"sec": 1, "nanosec": 200}, "frame_id": "pc"},
                "height": 2,
                "width": 3,
                "fields": [dict(spec) for spec in field_specs],
                "is_bigendian": False,
                "point_step": 8,
                "row_step": 24,
                "data": list(data_values),
                "is_dense": True,
            },
        )

    def _assert_laser_scan_roundtrip(self, ranges: list[float], intensities: list[float]) -> None:
        from sensor_msgs.msg import LaserScan

        msg = LaserScan()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = 200
        msg.header.frame_id = "laser"
        msg.angle_min = -1.0
        msg.angle_max = 1.0
        msg.angle_increment = 0.5
        msg.time_increment = 0.1
        msg.scan_time = 0.2
        msg.range_min = 0.3
        msg.range_max = 30.0
        msg.ranges = list(ranges)
        msg.intensities = list(intensities)

        binary = ros_msg_to_pdu_bytes(msg, "sensor_msgs/LaserScan")
        restored = pdu_bytes_to_ros_msg(binary, "sensor_msgs/LaserScan")

        self.assertEqual(
            {"ranges": list(restored.ranges), "intensities": list(restored.intensities)},
            {"ranges": list(ranges), "intensities": list(intensities)},
        )

    def _assert_camera_info_roundtrip(self, d_values: list[float]) -> None:
        from sensor_msgs.msg import CameraInfo

        msg = CameraInfo()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = 200
        msg.header.frame_id = "cam"
        msg.height = 480
        msg.width = 640
        msg.distortion_model = "plumb_bob"
        msg.d = list(d_values)
        msg.k = [1.0] * 9
        msg.r = [2.0] * 9
        msg.p = [3.0] * 12
        msg.binning_x = 1
        msg.binning_y = 2
        msg.roi.x_offset = 3
        msg.roi.y_offset = 4
        msg.roi.height = 5
        msg.roi.width = 6
        msg.roi.do_rectify = True

        binary = ros_msg_to_pdu_bytes(msg, "sensor_msgs/CameraInfo")
        restored = pdu_bytes_to_ros_msg(binary, "sensor_msgs/CameraInfo")

        self.assertEqual(list(restored.d), list(d_values))

    def _assert_float64_multi_array_roundtrip(self, dim_specs: list[dict], data_values: list[float]) -> None:
        from std_msgs.msg import Float64MultiArray, MultiArrayDimension

        msg = Float64MultiArray()
        msg.layout.dim = []
        for spec in dim_specs:
            dim = MultiArrayDimension()
            dim.label = spec["label"]
            dim.size = spec["size"]
            dim.stride = spec["stride"]
            msg.layout.dim.append(dim)
        msg.layout.data_offset = 9
        msg.data = list(data_values)

        binary = ros_msg_to_pdu_bytes(msg, "std_msgs/Float64MultiArray")
        restored = pdu_bytes_to_ros_msg(binary, "std_msgs/Float64MultiArray")

        self.assertEqual(
            {
                "layout": {
                    "dim": [
                        {"label": item.label, "size": item.size, "stride": item.stride}
                        for item in restored.layout.dim
                    ],
                    "data_offset": restored.layout.data_offset,
                },
                "data": list(restored.data),
            },
            {
                "layout": {"dim": [dict(spec) for spec in dim_specs], "data_offset": 9},
                "data": list(data_values),
            },
        )

    def _decode_point_cloud2(self, msg) -> dict:
        return {
            "header": {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id,
            },
            "height": msg.height,
            "width": msg.width,
            "fields": [
                {
                    "name": item.name,
                    "offset": item.offset,
                    "datatype": item.datatype,
                    "count": item.count,
                }
                for item in msg.fields
            ],
            "is_bigendian": bool(msg.is_bigendian),
            "point_step": msg.point_step,
            "row_step": msg.row_step,
            "data": list(msg.data),
            "is_dense": bool(msg.is_dense),
        }


if __name__ == "__main__":
    unittest.main()
