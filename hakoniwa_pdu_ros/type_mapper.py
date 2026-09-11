from __future__ import annotations

import array
import importlib
import struct
from functools import lru_cache
from typing import get_args, get_origin

from hakoniwa_pdu_ros.env_setup import configure_import_paths

configure_import_paths()


def normalize_ros_msg_type(type_name: str) -> str:
    if "/msg/" in type_name:
        package_name, _, msg_name = type_name.split("/", 2)
        return f"{package_name}/{msg_name}"
    return type_name


def import_ros_msg_class(type_name: str) -> type:
    normalized = normalize_ros_msg_type(type_name)
    package_name, msg_name = normalized.split("/", 1)
    module = importlib.import_module(f"{package_name}.msg")
    return getattr(module, msg_name)


def import_ros_service_class(type_name: str) -> type:
    package_name, namespace, service_name = type_name.split("/", 2)
    if namespace != "srv":
        raise ValueError(f"ROS service type must use package/srv/Type form: {type_name}")
    module = importlib.import_module(f"{package_name}.srv")
    return getattr(module, service_name)


def copy_matching_fields(src: object, dst: object) -> object:
    """Copy a ROS/PDU body by field name and return the destination object."""
    _copy_matching_fields(src, dst)
    return dst


def validate_pdu_converter(type_name: str) -> None:
    _load_converter(type_name)


def pdu_bytes_to_ros_msg(data: bytes, type_name: str) -> object:
    ros_msg_cls = import_ros_msg_class(type_name)
    _, pdu_to_py, _ = _load_converter(type_name)
    pdu_obj = pdu_to_py(bytearray(data))
    ros_msg = ros_msg_cls()
    _copy_matching_fields(pdu_obj, ros_msg)
    return ros_msg


def ros_msg_to_pdu_bytes(msg: object, type_name: str) -> bytes:
    pdu_pytype_cls, _, py_to_pdu = _load_converter(type_name)
    pdu_obj = pdu_pytype_cls()
    _copy_matching_fields(msg, pdu_obj)
    return bytes(py_to_pdu(pdu_obj))


@lru_cache(maxsize=None)
def _load_converter(type_name: str) -> tuple[type, callable, callable]:
    normalized = normalize_ros_msg_type(type_name)
    package_name, msg_name = normalized.split("/", 1)

    conv_module = importlib.import_module(
        f"hakoniwa_pdu.pdu_msgs.{package_name}.pdu_conv_{msg_name}"
    )
    pytype_module = importlib.import_module(
        f"hakoniwa_pdu.pdu_msgs.{package_name}.pdu_pytype_{msg_name}"
    )

    pdu_pytype_cls = getattr(pytype_module, msg_name)
    pdu_to_py = getattr(conv_module, f"pdu_to_py_{msg_name}")
    py_to_pdu = getattr(conv_module, f"py_to_pdu_{msg_name}")
    return pdu_pytype_cls, pdu_to_py, py_to_pdu


def _copy_matching_fields(src: object, dst: object) -> None:
    field_names = _field_names(src, dst)
    for name in field_names:
        src_value = getattr(src, name)
        dst_value = getattr(dst, name, None)
        # A ROS byte[] (sequence<octet>) only serialises correctly when its
        # elements are bytes-like; a list of ints is accepted, reads back
        # identical, and then serialises as all-0xff. Normalise on whichever
        # side declares the field as octet, before anything else looks at it.
        if _is_octet_sequence_field(dst, name):
            setattr(dst, name, _as_octet_bytes(src_value, name))
            continue
        if _is_octet_sequence_field(src, name):
            setattr(dst, name, _as_octet_ints(src_value, name))
            continue
        if isinstance(src_value, (bytes, bytearray)):
            decoded = _decode_binary_sequence(dst, name, src_value)
            if decoded is not None:
                setattr(dst, name, decoded)
                continue
        if _is_scalar(src_value):
            setattr(dst, name, src_value)
        elif isinstance(src_value, (list, tuple, array.array)):
            setattr(dst, name, _copy_list(src_value, dst, name, dst_value))
        elif _is_declared_sequence_field(src, dst, name):
            raise TypeError(
                f"Unsupported sequence value for field '{name}': "
                f"{type(src_value).__module__}.{type(src_value).__qualname__}"
            )
        else:
            if dst_value is None:
                setattr(dst, name, src_value)
            else:
                _copy_matching_fields(src_value, dst_value)


def _is_octet_sequence_field(obj: object, field_name: str) -> bool:
    """True when ``obj`` declares ``field_name`` as a ROS byte[] field.

    ROS `byte[]` is `sequence<octet>` in IDL. rclpy accepts a list of ints for
    such a field and reads it back unchanged, but serialises every element as
    0xff; only bytes-like elements survive. Measured on Jazzy with rclpy
    7.1.11 / rosidl-generator-py 0.22.2, with and without DDS.
    """
    field_type = _field_type_name(obj, field_name)
    if field_type is None:
        return False
    return _primitive_sequence_type(field_type) == "octet"


def _as_octet_bytes(value, field_name: str) -> bytes:
    """Normalise any accepted byte-sequence shape to ``bytes``."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        return bytes(value)
    out = bytearray()
    for element in value:
        if isinstance(element, (bytes, bytearray)):
            out.extend(element)
            continue
        number = int(element)
        if not 0 <= number <= 255:
            # Deliberately loud. rclpy renders an out-of-range value as 0xff,
            # exactly like the bug this normalisation exists to prevent, so
            # masking here would make a real error indistinguishable from it.
            raise ValueError(
                f"byte[] field '{field_name}' element out of range: {number}"
            )
        out.append(number)
    return bytes(out)


def _as_octet_ints(value, field_name: str) -> list:
    """Normalise any accepted byte-sequence shape to a list of ints."""
    out: list[int] = []
    if isinstance(value, (bytes, bytearray, memoryview)):
        return list(value)
    for element in value:
        if isinstance(element, (bytes, bytearray)):
            out.extend(element)
            continue
        number = int(element)
        if not 0 <= number <= 255:
            raise ValueError(
                f"byte[] field '{field_name}' element out of range: {number}"
            )
        out.append(number)
    return out


def _copy_list(src_list, dst_parent: object, field_name: str, dst_list: object) -> list:
    if not src_list:
        return []
    if not isinstance(src_list[0], (list, tuple)) and _is_scalar(src_list[0]):
        return list(src_list)

    item_type = _list_item_type(dst_parent, field_name)
    copied = []
    dst_items = dst_list if isinstance(dst_list, list) else []
    for index, src_item in enumerate(src_list):
        if _is_scalar(src_item):
            copied.append(src_item)
            continue
        if index < len(dst_items):
            dst_item = dst_items[index]
        elif item_type is not None:
            dst_item = item_type()
        else:
            dst_item = src_item.__class__()
        _copy_matching_fields(src_item, dst_item)
        copied.append(dst_item)
    return copied


def _list_item_type(dst_parent: object, field_name: str):
    annotations = getattr(dst_parent.__class__, "__annotations__", {})
    field_type = annotations.get(field_name)
    if field_type is not None:
        origin = get_origin(field_type)
        if origin in {list, tuple}:
            args = get_args(field_type)
            if args:
                return args[0]
        return None
    # ROS message classes leave __annotations__ empty and declare their fields
    # through get_fields_and_field_types() instead, so the path above always
    # returns None for them. Without this second lookup a nested message list
    # is filled with objects of the SOURCE type: the ROS message ends up
    # holding PDU objects, rosidl duck-types its way through serialisation,
    # and any destination-type-aware handling never gets a chance to run --
    # byte[] being the case that bites, since a PDU object hands it a list of
    # ints. Measured 2026-09-11 with tobas_mission_msgs/Mission, whose
    # __annotations__ is {} while get_fields_and_field_types() reports
    # {'items': 'sequence<tobas_mission_msgs/MissionItem>'}.
    declared = _field_type_name(dst_parent, field_name)
    if declared is None:
        return None
    inner = _primitive_sequence_type(declared)
    if inner is None or "/" not in inner:
        return None
    package_name, _, message_name = inner.partition("/")
    try:
        module = importlib.import_module(f"{package_name}.msg")
    except ModuleNotFoundError:
        return None
    return getattr(module, message_name, None)


def _decode_binary_sequence(dst_parent: object, field_name: str, raw: bytes | bytearray):
    field_type = _field_type_name(dst_parent, field_name)
    if field_type is None:
        return None
    primitive = _primitive_sequence_type(field_type)
    if primitive is None:
        return None
    if primitive == "string":
        return None
    if primitive in {"uint8", "int8"}:
        return list(raw)
    if primitive == "boolean":
        count = len(raw) // 4
        values = struct.unpack(f"<{count}i", bytes(raw)) if count else ()
        return [value != 0 for value in values]
    format_info = _primitive_struct_format(primitive)
    if format_info is None:
        return None
    format_char, item_size = format_info
    count = len(raw) // item_size
    return list(struct.unpack(f"<{count}{format_char}", bytes(raw))) if count else []
    return None


def _field_type_name(obj: object, field_name: str) -> str | None:
    getter = getattr(obj.__class__, "get_fields_and_field_types", None)
    if callable(getter):
        return getter().get(field_name)
    field_types = getattr(obj.__class__, "_fields_and_field_types", None)
    if isinstance(field_types, dict):
        return field_types.get(field_name)
    return None


def _is_declared_sequence_field(src: object, dst: object, field_name: str) -> bool:
    field_type = _field_type_name(src, field_name)
    if field_type is None:
        field_type = _field_type_name(dst, field_name)
    return field_type is not None and _primitive_sequence_type(field_type) is not None


def _primitive_sequence_type(field_type: str) -> str | None:
    if field_type.startswith("sequence<") and field_type.endswith(">"):
        return field_type[len("sequence<") : -1]
    if field_type.endswith("]"):
        return field_type.split("[", 1)[0]
    return None


def _primitive_struct_format(type_name: str) -> tuple[str, int] | None:
    mapping = {
        "int16": ("h", 2),
        "uint16": ("H", 2),
        "int32": ("i", 4),
        "uint32": ("I", 4),
        "int64": ("q", 8),
        "uint64": ("Q", 8),
        "float": ("f", 4),
        "float32": ("f", 4),
        "double": ("d", 8),
        "float64": ("d", 8),
    }
    return mapping.get(type_name)


def _field_names(src: object, dst: object) -> list[str]:
    names = []
    for name in dir(src):
        if name.startswith("_"):
            continue
        if callable(getattr(src, name)):
            continue
        if hasattr(dst, name):
            names.append(name)
    return names


def _is_scalar(value: object) -> bool:
    return isinstance(value, (bool, int, float, str, bytes, bytearray))
