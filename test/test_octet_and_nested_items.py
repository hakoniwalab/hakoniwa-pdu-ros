"""Tests for the two type_mapper fixes, in the style of the existing suite.

They use the same fake-ROS-module approach test/test_type_mapper.py already
uses, so they need neither a ROS installation nor a built native library.

What each one pins, and why it is written the way it is:

  - a list of ints copied into a `byte[]` (`sequence<octet>`) field must be
    STORED as bytes. rclpy accepts the list, reads it back identical, and then
    serialises every element as 0xff -- so the representation, not the values
    read back, is the thing worth asserting at this level. A unit test cannot
    serialise, and a value-only assertion does not discriminate at all:
    `bytes([0, 1, 2])` already equals what the fixed mapper stores.
  - a nested message array must be built with the DESTINATION element type.
    ROS message classes leave `__annotations__` empty, so without a second
    lookup the list is filled with objects of the source type, and every
    destination-aware rule below it -- including the byte[] handling above --
    is bypassed.
  - an out-of-range element must raise. Masking it would yield 0xff, which is
    exactly what the defect produces, so a masking mapper could not be told
    apart from a broken one.
"""

import sys
import types
import unittest

from hakoniwa_pdu_ros.type_mapper import _copy_matching_fields


def _install_fake_octet_modules() -> None:
    """A package whose message has a byte[] field inside a nested array."""
    pkg = types.ModuleType("octet_test_msgs")
    pkg_msg = types.ModuleType("octet_test_msgs.msg")

    class Item:
        def __init__(self) -> None:
            self.kind = 0
            self.data = b""

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"kind": "uint8", "data": "sequence<octet>"}

    class Bag:
        def __init__(self) -> None:
            self.items = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"items": "sequence<octet_test_msgs/Item>"}

    pkg_msg.Item = Item
    pkg_msg.Bag = Bag
    pkg.msg = pkg_msg
    sys.modules["octet_test_msgs"] = pkg
    sys.modules["octet_test_msgs.msg"] = pkg_msg


class _SourceItem:
    """Stands in for a generated PDU type: a uint8[] field decodes to ints."""

    def __init__(self, kind: int, data) -> None:
        self.kind = kind
        self.data = data


class _SourceBag:
    def __init__(self, items) -> None:
        self.items = list(items)


class OctetSequenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_octet_modules()

    def test_int_sequence_into_byte_array_is_stored_as_bytes(self) -> None:
        from octet_test_msgs.msg import Item

        target = Item()
        _copy_matching_fields(_SourceItem(7, (1, 2, 3, 255)), target)
        self.assertIsInstance(target.data, (bytes, bytearray))
        self.assertEqual(bytes(target.data), b"\x01\x02\x03\xff")
        self.assertEqual(target.kind, 7)

    def test_every_accepted_source_shape_stores_bytes(self) -> None:
        from octet_test_msgs.msg import Item

        shapes = (
            [0, 1, 2],
            (0, 1, 2),
            b"\x00\x01\x02",
            bytearray(b"\x00\x01\x02"),
            [b"\x00", b"\x01", b"\x02"],
        )
        for payload in shapes:
            target = Item()
            _copy_matching_fields(_SourceItem(1, payload), target)
            with self.subTest(shape=type(payload).__name__):
                # The type assertion is what makes this discriminate: a
                # value-only comparison passes against the unpatched mapper.
                self.assertIsInstance(target.data, (bytes, bytearray))
                self.assertEqual(bytes(target.data), b"\x00\x01\x02")

    def test_out_of_range_element_raises_instead_of_being_masked(self) -> None:
        from octet_test_msgs.msg import Item

        for bad in (-1, 256):
            with self.subTest(value=bad):
                with self.assertRaises(ValueError):
                    _copy_matching_fields(_SourceItem(1, [bad]), Item())

    def test_byte_array_back_to_a_pdu_style_target_becomes_ints(self) -> None:
        from octet_test_msgs.msg import Item

        source = Item()
        source.kind = 3
        source.data = b"\x0a\x0b"
        target = _SourceItem(0, [])
        _copy_matching_fields(source, target)
        # A PDU uint8[] field wants integers. Asserting the element type is
        # again what discriminates -- list(b"...") already equals [10, 11].
        self.assertIsInstance(target.data, list)
        self.assertTrue(all(isinstance(element, int) for element in target.data))
        self.assertEqual(target.data, [10, 11])


class NestedMessageArrayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_octet_modules()

    def test_nested_items_are_built_with_the_destination_type(self) -> None:
        from octet_test_msgs.msg import Bag, Item

        source = _SourceBag([_SourceItem(1, [1, 2]), _SourceItem(2, [])])
        target = Bag()
        _copy_matching_fields(source, target)

        self.assertEqual(len(target.items), 2)
        for item in target.items:
            self.assertIsInstance(item, Item)
        self.assertEqual(bytes(target.items[0].data), b"\x01\x02")
        self.assertEqual(bytes(target.items[1].data), b"")


if __name__ == "__main__":
    unittest.main()
