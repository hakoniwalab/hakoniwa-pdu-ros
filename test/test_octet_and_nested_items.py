"""Tests for byte[] handling and nested message-array element types.

They use the same fake-ROS-module approach test/test_type_mapper.py already
uses, so they need neither a ROS installation nor a built native library.

Scope of what a test at this level can show. The fake classes have plain
attributes and no generated setters, so these pin the MAPPER's behaviour, not
what a real rosidl-generated class accepts and not CDR correctness. The
representation is asserted rather than the values read back, because reading a
list of ints back returns the right numbers -- that is exactly why the defect
survived, and a value-only assertion does not discriminate at all
(`bytes([0, 1, 2])` already equals what the fixed mapper stores).
"""

import array
import sys
import types
import unittest

from hakoniwa_pdu_ros.type_mapper import _copy_matching_fields


def _install_fake_octet_modules() -> None:
    """Messages with byte[] fields, flat and nested."""
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

    class Widened:
        """Destination that reads the same bytes as a different primitive."""

        def __init__(self) -> None:
            self.data = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"data": "sequence<uint16>"}

    class BoundedBag:
        """Bounded nested array: parsing the bound off is follow-up scope."""

        def __init__(self) -> None:
            self.items = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"items": "sequence<octet_test_msgs/Item, 4>"}

    class Undeclared:
        """Declares a message element type its package does not provide."""

        def __init__(self) -> None:
            self.items = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"items": "sequence<octet_test_msgs/Missing>"}

    for cls in (Item, Bag, BoundedBag, Widened, Undeclared):
        setattr(pkg_msg, cls.__name__, cls)
    pkg.msg = pkg_msg
    sys.modules["octet_test_msgs"] = pkg
    sys.modules["octet_test_msgs.msg"] = pkg_msg


class _SourceItem:
    """Stands in for a generated PDU type: a uint8[] field decodes to ints.

    Default-constructible on purpose. The old implementation builds elements
    with ``src_item.__class__()``, so a source class that required arguments
    would make the nested test fail with a constructor TypeError before its
    real assertion was ever reached -- passing for the wrong reason.
    """

    def __init__(self, kind: int = 0, data=()) -> None:
        self.kind = kind
        # Stored as given. Coercing to a list here would make every shape in
        # the shapes test arrive as a list, so that test would assert nothing.
        self.data = data


class _SourceBag:
    def __init__(self, items=()) -> None:
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
            memoryview(b"\x00\x01\x02"),
            array.array("B", [0, 1, 2]),
            [b"\x00", b"\x01", b"\x02"],
        )
        for payload in shapes:
            target = Item()
            _copy_matching_fields(_SourceItem(1, payload), target)
            with self.subTest(shape=type(payload).__name__):
                self.assertIsInstance(target.data, (bytes, bytearray))
                self.assertEqual(bytes(target.data), b"\x00\x01\x02")

    def test_empty_payload_is_preserved(self) -> None:
        from octet_test_msgs.msg import Item

        target = Item()
        _copy_matching_fields(_SourceItem(1, []), target)
        self.assertEqual(bytes(target.data), b"")

    def test_unrecognised_shapes_keep_their_existing_behaviour(self) -> None:
        from octet_test_msgs.msg import Item

        # Normalisation reads the shapes a byte[] field legitimately arrives
        # in and leaves everything else where it was, so this change does not
        # alter what the mapper does with a malformed payload. Rejecting these
        # would be a change to the existing pass-through contract and belongs
        # in a follow-up; what is asserted here is that the value is untouched,
        # not that passing it through is correct.
        for payload in ([1.9], [b"ab", b""], [True], [-1], [256]):
            with self.subTest(payload=payload):
                target = Item()
                _copy_matching_fields(_SourceItem(1, payload), target)
                self.assertEqual(target.data, list(payload))

    def test_byte_array_back_to_an_undeclared_target_becomes_ints(self) -> None:
        from octet_test_msgs.msg import Item

        source = Item()
        source.kind = 3
        source.data = b"\x0a\x0b"
        target = _SourceItem()
        _copy_matching_fields(source, target)
        # A PDU uint8[] field wants integers. Asserting the element type is
        # what discriminates here -- list(b"...") already equals [10, 11].
        self.assertIsInstance(target.data, list)
        self.assertTrue(all(isinstance(element, int) for element in target.data))
        self.assertEqual(target.data, [10, 11])

    def test_octet_source_does_not_override_a_declared_destination(self) -> None:
        from octet_test_msgs.msg import Item, Widened

        source = Item()
        source.data = b"\x01\x00"
        target = Widened()
        _copy_matching_fields(source, target)
        # The destination declares sequence<uint16> and keeps its own reading
        # of the same bytes; the octet source must not force it to per-byte
        # integers.
        self.assertEqual(target.data, [1])

    def test_octet_on_both_sides_yields_bytes(self) -> None:
        from octet_test_msgs.msg import Item

        source = Item()
        source.data = b"\x07\x08"
        target = Item()
        _copy_matching_fields(source, target)
        self.assertIsInstance(target.data, (bytes, bytearray))
        self.assertEqual(bytes(target.data), b"\x07\x08")


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

    def test_bounded_declaration_keeps_its_previous_handling(self) -> None:
        from octet_test_msgs.msg import BoundedBag

        # "sequence<pkg/Type, N>" still reads as the element type "pkg/Type, N"
        # here, because parsing the bound off went to the follow-up. What this
        # pins is that the resolution added by this PR does not turn that into
        # an error: the copy goes through as it did before.
        target = BoundedBag()
        _copy_matching_fields(_SourceBag([_SourceItem(1, [1, 2])]), target)
        self.assertEqual(len(target.items), 1)

    def test_unresolvable_declared_element_type_is_reported(self) -> None:
        from octet_test_msgs.msg import Undeclared

        # Silently falling back to the source type is what the lookup exists
        # to prevent, so an unresolvable declaration has to be loud.
        with self.assertRaises(TypeError):
            _copy_matching_fields(_SourceBag([_SourceItem(1, [1])]), Undeclared())


if __name__ == "__main__":
    unittest.main()
