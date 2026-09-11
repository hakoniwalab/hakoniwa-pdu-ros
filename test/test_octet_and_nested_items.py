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

import sys
import types
import unittest

from hakoniwa_pdu_ros.type_mapper import _copy_matching_fields, _primitive_sequence_type


def _install_fake_octet_modules() -> None:
    """Messages with byte[] fields, unbounded and bounded, nested and flat."""
    pkg = types.ModuleType("octet_test_msgs")
    pkg_msg = types.ModuleType("octet_test_msgs.msg")

    class Item:
        def __init__(self) -> None:
            self.kind = 0
            self.data = b""

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"kind": "uint8", "data": "sequence<octet>"}

    class BoundedItem:
        def __init__(self) -> None:
            self.data = b""

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"data": "sequence<octet, 8>"}

    class Bag:
        def __init__(self) -> None:
            self.items = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"items": "sequence<octet_test_msgs/Item>"}

    class BoundedBag:
        def __init__(self) -> None:
            self.items = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"items": "sequence<octet_test_msgs/Item, 4>"}

    class Widened:
        """Destination that reads the same bytes as a different primitive."""

        def __init__(self) -> None:
            self.data = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"data": "sequence<uint16>"}

    class Undeclared:
        """Declares a message element type its package does not provide."""

        def __init__(self) -> None:
            self.items = []

        @classmethod
        def get_fields_and_field_types(cls) -> dict:
            return {"items": "sequence<octet_test_msgs/Missing>"}

    for cls in (Item, BoundedItem, Bag, BoundedBag, Widened, Undeclared):
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
        self.data = list(data)


class _SourceBag:
    def __init__(self, items=()) -> None:
        self.items = list(items)


class SequenceDeclarationParsingTest(unittest.TestCase):
    def test_bounds_are_stripped_from_the_element_type(self) -> None:
        cases = {
            "sequence<octet>": "octet",
            "sequence<octet, 8>": "octet",
            "sequence<pkg/Type>": "pkg/Type",
            "sequence<pkg/Type, 5>": "pkg/Type",
            "pkg/Type[5]": "pkg/Type",
            "uint8[]": "uint8",
        }
        for declaration, expected in cases.items():
            with self.subTest(declaration=declaration):
                self.assertEqual(_primitive_sequence_type(declaration), expected)


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
                self.assertIsInstance(target.data, (bytes, bytearray))
                self.assertEqual(bytes(target.data), b"\x00\x01\x02")

    def test_empty_payload_is_preserved(self) -> None:
        from octet_test_msgs.msg import Item

        target = Item()
        _copy_matching_fields(_SourceItem(1, []), target)
        self.assertEqual(bytes(target.data), b"")

    def test_bounded_byte_array_is_normalised_too(self) -> None:
        from octet_test_msgs.msg import BoundedItem

        target = BoundedItem()
        _copy_matching_fields(_SourceItem(0, [1, 2]), target)
        self.assertIsInstance(target.data, (bytes, bytearray))
        self.assertEqual(bytes(target.data), b"\x01\x02")

    def test_malformed_elements_are_rejected_rather_than_coerced(self) -> None:
        from octet_test_msgs.msg import Item

        # Each of these has a plausible-looking coercion that silently changes
        # the payload or its length, which is the failure mode being fixed.
        for payload, error in (
            ("12", TypeError),            # would become b"\x01\x02"
            ([1.9], TypeError),           # would become b"\x01"
            ([b"ab", b""], ValueError),   # would merge two elements into one
            ([True], TypeError),          # bool is an int subclass
            ([-1], ValueError),
            ([256], ValueError),
        ):
            with self.subTest(payload=payload):
                with self.assertRaises(error):
                    _copy_matching_fields(_SourceItem(1, payload), Item())

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

    def test_bounded_nested_array_resolves_its_element_type(self) -> None:
        from octet_test_msgs.msg import BoundedBag, Item

        target = BoundedBag()
        _copy_matching_fields(_SourceBag([_SourceItem(1, [3])]), target)
        self.assertIsInstance(target.items[0], Item)
        self.assertEqual(bytes(target.items[0].data), b"\x03")

    def test_existing_elements_of_the_wrong_type_are_replaced(self) -> None:
        from octet_test_msgs.msg import Bag, Item

        target = Bag()
        # A destination left holding source-type elements by an earlier copy.
        target.items = [_SourceItem(9, [9])]
        _copy_matching_fields(_SourceBag([_SourceItem(1, [1, 2])]), target)
        self.assertIsInstance(target.items[0], Item)
        self.assertEqual(bytes(target.items[0].data), b"\x01\x02")

    def test_unresolvable_declared_element_type_is_reported(self) -> None:
        from octet_test_msgs.msg import Undeclared

        # Silently falling back to the source type is what the lookup exists
        # to prevent, so an unresolvable declaration has to be loud.
        with self.assertRaises(TypeError):
            _copy_matching_fields(_SourceBag([_SourceItem(1, [1])]), Undeclared())


if __name__ == "__main__":
    unittest.main()
