# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

# See pebbletechnology.atlassian.net/wiki/display/DEV/Project%3A+Vibe+Pattern+Format
import json
import os
import struct
import sys
import unittest

# Allow us to run even if not at the `tools` directory.
root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.insert(0, root_dir)

from json2vibe import *


class TestJsonToVibe(unittest.TestCase):
    def get_json_data(self, json_key):
        with open(
            os.path.join(os.path.dirname(__file__), "json2vibe_test.json"), "r"
        ) as f:
            return json.load(f)[json_key]

    def test_check_vibe_file_serialization(self):
        serialized_vibe_file = VibeFile(
            score=VibeScore(
                version=1,
                attr_list=VibeAttributeList(
                    attributes=[
                        VibeAttribute(
                            attribute=VibeNoteList(
                                notes=[
                                    VibeNote(
                                        vibe_duration_ms=1234,
                                        brake_duration_ms=99,
                                        strength=-33,
                                    ),
                                    VibeNote(
                                        vibe_duration_ms=999,
                                        brake_duration_ms=0,
                                        strength=0,
                                    ),
                                    VibeNote(
                                        vibe_duration_ms=55,
                                        brake_duration_ms=19,
                                        strength=76,
                                    ),
                                ]
                            )
                        ),
                        VibeAttribute(
                            attribute=VibePattern(indices=[0, 1, 2, 2, 1, 0])
                        ),
                        VibeAttribute(attribute=VibePatternRepeatDelay(duration=500)),
                    ]
                ),
            )
        ).serialise()

        to_test_byte_array = bytearray()
        to_test_byte_array.extend(b"VIBE")  # fourcc = 'VIBE'
        to_test_byte_array.extend(b"\x01\x00")  # version = 1
        to_test_byte_array.extend(b"\x00\x00\x00\x00")  # reserved
        to_test_byte_array.extend(b"\x1e\x00")  # att_list_size = 25

        # GenericAttributeList
        to_test_byte_array.extend(b"\x03")  # num_attributes = 3

        to_test_byte_array.extend(b"\x01")  # VibeAttributeIdVibeNotes
        to_test_byte_array.extend(b"\x0c\x00")  # 3 notes * 4 bytes per note = 12 bytes
        # First note
        to_test_byte_array.extend(b"\xd2\x04")  # vibe_duration_ms = 1234
        to_test_byte_array.extend(b"\x63")  # brake_duration_ms = 99
        to_test_byte_array.extend(b"\xdf")  # strength = -33
        # Second note
        to_test_byte_array.extend(b"\xe7\x03")  # vibe_duration_ms = 999
        to_test_byte_array.extend(b"\x00")  # brake_duration_ms = 0
        to_test_byte_array.extend(b"\x00")  # strength = 0
        # Third note
        to_test_byte_array.extend(b"\x37\x00")  # vibe_duration_ms = 55
        to_test_byte_array.extend(b"\x13")  # brake_duration_ms =  19
        to_test_byte_array.extend(b"\x4c")  # strength = 76

        to_test_byte_array.extend(b"\x02")  # VibeAttributeIdVibePattern
        to_test_byte_array.extend(b"\x06\x00")  # pattern contains 6 items
        to_test_byte_array.extend(
            b"\x00\x01\x02\x02\x01\x00"
        )  # pattern = [0,1,2,2,1,0]

        to_test_byte_array.extend(b"\x03")  # VibeAttributeId_RepeatDelay
        to_test_byte_array.extend(b"\x02\x00")  # uint16 size in bytes
        to_test_byte_array.extend(b"\xf4\x01")  # pattern = [0,1,2,2,1,0]

        self.assertEqual(bytearray(serialized_vibe_file), to_test_byte_array)

    def check_proper_vibe_resource(self, serialized_data):
        parsed_vibe_file, parsed_length = VibeFile().parse(serialized_data)
        self.assertEqual(parsed_length, 30)
        to_compare = VibeFile(
            fourcc="VIBE",
            score=VibeScore(
                version=1,
                reserved=None,
                length=18,
                attr_list=VibeAttributeList(
                    num_attributes=2,
                    attributes=[
                        VibeAttribute(
                            id=0x01,
                            length=8,
                            attribute=VibeNoteList(
                                notes=[
                                    VibeNote(
                                        vibe_duration_ms=15,
                                        brake_duration_ms=9,
                                        strength=100,
                                    ),
                                    VibeNote(
                                        vibe_duration_ms=100,
                                        brake_duration_ms=0,
                                        strength=0,
                                    ),
                                ]
                            ),
                        ),
                        VibeAttribute(
                            id=0x02, length=3, attribute=VibePattern(indices=[0, 1, 0])
                        ),
                    ],
                ),
            ),
        )
        self.assertEqual(parsed_vibe_file, to_compare)

    def test_proper_vibe_resource_string_ids(self):
        json_data = self.get_json_data("good_using_string_ids")
        self.check_proper_vibe_resource(serialize(json_data))

    def test_vibe_resource_numeric_ids(self):
        json_data = self.get_json_data("good_using_numeric_ids")
        self.check_proper_vibe_resource(serialize(json_data))

    def test_vibe_resource_negative_strengths(self):
        json_data = self.get_json_data("good_negative_strength")
        parsed_vibe_file, parsed_length = VibeFile().parse(serialize(json_data))
        self.assertEqual(parsed_length, 37)
        to_compare = VibeFile(
            fourcc="VIBE",
            score=VibeScore(
                version=1,
                reserved=None,
                length=25,
                attr_list=VibeAttributeList(
                    num_attributes=2,
                    attributes=[
                        VibeAttribute(
                            id=0x01,
                            length=12,
                            attribute=VibeNoteList(
                                notes=[
                                    VibeNote(
                                        vibe_duration_ms=1700,
                                        brake_duration_ms=120,
                                        strength=-76,
                                    ),
                                    VibeNote(
                                        vibe_duration_ms=900,
                                        brake_duration_ms=100,
                                        strength=-50,
                                    ),
                                    VibeNote(
                                        vibe_duration_ms=2000,
                                        brake_duration_ms=0,
                                        strength=0,
                                    ),
                                ]
                            ),
                        ),
                        VibeAttribute(
                            id=0x02,
                            length=6,
                            attribute=VibePattern(indices=[1, 1, 2, 0, 2, 0]),
                        ),
                    ],
                ),
            ),
        )

    def test_nonzero_repeating_delay(self):
        json_data = self.get_json_data("nonzero_repeating_delay")
        parsed_vibe_file, parsed_length = VibeFile().parse(serialize(json_data))
        self.assertEqual(parsed_length, 35)
        to_compare = VibeFile(
            fourcc="VIBE",
            score=VibeScore(
                version=1,
                reserved=None,
                length=23,
                attr_list=VibeAttributeList(
                    num_attributes=3,
                    attributes=[
                        VibeAttribute(
                            id=0x01,
                            length=8,
                            attribute=VibeNoteList(
                                notes=[
                                    VibeNote(
                                        vibe_duration_ms=15,
                                        brake_duration_ms=9,
                                        strength=100,
                                    ),
                                    VibeNote(
                                        vibe_duration_ms=100,
                                        brake_duration_ms=0,
                                        strength=0,
                                    ),
                                ]
                            ),
                        ),
                        VibeAttribute(
                            id=0x02, length=3, attribute=VibePattern(indices=[0, 1, 0])
                        ),
                        VibeAttribute(
                            id=0x03,
                            length=2,
                            attribute=VibePatternRepeatDelay(duration=1092),
                        ),
                    ],
                ),
            ),
        )

    def test_no_pattern_throws_error(self):
        with self.assertRaises(KeyError):
            serialize(self.get_json_data("bad_no_pattern"))

    def test_nonexistent_id_throws_error(self):
        with self.assertRaises(KeyError):
            serialize(self.get_json_data("bad_reference_nonexistent_id"))

    def test_negative_vibe_duration_throws_error(self):
        with self.assertRaisesRegex(
            struct.error, r"(integer out of range|'H' format requires)"
        ):
            serialize(self.get_json_data("bad_negative_vibe_duration"))

    def test_negative_brake_duration_throws_error(self):
        with self.assertRaisesRegex(
            struct.error, r"(ubyte format requires|'B' format requires)"
        ):
            serialize(self.get_json_data("bad_negative_brake_duration"))

    def test_strength_above_100_throws_error(self):
        with self.assertRaisesRegex(
            ValueError,
            '"strength" 150 out of bounds. Values between -100 and 100 only.',
        ):
            serialize(self.get_json_data("bad_strength_greater_than_100"))


if __name__ == "__main__":
    unittest.main()
