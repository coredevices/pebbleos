# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import array
import os
import sys
import unittest
from struct import pack

# Allow us to run even if not at the `tools` directory.
root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.insert(0, root_dir)

from generate_pdcs import pebble_commands

circle_example_output = [
    2,
    0,
    pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF),
    1,
    pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF),
    0x2C,
    0x01,
    0x01,
    0x00,
    0xFA,
    0xFF,
    0x06,
    0x00,
]
path_example_output = [
    1,
    0,
    pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF),
    1,
    pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF),
    1,
    0,
    0x02,
    0x00,
    0x01,
    0x00,
    0x06,
    0x00,
    0xFC,
    0xFF,
    0x02,
    0x00,
]


class MyTestCase(unittest.TestCase):
    def assertPointsEqual(self, p1, p2):
        self.assertTrue(
            pebble_commands.compare_points(p1, p2),
            f"({p1[0]}, {p1[1]}) != ({p2[0]}, {p2[1]})",
        )

    def test_convert_to_pebble_coordinates(self):
        p, valid = pebble_commands.convert_to_pebble_coordinates((0.1, -0.1))
        self.assertPointsEqual(p, (0.0, -1.0))
        self.assertFalse(valid)

        p, valid = pebble_commands.convert_to_pebble_coordinates((0.9, -0.9))
        self.assertPointsEqual(p, (0.0, -1.0))
        self.assertFalse(valid)

        p, valid = pebble_commands.convert_to_pebble_coordinates((1.0, -1.0))
        self.assertTrue(valid)
        self.assertPointsEqual(p, (1.0, -1.0))

        p, valid = pebble_commands.convert_to_pebble_coordinates((0.5, 0.5))
        self.assertTrue(valid)
        self.assertPointsEqual(p, (0.0, 0.0))

        p, valid = pebble_commands.convert_to_pebble_coordinates((1.0, 0.5))
        self.assertTrue(valid)
        self.assertPointsEqual(p, (1.0, 0.0))

        p, valid = pebble_commands.convert_to_pebble_coordinates((1.0, 0.5))
        self.assertTrue(valid)
        self.assertPointsEqual(p, (1.0, 0.0))

    def test_convert_to_pebble_precise_coordinates(self):
        p, valid = pebble_commands.convert_to_pebble_coordinates(
            (0.1, -0.1), precise=True
        )
        self.assertPointsEqual(p, (-3.0, -5.0))
        self.assertFalse(valid)

        p, valid = pebble_commands.convert_to_pebble_coordinates(
            (0.9, -0.9), precise=True
        )
        self.assertPointsEqual(p, (3.0, -11.0))
        self.assertFalse(valid)

        p, valid = pebble_commands.convert_to_pebble_coordinates(
            (1.0, -1.0), precise=True
        )
        self.assertTrue(valid)
        self.assertPointsEqual(p, (4.0, -12.0))

        p, valid = pebble_commands.convert_to_pebble_coordinates(
            (0.5, 0.5), precise=True
        )
        self.assertTrue(valid)
        self.assertPointsEqual(p, (0.0, 0.0))

        p, valid = pebble_commands.convert_to_pebble_coordinates(
            (1.0, 0.5), precise=True
        )
        self.assertTrue(valid)
        self.assertPointsEqual(p, (4.0, 0.0))

        p, valid = pebble_commands.convert_to_pebble_coordinates(
            (1.125, 0.75), precise=True
        )
        self.assertTrue(valid)
        self.assertPointsEqual(p, (5.0, 2.0))

        p, valid = pebble_commands.convert_to_pebble_coordinates(
            (0.25, -0.25), precise=True
        )
        self.assertTrue(valid)
        self.assertPointsEqual(p, (-2.0, -6.0))

    def test_find_nearest_valid_point(self):
        p = (-0.1, 0.1)
        self.assertTrue(
            pebble_commands.compare_points(
                pebble_commands.find_nearest_valid_point(p), (0.0, 0.0)
            )
        )

        p = (-0.3, 0.25)
        self.assertTrue(
            pebble_commands.compare_points(
                pebble_commands.find_nearest_valid_point(p), (-0.5, 0.5)
            )
        )

        p = (-0.3, -0.25)
        self.assertTrue(
            pebble_commands.compare_points(
                pebble_commands.find_nearest_valid_point(p), (-0.5, -0.5)
            )
        )

    def test_find_nearest_valid_precise_point(self):
        p = (-0.1, 0.1)
        self.assertTrue(
            pebble_commands.compare_points(
                pebble_commands.find_nearest_valid_precise_point(p), (-0.125, 0.125)
            )
        )

        p = (-0.3, 0.25)
        self.assertTrue(
            pebble_commands.compare_points(
                pebble_commands.find_nearest_valid_precise_point(p), (-0.25, 0.25)
            )
        )

        p = (1.05, -1.0625)
        self.assertTrue(
            pebble_commands.compare_points(
                pebble_commands.find_nearest_valid_precise_point(p), (1.0, -1.125)
            )
        )

    def test_path_serialize(self):
        points = [(1.5, 6.5), (-3.5, 2.5)]
        path = pebble_commands.PathCommand(
            points,
            True,
            (0, 0),
            1,
            pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF),
            pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF),
        )
        result = path.serialize()
        expected = path_example_output[:]
        self.assertEqual(result, array.array("B", expected).tobytes())

        points = [(1.5, 6.5), (-3.5, 2.5)]
        path = pebble_commands.PathCommand(
            points,
            False,
            (0, 0),
            1,
            pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF),
            pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF),
        )
        expected[5] = 0
        result = path.serialize()
        self.assertEqual(result, array.array("B", expected).tobytes())

    def test_circle_serialize(self):
        center = (-5.5, 6.5)
        circle = pebble_commands.CircleCommand(
            center,
            300,
            (0, 0),
            1,
            pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF),
            pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF),
        )
        result = circle.serialize()
        expected = circle_example_output
        self.assertEqual(result, array.array("B", expected).tobytes())

    def test_serialize_image(self):
        points = [(1.5, 6.5), (-3.5, 2.5)]
        path = pebble_commands.PathCommand(
            points,
            True,
            (0, 0),
            1,
            pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF),
            pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF),
        )
        center = (-5.5, 6.5)
        circle = pebble_commands.CircleCommand(
            center,
            300,
            (0, 0),
            1,
            pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF),
            pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF),
        )
        image = pebble_commands.serialize_image([path, circle], (10, 400))

        expected = [1, 0, 0x0A, 0x00, 0x90, 0x01, 0x02, 0x00]
        expected = array.array(
            "B", expected + path_example_output + circle_example_output
        ).tobytes()
        image_expected = b"PDCI" + pack("I", len(expected)) + expected

        self.assertEqual(image, image_expected)

    def test_serialize_sequence(self):
        points = [(1.5, 6.5), (-3.5, 2.5)]
        path1 = pebble_commands.PathCommand(
            points,
            True,
            (0, 0),
            1,
            pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF),
            pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF),
        )
        center = (-5.5, 6.5)
        circle1 = pebble_commands.CircleCommand(
            center,
            300,
            (0, 0),
            1,
            pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF),
            pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF),
        )
        frames = []
        frames.append([path1, circle1])
        points = [(1.5, 6.5), (-3.5, 2.5)]
        path2 = pebble_commands.PathCommand(
            points,
            False,
            (0, 0),
            1,
            pebble_commands.convert_color(0x00, 0x00, 0xFF, 0xFF),
            pebble_commands.convert_color(0xAA, 0xFF, 0x55, 0xFF),
        )
        center = (-5.5, 6.5)
        circle2 = pebble_commands.CircleCommand(
            center,
            280,
            (0, 0),
            1,
            pebble_commands.convert_color(0x00, 0xAA, 0xFF, 0xFF),
            pebble_commands.convert_color(0x55, 0xFF, 0x00, 0xFF),
        )
        frames.append([path2, circle2])
        seq = pebble_commands.serialize_sequence(frames, (10, 400), 33, 5)

        expected = [
            1,
            0,
            0x0A,
            0x00,
            0x90,
            0x01,
            0x05,
            0x00,
            0x02,
            0x00,
            0x21,
            0x00,
            0x02,
            0x00,
        ]
        expected += path_example_output + circle_example_output
        path_ex2 = path_example_output[:]
        path_ex2[2] = pebble_commands.convert_color(0x00, 0x00, 0xFF, 0xFF)
        path_ex2[4] = pebble_commands.convert_color(0xAA, 0xFF, 0x55, 0xFF)
        path_ex2[5] = 0
        circle_ex2 = circle_example_output
        circle_ex2[2] = pebble_commands.convert_color(0x00, 0xAA, 0xFF, 0xFF)
        circle_ex2[4] = pebble_commands.convert_color(0x55, 0xFF, 0x00, 0xFF)
        circle_ex2[5] = 0x18
        expected += [0x21, 0x00, 0x02, 0x00]
        expected += path_ex2 + circle_ex2
        seq_expected = (
            b"PDCS" + pack("I", len(expected)) + array.array("B", expected).tobytes()
        )
        self.assertEqual(seq, seq_expected)


if __name__ == "__main__":
    unittest.main()
