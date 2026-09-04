# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import unittest
import xml.etree.ElementTree as ET

# Allow us to run even if not at the `tools` directory.
root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.insert(0, root_dir)

from generate_pdcs import pebble_commands, svg2commands

svg_header = (
    '<svg version="1.1" id="Layer_1" xmlns="http://www.w3.org/2000/svg" '
    'xmlns:xlink="http://www.w3.org/1999/xlink" x="0px" y="0px" viewBox="0 0 144 168" '
    'enable-background="new 0 0 144 168" xml:space="preserve">'
)


def create_root(s):
    return ET.fromstring(svg_header + s + "</svg>")


def create_element(s):
    return next(iter(create_root(s)))


class MyTestCase(unittest.TestCase):
    def test_parse_path(self):
        # test basic vertical line path
        path_element = ET.fromstring('<path d="M-1.5,2.5v2"/>')
        command = svg2commands.parse_path(
            path_element,
            (0, 0),
            0,
            0,
            0,
            verbose=False,
            precise=False,
            raise_error=False,
        )
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(len(command.points), 2)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (-2.0, 2.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (-2.0, 4.0)),
            str(command.points[1]),
        )
        self.assertTrue(command.open)

        # test basic multi-line open path described as a sequence of points
        path_element = ET.fromstring('<path d="M -1.5,2.5 -3.5,6.5 4.5,6.5"/>')
        command = svg2commands.parse_path(
            path_element,
            (0, 0),
            0,
            0,
            0,
            verbose=False,
            precise=False,
            raise_error=False,
        )
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(len(command.points), 3)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (-2.0, 2.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (-4.0, 6.0)),
            str(command.points[1]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[2], (4.0, 6.0)),
            str(command.points[2]),
        )
        self.assertTrue(command.open)

        # test basic multi-line closed path described as a sequence of points
        path_element = ET.fromstring('<path d="M -1.5,2.5 -3.5,6.5 4.5,6.5 z"/>')
        command = svg2commands.parse_path(
            path_element,
            (0, 0),
            0,
            0,
            0,
            verbose=False,
            precise=False,
            raise_error=False,
        )
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(len(command.points), 3)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (-2.0, 2.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (-4.0, 6.0)),
            str(command.points[1]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[2], (4.0, 6.0)),
            str(command.points[2]),
        )
        self.assertFalse(command.open)

    def test_parse_circle(self):
        circle_element = ET.fromstring('<circle cx="72.5" cy="84.5" r="12"/>')
        command = svg2commands.parse_circle(
            circle_element,
            (0, 0),
            0,
            0,
            0,
            verbose=False,
            precise=False,
            raise_error=False,
        )
        self.assertIsInstance(command, pebble_commands.CircleCommand)
        self.assertEqual(len(command.points), 1)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (72.0, 84.0)),
            str(command.points[0]),
        )
        self.assertEqual(command.radius, 12.0)

    def test_parse_polyline(self):
        # test polyline element parsing
        polyline_element = ET.fromstring(
            '<polyline points="34.5,23.5 26.5,6.5 110.5,6.5 118.5,23.5 "/>'
        )
        command = svg2commands.parse_polyline(
            polyline_element,
            (0, 0),
            0,
            0,
            0,
            verbose=False,
            precise=False,
            raise_error=False,
        )
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(len(command.points), 4)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (34.0, 23.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (26.0, 6.0)),
            str(command.points[1]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[2], (110.0, 6.0)),
            str(command.points[2]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[3], (118.0, 23.0)),
            str(command.points[3]),
        )
        self.assertTrue(command.open)

    def test_parse_polygon(self):
        # test polygon (closed path) element parsing
        polygon_element = ET.fromstring(
            '<polygon points="34.5,23.5 26.5,6.5 110.5,6.5 118.5,23.5 "/>'
        )
        command = svg2commands.parse_polygon(
            polygon_element,
            (0, 0),
            0,
            0,
            0,
            verbose=False,
            precise=False,
            raise_error=False,
        )
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(len(command.points), 4)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (34.0, 23.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (26.0, 6.0)),
            str(command.points[1]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[2], (110.0, 6.0)),
            str(command.points[2]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[3], (118.0, 23.0)),
            str(command.points[3]),
        )
        self.assertFalse(command.open)

    def test_parse_line(self):
        # test line element parsing
        line_element = ET.fromstring(
            '<line x1="26.5" y1="139.5" x2="118.5" y2="139.5"/>'
        )
        command = svg2commands.parse_line(
            line_element,
            (0, 0),
            0,
            0,
            0,
            verbose=False,
            precise=False,
            raise_error=False,
        )
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(len(command.points), 2)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (26.0, 139.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (118.0, 139.0)),
            str(command.points[1]),
        )
        self.assertTrue(command.open)

    def test_parse_rect(self):
        # test rect element parsing (converts to a closed path)
        rect_element = ET.fromstring('<rect x="-1.5" y="2.5" width="4" height="5"/>')
        command = svg2commands.parse_rect(
            rect_element,
            (0, 0),
            0,
            0,
            0,
            verbose=False,
            precise=False,
            raise_error=False,
        )
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(len(command.points), 4)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (-2.0, 2.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (2.0, 2.0)),
            str(command.points[1]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[2], (2.0, 7.0)),
            str(command.points[2]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[3], (-2.0, 7.0)),
            str(command.points[3]),
        )
        self.assertFalse(command.open)

    def test_ignore_display_none(self):
        rect_element = create_element(
            '<g><rect x="-1.5" y="2.5" width="4" stroke="#0055FF" stroke-width="3" height="5"/></g>'
        )
        commands, _ = svg2commands.get_commands((0, 0), rect_element, precise=True)
        self.assertEqual(len(commands), 1)
        rect_element_none = create_element(
            '<g><rect x="-1.5" y="2.5" width="4" stroke="#0055FF" stroke-width="3" height="5" display="none"/></g>'
        )
        commands, _ = svg2commands.get_commands((0, 0), rect_element_none, precise=True)
        self.assertEqual(len(commands), 0)

    def test_create_command(self):
        element = create_element(
            '<polyline fill="#AAFF00" stroke="#0055FF" stroke-width="3" '
            'points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        # test that PathCommand is created from element correctly
        command = svg2commands.create_command((0, 0), element)
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(len(command.points), 3)
        self.assertEqual(
            command.fill_color, pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF)
        )
        self.assertEqual(
            command.stroke_color, pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF)
        )
        self.assertEqual(command.stroke_width, 3)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (34.0, 23.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (26.0, 6.0)),
            str(command.points[1]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[2], (110.0, 6.0)),
            str(command.points[2]),
        )
        self.assertTrue(command.open)

        # element should not be created if no color is specified (everything is transparent)
        element = create_element(
            '<polyline stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        command = svg2commands.create_command((0, 0), element)
        self.assertTrue(command is None)

        # element with no stroke width is created
        element = create_element(
            '<polyline fill="#AAFF00" stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        command = svg2commands.create_command((0, 0), element)
        self.assertEqual(command.stroke_color, 0)
        self.assertEqual(command.stroke_width, 0)
        self.assertEqual(
            command.fill_color, pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF)
        )

        # element with no fill is created
        element = create_element(
            '<polyline stroke="#0055FF" stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        command = svg2commands.create_command((0, 0), element)
        self.assertEqual(
            command.stroke_color, pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF)
        )
        self.assertEqual(command.stroke_width, 3)
        self.assertEqual(command.fill_color, 0)

        # test that opacity is assigned correctly (use opacity = 0.34 to ensure truncation to (255 / 3))
        element = create_element(
            '<polyline fill-opacity="0.34" stroke-opacity="0.5" fill="#AAFF00" stroke="#0055FF" '
            'stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        command = svg2commands.create_command((0, 0), element)
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(
            command.fill_color, pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0x55)
        )
        self.assertEqual(
            command.stroke_color, pebble_commands.convert_color(0x00, 0x55, 0xFF, 0x55)
        )
        self.assertEqual(command.stroke_width, 3)

        # test that opacity is compounded when 'opacity' tag is included
        element = create_element(
            '<polyline opacity="0.34" fill-opacity="0.67" stroke-opacity="1.0" fill="#AAFF00" '
            'stroke="#0055FF" stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        command = svg2commands.create_command((0, 0), element)
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(command.fill_color, 0)
        self.assertEqual(
            command.stroke_color, pebble_commands.convert_color(0x00, 0x55, 0xFF, 0x55)
        )
        self.assertEqual(command.stroke_width, 3)

        # stroke color should be set to clear when width is 0
        element = create_element(
            '<polyline fill="#AAFF00" stroke="#0055FF" stroke-width="0" '
            'points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        command = svg2commands.create_command((0, 0), element)
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(
            command.fill_color, pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF)
        )
        self.assertEqual(command.stroke_color, 0)
        self.assertEqual(command.stroke_width, 0)

        # test that all points are shifted when a translation is specified
        element = create_element(
            '<polyline fill="#AAFF00" stroke="#0055FF" stroke-width="3" '
            'points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        translate = (1, -2)
        command = svg2commands.create_command(translate, element)
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(len(command.points), 3)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (35.0, 21.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (27.0, 4.0)),
            str(command.points[1]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[2], (111.0, 4.0)),
            str(command.points[2]),
        )
        self.assertTrue(command.open)

        # test element other than polyline
        element = create_element(
            '<line fill="none" stroke="#0000FF" stroke-width="5" x1="26.5" y1="139.5" x2="118.5" '
            'y2="139.5"/>'
        )
        command = svg2commands.create_command((0, 0), element)
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(command.fill_color, 0)
        self.assertEqual(
            command.stroke_color, pebble_commands.convert_color(0x00, 0x00, 0xFF, 0xFF)
        )
        self.assertEqual(command.stroke_width, 5)
        self.assertEqual(len(command.points), 2)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (26.0, 139.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (118.0, 139.0)),
            str(command.points[1]),
        )

        # elements at x.0 should stay at x.0
        element = create_element(
            '<line fill="none" stroke="#0000FF" stroke-width="5" x1="26.0" y1="139.5" x2="118.5" '
            'y2="139.0"/>'
        )
        command = svg2commands.create_command((0, 0), element)
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(command.fill_color, 0)
        self.assertEqual(
            command.stroke_color, pebble_commands.convert_color(0x00, 0x00, 0xFF, 0xFF)
        )
        self.assertEqual(command.stroke_width, 5)
        self.assertEqual(len(command.points), 2)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (26.0, 139.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (118.0, 139.0)),
            str(command.points[1]),
        )

        # test elements that do not describe a path or circle are handled
        element = create_element('<layer bla="5"/>')
        command = svg2commands.create_command((0, 0), element)
        self.assertTrue(command is None)

    def test_parse_svg(self):
        root = create_root(
            '<line fill="none" stroke="#0000FF" stroke-width="5" x1="1.5" y1="5.5" x2="-6.5" y2="-1.5"/>'
            '<polyline fill="#AAFF00" stroke="#0055FF" stroke-width="3" points="-3.5,4.5 9.5,6.5 '
            '7.5,6.5"/>'
        )
        translate = (-1, 2)
        commands, _ = svg2commands.get_commands(translate, root, precise=False)
        self.assertEqual(len(commands[0].points), 2)
        self.assertTrue(
            pebble_commands.compare_points(commands[0].points[0], (0.0, 7.0)),
            str(commands[0].points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(commands[0].points[1], (-8.0, 0.0)),
            str(commands[0].points[1]),
        )
        self.assertEqual(commands[0].fill_color, 0)
        self.assertEqual(
            commands[0].stroke_color,
            pebble_commands.convert_color(0x00, 0x00, 0xFF, 0xFF),
        )
        self.assertEqual(commands[0].stroke_width, 5)
        self.assertEqual(len(commands[1].points), 3)
        self.assertTrue(
            pebble_commands.compare_points(commands[1].points[0], (-5.0, 6.0)),
            str(commands[1].points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(commands[1].points[1], (8.0, 8.0)),
            str(commands[1].points[1]),
        )
        self.assertTrue(
            pebble_commands.compare_points(commands[1].points[2], (6.0, 8.0)),
            str(commands[1].points[2]),
        )
        self.assertEqual(
            commands[1].fill_color,
            pebble_commands.convert_color(0xAA, 0xFF, 0x00, 0xFF),
        )
        self.assertEqual(
            commands[1].stroke_color,
            pebble_commands.convert_color(0x00, 0x55, 0xFF, 0xFF),
        )
        self.assertEqual(commands[1].stroke_width, 3)

    def test_get_info(self):
        xml = ET.fromstring(svg_header + "</svg>")
        translate, size = svg2commands.get_info(xml)
        self.assertEqual(translate[0], 0)
        self.assertEqual(translate[1], 0)
        self.assertEqual(size[0], 144)
        self.assertEqual(size[1], 168)

    def test_parse_precise_path(self):
        # test that sub-pixel precision is achieved by scaling up by a factor of 8 for precise paths
        line_element = ET.fromstring('<line x1="1.5" y1="2.125" x2="4.875" y2="9"/>')
        command = svg2commands.parse_line(
            line_element, (0, 0), 0, 0, 0, verbose=False, precise=True, raise_error=True
        )
        self.assertIsInstance(command, pebble_commands.PathCommand)
        self.assertEqual(len(command.points), 2)
        self.assertTrue(
            pebble_commands.compare_points(command.points[0], (8.0, 13.0)),
            str(command.points[0]),
        )
        self.assertTrue(
            pebble_commands.compare_points(command.points[1], (35.0, 68.0)),
            str(command.points[1]),
        )
        self.assertTrue(command.open)
        self.assertEqual(command.type, pebble_commands.DRAW_COMMAND_TYPE_PRECISE_PATH)

    def test_parse_with_error_raising(self):
        # test that inaccurate points are rejected with raise_error == True (pebble_commands.InvalidPointException raised)
        line_element = ET.fromstring('<line x1="1.5" y1="2.1" x2="4.6" y2="9"/>')
        with self.assertRaises(svg2commands.pebble_commands.InvalidPointException):
            svg2commands.parse_line(
                line_element,
                (0, 0),
                0,
                0,
                0,
                verbose=False,
                precise=True,
                raise_error=True,
            )

        # test that using precise == False results in points being rejected with raise_error == True
        # (pebble_commands.InvalidPointException raised)
        line_element = ET.fromstring('<line x1="1.5" y1="2.125" x2="4.875" y2="9"/>')
        with self.assertRaises(svg2commands.pebble_commands.InvalidPointException):
            svg2commands.parse_line(
                line_element,
                (0, 0),
                0,
                0,
                0,
                verbose=False,
                precise=False,
                raise_error=True,
            )


if __name__ == "__main__":
    unittest.main()
