import os
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.abspath(".."))
from pblconvert.svg2pdc import pdc
from pblconvert.svg2pdc.svg import surface_from_svg

root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.insert(0, root_dir)


svg_header = (
    '<?xml version="1.0" encoding="utf-8"?><svg version="1.1" id="Layer_1" xmlns="http://www.w3.org/2000/svg" '
    'xmlns:xlink="http://www.w3.org/1999/xlink" x="0px" y="0px" viewBox="0 0 144 168" '
    'enable-background="new 0 0 144 168" xml:space="preserve">'
)
svg_footer = "</svg>"


def create_root(s):
    return ET.fromstring(svg_header + s + "</svg>")


def create_element(s):
    return create_root(s).getchildren()[0]


def parse_svg_element(svg_text, precise=False, raise_error=False, translate=None):
    return (parse_svg_elements(svg_text) + [None])[0]


def parse_svg_elements(svg_text, translate=None):
    svg = svg_header + svg_text + svg_footer
    surface = surface_from_svg(bytestring=svg)
    return surface.pdc_commands


class MyTestCase(unittest.TestCase):
    def test_parse_path(self):
        # test basic vertical line path
        path_element = '<path d="M-1.5,2.5v2" stroke="black" />'
        command = parse_svg_element(path_element, precise=False, raise_error=False)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(len(command.points), 2)
        self.assertTrue(
            pdc.compare_points(command.points[0], (-1.5, 2.5)), str(command.points[0])
        )
        self.assertTrue(
            pdc.compare_points(command.points[1], (-1.5, 4.5)), str(command.points[1])
        )
        self.assertTrue(command.open)

        # test basic multi-line open path described as a sequence of points
        path_element = '<path d="M -1.5,2.5 -3.5,6.5 4.5,6.5" stroke="black" />'
        command = parse_svg_element(path_element, precise=False, raise_error=False)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(len(command.points), 3)
        self.assertTrue(
            pdc.compare_points(command.points[0], (-1.5, 2.5)), str(command.points[0])
        )
        self.assertTrue(
            pdc.compare_points(command.points[1], (-3.5, 6.5)), str(command.points[1])
        )
        self.assertTrue(
            pdc.compare_points(command.points[2], (4.5, 6.5)), str(command.points[2])
        )
        self.assertTrue(command.open)

        # test basic multi-line closed path described as a sequence of points
        path_element = '<path d="M -1.5,2.5 -3.5,6.5 4.5,6.5 z" stroke="black" />'
        command = parse_svg_element(path_element, precise=False, raise_error=False)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(len(command.points), 3)
        self.assertTrue(
            pdc.compare_points(command.points[0], (-1.5, 2.5)), str(command.points[0])
        )
        self.assertTrue(
            pdc.compare_points(command.points[1], (-3.5, 6.5)), str(command.points[1])
        )
        self.assertTrue(
            pdc.compare_points(command.points[2], (4.5, 6.5)), str(command.points[2])
        )
        self.assertFalse(command.open)

    def test_parse_circle(self):
        circle_element = '<circle cx="72.5" cy="84.5" r="12" fill="black" />'
        command = parse_svg_element(circle_element, precise=False, raise_error=False)
        self.assertIsInstance(command, pdc.CircleCommand)
        self.assertEqual(len(command.points), 1)
        self.assertTrue(
            pdc.compare_points(command.points[0], (72.5, 84.5)), str(command.points[0])
        )
        self.assertEqual(command.radius, 12.0)

    def test_parse_polyline(self):
        # test polyline element parsing
        polyline_element = '<polyline points="34.5,23.5 26.5,6.5 110.5,6.5 118.5,23.5 " stroke="yellow" />'
        command = parse_svg_element(polyline_element, precise=False, raise_error=False)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(len(command.points), 4)
        self.assertTrue(
            pdc.compare_points(command.points[0], (34.5, 23.5)), str(command.points[0])
        )
        self.assertTrue(
            pdc.compare_points(command.points[1], (26.5, 6.5)), str(command.points[1])
        )
        self.assertTrue(
            pdc.compare_points(command.points[2], (110.5, 6.5)), str(command.points[2])
        )
        self.assertTrue(
            pdc.compare_points(command.points[3], (118.5, 23.5)), str(command.points[3])
        )
        self.assertTrue(command.open)

    def test_parse_polygon(self):
        # test polygon (closed path) element parsing
        polygon_element = (
            '<polygon points="34.5,23.5 26.5,6.5 110.5,6.5 118.5,23.5 " fill="green"/>'
        )
        command = parse_svg_element(polygon_element, precise=False, raise_error=False)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(len(command.points), 4)
        self.assertTrue(
            pdc.compare_points(command.points[0], (34.5, 23.5)), str(command.points[0])
        )
        self.assertTrue(
            pdc.compare_points(command.points[1], (26.5, 6.5)), str(command.points[1])
        )
        self.assertTrue(
            pdc.compare_points(command.points[2], (110.5, 6.5)), str(command.points[2])
        )
        self.assertTrue(
            pdc.compare_points(command.points[3], (118.5, 23.5)), str(command.points[3])
        )
        self.assertFalse(command.open)

    def test_parse_line(self):
        # test line element parsing
        line_element = (
            '<line x1="26.5" y1="139.5" x2="118.5" y2="139.5" stroke="blue" />'
        )
        command = parse_svg_element(line_element, precise=False, raise_error=False)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(len(command.points), 2)
        self.assertTrue(
            pdc.compare_points(command.points[0], (26.5, 139.5)), str(command.points[0])
        )
        self.assertTrue(
            pdc.compare_points(command.points[1], (118.5, 139.5)),
            str(command.points[1]),
        )
        self.assertTrue(command.open)

    def test_parse_rect(self):
        # test rect element parsing (converts to a closed path)
        rect_element = '<rect x="-1.5" y="2.5" width="4" height="5" fill="blue"/>'
        command = parse_svg_element(rect_element, precise=False, raise_error=False)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(len(command.points), 4)
        self.assertTrue(
            pdc.compare_points(command.points[0], (-1.5, 2.5)), str(command.points[0])
        )
        self.assertTrue(
            pdc.compare_points(command.points[1], (2.5, 2.5)), str(command.points[1])
        )
        self.assertTrue(
            pdc.compare_points(command.points[2], (2.5, 7.5)), str(command.points[2])
        )
        self.assertTrue(
            pdc.compare_points(command.points[3], (-1.5, 7.5)), str(command.points[3])
        )
        self.assertFalse(command.open)

    def test_ignore_display_none(self):
        commands = parse_svg_elements(
            '<g><rect x="-1.5" y="2.5" width="4" stroke="#0055FF" stroke-width="3" height="5"/></g>'
        )
        self.assertEqual(len(commands), 1)
        commands = parse_svg_elements(
            '<g><rect x="-1.5" y="2.5" width="4" stroke="#0055FF" stroke-width="3" height="5" display="none"/></g>'
        )
        self.assertEqual(len(commands), 0)
        commands = parse_svg_elements(
            '<g display="none"><rect x="-1.5" y="2.5" width="4" stroke="#0055FF" stroke-width="3" height="5"/></g>'
        )
        self.assertEqual(len(commands), 0)
        commands = parse_svg_elements(
            '<g visibility="hidden"><rect x="-1.5" y="2.5" width="4" stroke="#0055FF" stroke-width="3" height="5"/></g>'
        )
        self.assertEqual(len(commands), 0)

    def test_create_command(self):
        element = '<polyline fill="#AAFF00" stroke="#0055FF" stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        # test that PathCommand is created from element correctly
        command = parse_svg_element(element)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(len(command.points), 3)
        self.assertEqual(command.fill_color, pdc.convert_color(0xAA, 0xFF, 0x00, 0xFF))
        self.assertEqual(
            command.stroke_color, pdc.convert_color(0x00, 0x55, 0xFF, 0xFF)
        )
        self.assertEqual(command.stroke_width, 3)
        self.assertTrue(
            pdc.compare_points(command.points[0], (34.5, 23.5)), str(command.points[0])
        )
        self.assertTrue(
            pdc.compare_points(command.points[1], (26.5, 6.5)), str(command.points[1])
        )
        self.assertTrue(
            pdc.compare_points(command.points[2], (110.5, 6.5)), str(command.points[2])
        )
        self.assertTrue(command.open)

        # element should not be created if no color is specified (everything is transparent)
        element = '<polyline stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        command = parse_svg_element(element)
        self.assertTrue(command is None)

        # element with no stroke width is created
        element = '<polyline fill="#AAFF00" stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        command = parse_svg_element(element)
        self.assertEqual(command.stroke_color, 0)
        self.assertEqual(command.stroke_width, 0)
        self.assertEqual(command.fill_color, pdc.convert_color(0xAA, 0xFF, 0x00, 0xFF))

        # element with no fill is created
        element = '<polyline stroke="#0055FF" stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        command = parse_svg_element(element)
        self.assertEqual(
            command.stroke_color, pdc.convert_color(0x00, 0x55, 0xFF, 0xFF)
        )
        self.assertEqual(command.stroke_width, 3)
        self.assertEqual(command.fill_color, 0)

        # test that opacity is assigned correctly (use opacity = 0.34 to ensure truncation to (255 / 3))
        element = (
            '<polyline fill-opacity="0.34" stroke-opacity="0.5" fill="#AAFF00" stroke="#0055FF" '
            'stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        command = parse_svg_element(element)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(command.fill_color, pdc.convert_color(0xAA, 0xFF, 0x00, 0x55))
        self.assertEqual(
            command.stroke_color, pdc.convert_color(0x00, 0x55, 0xFF, 0x55)
        )
        self.assertEqual(command.stroke_width, 3)

        # test that opacity is compounded when 'opacity' tag is included
        element = (
            '<polyline opacity="0.34" fill-opacity="0.67" stroke-opacity="1.0" fill="#AAFF00" '
            'stroke="#0055FF" stroke-width="3" points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        command = parse_svg_element(element)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(command.fill_color, 0)
        self.assertEqual(
            command.stroke_color, pdc.convert_color(0x00, 0x55, 0xFF, 0x55)
        )
        self.assertEqual(command.stroke_width, 3)

        # stroke color should be set to clear when width is 0
        element = (
            '<polyline fill="#AAFF00" stroke="#0055FF" stroke-width="0" '
            'points="34.5,23.5 26.5,6.5 110.5,6.5"/>'
        )
        command = parse_svg_element(element)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(command.fill_color, pdc.convert_color(0xAA, 0xFF, 0x00, 0xFF))
        self.assertEqual(command.stroke_color, 0)
        self.assertEqual(command.stroke_width, 0)

        # test element other than polyline
        element = '<line fill="none" stroke="#0000FF" stroke-width="5" x1="26.0" y1="139.5" x2="118.5" y2="139.0"/>'
        command = parse_svg_element(element)
        self.assertIsInstance(command, pdc.PathCommand)
        self.assertEqual(command.fill_color, 0)
        self.assertEqual(
            command.stroke_color, pdc.convert_color(0x00, 0x00, 0xFF, 0xFF)
        )
        self.assertEqual(command.stroke_width, 5)
        self.assertEqual(len(command.points), 2)
        self.assertTrue(
            pdc.compare_points(command.points[0], (26.0, 139.5)), str(command.points[0])
        )
        self.assertTrue(
            pdc.compare_points(command.points[1], (118.5, 139.0)),
            str(command.points[1]),
        )

        # test elements that do not describe a path or circle are handled
        element = '<layer bla="5"/>'
        command = parse_svg_element(element)
        self.assertTrue(command is None)

    def test_parse_svg(self):
        root = (
            '<line fill="none" stroke="#0000FF" stroke-width="5" x1="1.5" y1="5.5" x2="-6.5" y2="-1.5"/>'
            '<polyline fill="#AAFF00" stroke="#0055FF" stroke-width="3" points="-3.5,4.5 9.5,6.5 '
            '7.5,6.5"/>'
        )
        commands = parse_svg_elements(root)
        self.assertEqual(len(commands), 2)
        self.assertEqual(len(commands[0].points), 2)
        self.assertTrue(
            pdc.compare_points(commands[0].points[0], (1.5, 5.5)),
            str(commands[0].points[0]),
        )
        self.assertTrue(
            pdc.compare_points(commands[0].points[1], (-6.5, -1.5)),
            str(commands[0].points[1]),
        )
        self.assertEqual(commands[0].fill_color, 0)
        self.assertEqual(
            commands[0].stroke_color, pdc.convert_color(0x00, 0x00, 0xFF, 0xFF)
        )
        self.assertEqual(commands[0].stroke_width, 5)
        self.assertEqual(len(commands[1].points), 3)
        self.assertTrue(
            pdc.compare_points(commands[1].points[0], (-3.5, 4.5)),
            str(commands[1].points[0]),
        )
        self.assertTrue(
            pdc.compare_points(commands[1].points[1], (9.5, 6.5)),
            str(commands[1].points[1]),
        )
        self.assertTrue(
            pdc.compare_points(commands[1].points[2], (7.5, 6.5)),
            str(commands[1].points[2]),
        )
        self.assertEqual(
            commands[1].fill_color, pdc.convert_color(0xAA, 0xFF, 0x00, 0xFF)
        )
        self.assertEqual(
            commands[1].stroke_color, pdc.convert_color(0x00, 0x55, 0xFF, 0xFF)
        )
        self.assertEqual(commands[1].stroke_width, 3)

    def test_transformed_circle(self):
        circle_element = '<circle cx="72.5" cy="84.5" r="12" fill="black" transform="translate(12,34)"/>'
        command = parse_svg_element(circle_element, precise=False, raise_error=False)
        self.assertIsInstance(command, pdc.CircleCommand)
        self.assertEqual(len(command.points), 1)
        self.assertTrue(
            pdc.compare_points(command.points[0], (12 + 72.5, 34 + 84.5)),
            str(command.points[0]),
        )
        self.assertEqual(command.radius, 12.0)

    def test_scaled_stroke(self):
        strokes = (
            '<g transform="scale(9,1)">'
            '<line stroke="black" stroke-width="5" x1="10" y1="50" x2="10" y2="350"/>'
            '<line vector-effect="non-scaling-stroke" stroke="black" stroke-width="5" x1="32" y1="50" x2="32" y2="350"/>'
            '<line vector-effect="none" stroke="black" stroke-width="5" x1="55" y1="50" x2="55" y2="350"/>'
            "</g>"
        )
        commands = parse_svg_elements(strokes)
        self.assertEqual(3, len(commands))
        self.assertEqual(9 * 5, commands[0].stroke_width)
        self.assertEqual(5, commands[1].stroke_width)
        self.assertEqual(9 * 5, commands[2].stroke_width)

    def test_empty_width_height(self):
        svg = (
            '<?xml version="1.0" encoding="utf-8"?>'
            '<svg version="1.1" xmlns="http://www.w3.org/2000/svg" '
            'viewBox="20 30 40 50">'
            '<rect x="1" y="1" width="5" height="5" stroke="black"/></svg>'
        )
        surface = surface_from_svg(bytestring=svg)
        self.assertEqual(surface.size(), (40, 50))
        self.assertEqual(surface.pdc_commands[0].bounding_box(), (-19, -29, 5, 5))

    def test_only_width_height(self):
        svg = (
            '<?xml version="1.0" encoding="utf-8"?>'
            '<svg version="1.1" xmlns="http://www.w3.org/2000/svg" '
            'width="60" height="70">'
            '<rect x="1" y="1" width="5" height="5" stroke="black"/></svg>'
        )
        surface = surface_from_svg(bytestring=svg)
        self.assertEqual(surface.size(), (60, 70))
        self.assertEqual(surface.pdc_commands[0].bounding_box(), (1, 1, 5, 5))

    def test_width_height_and_box(self):
        svg = (
            '<?xml version="1.0" encoding="utf-8"?>'
            '<svg version="1.1" xmlns="http://www.w3.org/2000/svg" '
            'viewBox="20 30 40 50" width="60" height="70">'
            '<rect x="1" y="1" width="5" height="5" stroke="black"/></svg>'
        )
        surface = surface_from_svg(bytestring=svg)
        self.assertEqual(surface.size(), (60, 70))
        expected = (-22.73333333333333, -40.6, 7.0, 7.0)
        self.assertEqual(surface.pdc_commands[0].bounding_box(), expected)

    def test_neither_size_nor_viewbox(self):
        svg = (
            '<?xml version="1.0" encoding="utf-8"?>'
            '<svg version="1.1" xmlns="http://www.w3.org/2000/svg" '
            ">"
            '<rect x="1" y="1" width="5" height="5" stroke="black"/></svg>'
        )
        surface = surface_from_svg(bytestring=svg)
        self.assertEqual(surface.size(), (6, 6))
        self.assertEqual(surface.pdc_commands[0].bounding_box(), (1, 1, 5, 5))

    def test_zero_viewbox(self):
        svg = (
            '<?xml version="1.0" encoding="utf-8"?>'
            '<svg version="1.1" xmlns="http://www.w3.org/2000/svg" '
            'viewBox="20 30 0 0">'
            '<rect x="1" y="1" width="5" height="5" stroke="black"/></svg>'
        )
        surface = surface_from_svg(bytestring=svg)
        self.assertEqual(surface.size(), (0, 0))
        self.assertEqual(0, len(surface.pdc_commands))

    def test_ignore_non_px_size(self):
        svg = (
            '<?xml version="1.0" encoding="utf-8"?>'
            '<svg version="1.1" xmlns="http://www.w3.org/2000/svg" '
            'width="12cm" height="4cm" viewBox="0 0 1200 400">'
            '<rect x="1" y="1" width="5" height="5" stroke="black"/></svg>'
        )
        surface = surface_from_svg(bytestring=svg)
        self.assertEqual(surface.size(), (1200, 400))

    def test_adds_viewbox_if_missing(self):
        svg = (
            '<?xml version="1.0" encoding="utf-8"?>'
            '<svg version="1.1" xmlns="http://www.w3.org/2000/svg">'
            '<rect x="1" y="1" width="5" height="5" stroke="black"/></svg>'
        )
        surface = surface_from_svg(bytestring=svg)
        self.assertEqual(surface.size(), (6, 6))
        et = surface.element_tree()
        self.assertEqual(et.getroot().get("viewBox"), "0 0 6 6")

    def test_preserves_viewbox_if_exists(self):
        svg = (
            '<?xml version="1.0" encoding="utf-8"?>'
            '<svg version="1.1" xmlns="http://www.w3.org/2000/svg" viewBox="1 2 3 4">'
            '<rect x="1" y="1" width="5" height="5" stroke="black"/></svg>'
        )
        surface = surface_from_svg(bytestring=svg)
        self.assertEqual(surface.size(), (3, 4))
        et = surface.element_tree()
        self.assertEqual(et.getroot().get("viewBox"), "1 2 3 4")


if __name__ == "__main__":
    unittest.main()
