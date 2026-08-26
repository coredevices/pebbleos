import os
import shutil
import sys
import tempfile
from struct import pack
from subprocess import PIPE, Popen

from pebble_image_routines import (
    nearest_color_to_pebble64_palette,
    rgba32_triplet_to_argb8,
    truncate_color_to_pebble64_palette,
)

DRAW_COMMAND_VERSION = 1
DRAW_COMMAND_TYPE_PATH = 1
DRAW_COMMAND_TYPE_CIRCLE = 2
DRAW_COMMAND_TYPE_PRECISE_PATH = 3

epsilon = sys.float_info.epsilon


def valid_color(r, g, b, a):
    return (
        (r <= 0xFF)
        and (g <= 0xFF)
        and (b <= 0xFF)
        and (a <= 0xFF)
        and (r >= 0x00)
        and (g >= 0x00)
        and (b >= 0x00)
        and (a >= 0x00)
    )


def convert_color(r, g, b, a, truncate=True):

    valid = valid_color(r, g, b, a)
    if not valid:
        print(f"Invalid color: ({r}, {g}, {b}, {a})")
        return 0

    if truncate:
        (r, g, b, a) = truncate_color_to_pebble64_palette(r, g, b, a)
    else:
        (r, g, b, a) = nearest_color_to_pebble64_palette(r, g, b, a)

    return rgba32_triplet_to_argb8(r, g, b, a)


def sum_points(p1, p2):
    return p1[0] + p2[0], p1[1] + p2[1]


def subtract_points(p1, p2):
    return p1[0] - p2[0], p1[1] - p2[1]


def round_point(p):
    return round(p[0] + epsilon), round(
        p[1] + epsilon
    )  # hack to get around the fact that python rounds negative
    # numbers downwards


def scale_point(p, factor):
    return p[0] * factor, p[1] * factor


def find_nearest_valid_point(p):
    return (round(p[0] * 2.0) / 2.0), (round(p[1] * 2.0) / 2.0)


def find_nearest_valid_precise_point(p):
    return (round(p[0] * 8.0) / 8.0), (round(p[1] * 8.0) / 8.0)


def convert_to_pebble_coordinates(point, precise=False):
    # convert from graphic tool coordinate system to pebble coordinate system so that they render the same on
    # both

    if not precise:
        nearest = find_nearest_valid_point(
            point
        )  # used to give feedback to user if the point shifts considerably
    else:
        nearest = find_nearest_valid_precise_point(point)

    problem = (
        None
        if compare_points(point, nearest)
        else f"Invalid point: ({point[0]:.2f}, {point[1]:.2f}). Used closest supported coordinate: ({nearest[0]}, {nearest[1]})"
    )

    translated = sum_points(point, (-0.5, -0.5))  # translate point by (-0.5, -0.5)
    if precise:
        translated = scale_point(translated, 8)  # scale point for precise coordinates
    rounded = round_point(translated)

    return rounded, problem


def compare_points(p1, p2):
    return p1[0] == p2[0] and p1[1] == p2[1]


class InvalidPointException(Exception):
    pass


def bounding_box_around_points(points):
    result = None
    for p in points:
        result = extend_bounding_box(result, p)
    return result


def extend_bounding_box(rect, point=None, rect2=None):
    if rect is None:
        return rect2 if rect2 is not None else (point[0], point[1], 0, 0)

    if rect2 is not None:
        top_left = (rect2[0], rect2[1])
        bottom_right = (rect2[0] + rect2[2], rect2[1] + rect2[3])
        rect = extend_bounding_box(rect, point=top_left)
        rect = extend_bounding_box(rect, point=bottom_right)
        return rect

    assert point is not None
    min_x = min(rect[0], point[0])
    min_y = min(rect[1], point[1])
    max_x = max(rect[0] + rect[2], point[0])
    max_y = max(rect[1] + rect[3], point[1])
    return (min_x, min_y, max_x - min_x, max_y - min_y)


PDC2PNG = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../bin/pdc2png")


def convert_to_png(pdc_data):
    tmp_dir = tempfile.mkdtemp()
    try:
        pdc_path = os.path.join(tmp_dir, "image.pdc")
        with open(pdc_path, "wb") as pdc_file:
            pdc_file.write(pdc_data)

        cmd = "%s %s" % (PDC2PNG, pdc_path)
        p = Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE)
        stdout, stderr = p.communicate()
        if p.returncode != 0:
            raise OSError(stderr)

        png_path = os.path.join(tmp_dir, "image.png")
        with open(png_path, "rb") as png_file:
            return png_file.read()
    finally:
        shutil.rmtree(tmp_dir)


class Command:
    """Serialized draw command.

    Format spec: docs/reference/formats/pdc.md; the canonical serializers
    live in tools/generate_pdcs/pebble_commands.py.
    """

    def __init__(
        self, points, stroke_width=0, stroke_color=0, fill_color=0, raise_error=False
    ):
        # for i in range(len(points)):
        #     points[i], valid = convert_to_pebble_coordinates(points[i], precise)
        #     if not valid and raise_error:
        #         raise InvalidPointException("Invalid point in command")

        self.points = list(points)
        self.stroke_width = stroke_width
        self.stroke_color = stroke_color
        self.fill_color = fill_color

    def is_precise(self):
        return False

    def transform(self, transformer):
        self.points = list([transformer.transform_point(p) for p in self.points])

    def finalize(self, annotator):
        grid_annotation = None
        for p in self.points:
            converted, problem = convert_to_pebble_coordinates(p, self.is_precise())

            if problem is not None:
                if grid_annotation is None:
                    link = (
                        "https://pebbleos-core.readthedocs.io/en/latest/"
                        "reference/formats/pdc.html#coordinate-grid"
                    )
                    grid_annotation = annotator.add_annotation(
                        "Element is expressed with unsupported coordinate(s).",
                        link=link,
                    )
                grid_annotation.add_highlight(p[0], p[1], details=problem)


    def bounding_box(self):
        result = None
        for p in self.points:
            result = extend_bounding_box(result, point=p)
        return result

    def serialize_common(self):
        return pack(
            "<BBBB",
            0,  # reserved byte
            self.stroke_color,
            self.stroke_width,
            self.fill_color,
        )

    def serialize_points(self):
        s = pack("<H", len(self.points))  # number of points (16-bit)
        for p in self.points:
            converted, _ = convert_to_pebble_coordinates(p, self.is_precise())
            s += pack(
                "<hh",
                int(converted[0]),  # x (16-bit)
                int(converted[1]),
            )  # y (16-bit)
        return s


class PathCommand(Command):
    def __init__(
        self,
        points,
        path_open,
        stroke_width=0,
        stroke_color=0,
        fill_color=0,
        precise=False,
        raise_error=False,
    ):
        self.open = path_open
        self.type = (
            DRAW_COMMAND_TYPE_PATH if not precise else DRAW_COMMAND_TYPE_PRECISE_PATH
        )
        Command.__init__(
            self, points, stroke_width, stroke_color, fill_color, raise_error
        )

    def is_precise(self):
        return self.type == DRAW_COMMAND_TYPE_PRECISE_PATH

    def serialize(self):
        s = pack("B", self.type)  # command type
        s += self.serialize_common()
        s += pack(
            "<BB",
            int(self.open),  # open path boolean
            0,
        )  # unused byte in path
        s += self.serialize_points()
        return s

    def __str__(self):
        points = self.points[:]
        if self.type == DRAW_COMMAND_TYPE_PRECISE_PATH:
            type = "P"
            for i in range(len(points)):
                points[i] = scale_point(points[i], 0.125)
        else:
            type = ""
        return (
            f"Path: [fill color:{self.fill_color}; stroke color:{self.stroke_color}; stroke width:{self.stroke_width}] {points} {self.open} {type}"
        )


class CircleCommand(Command):
    def __init__(self, center, radius, stroke_width=0, stroke_color=0, fill_color=0):
        points = [(center[0], center[1])]
        Command.__init__(self, points, stroke_width, stroke_color, fill_color)
        self.radius = radius

    def transform(self, transformer):
        super().transform(transformer)

        (dx, dy) = transformer.transform_distance(self.radius, self.radius)
        self.radius = min(dx, dy)
        if dx != dy:
            annotation = transformer.add_annotation(
                "Only rigid transformations for circles are supported.",
                transformed=True,
            )
            center = self.points[0]
            annotation.add_highlight(center[0] - dx, center[1] - dy, dx * 2, dy * 2)

    def serialize(self):
        s = pack("B", DRAW_COMMAND_TYPE_CIRCLE)  # command type
        s += self.serialize_common()
        s += pack("<H", int(self.radius))  # circle radius (16-bit)
        s += self.serialize_points()
        return s

    def __str__(self):
        return f"Circle: [fill color:{self.fill_color}; stroke color:{self.stroke_color}; stroke width:{self.stroke_width}] {self.points[0]} {self.radius}"


def serialize_header(size):
    return pack(
        "<BBhh", DRAW_COMMAND_VERSION, 0, int(round(size[0])), int(round(size[1]))
    )


def serialize(commands):
    output = pack("<H", len(commands))  # number of commands in list
    for c in commands:
        output += c.serialize()

    return output


def serialize_image(commands, size):
    s = serialize_header(size)
    s += serialize(commands)

    output = b"PDCI"
    output += pack("<I", len(s))
    output += s
    return output
