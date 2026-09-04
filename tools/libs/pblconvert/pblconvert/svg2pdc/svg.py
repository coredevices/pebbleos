import io
from functools import partial

import cairosvg
from annotation import NS_ANNOTATION, PREFIX_ANNOTATION, TAG_HIGHLIGHT, Annotation
from cairosvg.parser import Tree
from cairosvg.surface import color, gradient_or_pattern, node_format, normalize, size
from cairosvg.surface.helpers import paint, point
from exceptions import *
from lxml import etree
from pdc import (
    CircleCommand,
    PathCommand,
    bounding_box_around_points,
    extend_bounding_box,
)
from pebble_image_routines import (
    rgba32_triplet_to_argb8,
    truncate_color_to_pebble64_palette,
)
from StringIO import StringIO

try:
    import cairocffi as cairo
# OSError means cairocffi is installed,
# but could not load a cairo dynamic library.
# pycairo may still be available with a statically-linked cairo.
except (ImportError, OSError):
    import cairo  # pycairo


def cairo_from_png(path):
    surface = cairo.ImageSurface.create_from_png(path)
    return surface, cairo.Context(surface)


class PDCContext(cairo.Context):
    def line_to(self, x, y):
        super().line_to(x, y)


# http://effbot.org/zone/element-lib.htm#prettyprint
def indent(elem, level=0):
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for child in elem:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


class PDCSurface(cairosvg.surface.PNGSurface):
    # noinspection PyMissingConstructor
    def __init__(
        self, tree, output, dpi, parent_surface=None, approximate_bezier=False
    ):
        self.svg_tree = tree
        self.cairo = None
        self.cairosvg_tags = []
        self.pdc_commands = []
        self.approximate_bezier = approximate_bezier
        self.stored_size = None
        self.context_width, self.context_height = None, None
        self.cursor_position = [0, 0]
        self.cursor_d_position = [0, 0]
        self.text_path_width = 0
        self.tree_cache = {(tree.url, tree["id"]): tree}
        if parent_surface:
            self.markers = parent_surface.markers
            self.gradients = parent_surface.gradients
            self.patterns = parent_surface.patterns
            self.masks = parent_surface.masks
            self.paths = parent_surface.paths
            self.filters = parent_surface.filters
        else:
            self.markers = {}
            self.gradients = {}
            self.patterns = {}
            self.masks = {}
            self.paths = {}
            self.filters = {}
        self.page_sizes = []
        self._old_parent_node = self.parent_node = None
        self.output = output
        self.dpi = dpi
        self.font_size = size(self, "12pt")
        self.stroke_and_fill = True
        # we only support px
        for unit in ["mm", "cm", "in", "pt", "pc"]:
            for attr in ["width", "height"]:
                value = tree.get(attr)
                if value is not None and unit in value:
                    tree.pop(attr)
                    value = None

        width, height, viewbox = node_format(self, tree)
        # Actual surface dimensions: may be rounded on raster surfaces types
        self.cairo, self.width, self.height = self._create_surface(
            width * self.device_units_per_user_units,
            height * self.device_units_per_user_units,
        )
        self.page_sizes.append((self.width, self.height))
        self.context = PDCContext(self.cairo)
        # We must scale the context as the surface size is using physical units
        self.context.scale(
            self.device_units_per_user_units, self.device_units_per_user_units
        )

        # SVG spec says "viewbox of size 0 means 'don't render'"
        if viewbox is not None and viewbox[2] <= 0 and viewbox[3] <= 0:
            return

        # Initial, non-rounded dimensions
        self.set_context_size(width, height, viewbox)
        self.context.move_to(0, 0)

        # register PDC namespace and add temporary fake attribute to propagate prefix
        etree.register_namespace(PREFIX_ANNOTATION, NS_ANNOTATION)
        ns_fake_attr = f"{{{NS_ANNOTATION}}}foo"
        tree.node.set(ns_fake_attr, "bar")

        # remove all PDC elements (annotations in case we're processing an annotated SVG)
        def remove_pdc_elements(elem):
            for child in elem:
                if isinstance(child.tag, str) and child.tag.startswith(
                    f"{{{NS_ANNOTATION}}}"
                ):
                    elem.remove(child)
                else:
                    remove_pdc_elements(child)

        remove_pdc_elements(tree.node)
        self.draw_root(tree)
        tree.node.attrib.pop(ns_fake_attr)

    def size(self):
        if self.stored_size is not None:
            return self.stored_size

        result = None
        for command in self.pdc_commands:
            result = extend_bounding_box(result, rect2=command.bounding_box())

        if result is None:
            return (0, 0)

        # returned size is diagonal from (0, 0) to max_x/max_y
        return (max(0, result[0] + result[2]), max(0, result[1] + result[3]))

    def cairo_tag_func(self, tag):
        return self.cairosvg_tags[0][tag]

    def cairo_tags_push_and_wrap(self):
        self.cairosvg_tags.append(cairosvg.surface.TAGS.copy())
        custom_impl = {
            "polyline": polyline,
            "polygon": polygon,
            "line": line,
            "rect": rect,
            "circle": circle,
            "path": path,
            "svg": svg,
        }
        for k, v in custom_impl.iteritems():
            original = self.cairo_tag_func(k)
            cairosvg.surface.TAGS[k] = partial(custom_impl[k], original=original)

    def draw_root(self, node):
        if node.get("display", "").upper() == "NONE":
            node.annotations = []
            Annotation(
                node, 'Attribute display="none" for root element will be ignored.'
            )
            node.pop("display")

        super().draw_root(node)

    def draw(self, node):
        self.cairo_tags_push_and_wrap()
        if not hasattr(node, "annotations"):
            node.annotations = []
        super().draw(node)
        cairosvg.surface.TAGS = self.cairosvg_tags.pop()

    def element_tree(self):
        indent(self.svg_tree.node)
        # ensure that viewbox is always set
        view_box = self.svg_tree.node.get("viewBox", "0 0 {} {}".format(*self.size()))
        self.svg_tree.node.set("viewBox", view_box)

        return etree.ElementTree(self.svg_tree.node)

    def render_annoations_on_top(self, png_path):
        surface, ctx = cairo_from_png(png_path)

        def iterate(elem):
            if TAG_HIGHLIGHT == elem.tag:
                args = [float(elem.get(k, "1")) for k in ["x", "y", "width", "height"]]
                ctx.rectangle(*args)
                ctx.set_source_rgb(1, 0, 0)
                ctx.stroke()

            for child in elem:
                iterate(child)

        iterate(self.svg_tree.node)
        result = StringIO()
        surface.write_to_png(result)
        return result.getvalue()


def gcolor8_is_visible(color):
    return (color & 0b11000000) != 0


def svg_color(surface, node, opacity, attribute, default=None):
    paint_source, paint_color = paint(node.get(attribute, default))
    if gradient_or_pattern(surface, node, paint_source):
        return 0
    (r, g, b, a) = color(paint_color, opacity)
    color256 = truncate_color_to_pebble64_palette(
        int(r * 255), int(g * 255), int(b * 255), int(a * 255)
    )
    gcolor8 = rgba32_triplet_to_argb8(*color256)
    return gcolor8 if gcolor8_is_visible(gcolor8) else 0


def svg(surface, node, original=None):
    original(surface, node)
    width, height, viewbox = node_format(surface, node)
    if "width" not in node:
        width = None if viewbox is None else viewbox[2]
    if "height" not in node:
        height = None if viewbox is None else viewbox[3]
    if (width is not None) and (height is not None):
        surface.stored_size = (width, height)


def line(surface, node, original=None):
    x1, y1, x2, y2 = tuple(
        size(surface, node.get(position), position[0])
        for position in ("x1", "y1", "x2", "y2")
    )
    points = [(x1, y1), (x2, y2)]
    command = PathCommand(points, path_open=True, precise=True)
    handle_command(surface, node, command)

    return original(surface, node)


def polygon(surface, node, original=None):
    return poly_element(surface, node, open=False, original=original)


def polyline(surface, node, original=None):
    return poly_element(surface, node, open=True, original=original)


def poly_element(surface, node, open, original=None):
    points_attr = normalize(node.get("points"))
    points = []
    while points_attr:
        x, y, points_attr = point(surface, points_attr)
        points.append((x, y))
    command = PathCommand(points, path_open=True, precise=True)
    command.open = open
    handle_command(surface, node, command)

    return original(surface, node)


def rect(surface, node, original=None):
    x, y = size(surface, node.get("x"), "x"), size(surface, node.get("y"), "y")
    width = size(surface, node.get("width"), "x")
    height = size(surface, node.get("height"), "y")

    rx = node.get("rx")
    ry = node.get("ry")
    if rx and (ry is None):
        ry = rx
    elif ry and (rx is None):
        rx = ry
    rx = size(surface, rx, "x")
    ry = size(surface, ry, "y")
    if not (rx == 0 and ry == 0):
        annotation = Annotation(node, "Rounded rectangles are not supported.")
        right = x + width - rx
        bottom = y + height - ry
        annotation.add_highlight(x, y, rx, ry)
        annotation.add_highlight(right, y, rx, ry)
        annotation.add_highlight(right, bottom, rx, ry)
        annotation.add_highlight(x, bottom, rx, ry)

    points = [(x, y), (x + width, y), (x + width, y + height), (x, y + height)]
    command = PathCommand(points, path_open=False, precise=True)
    handle_command(surface, node, command)

    return original(surface, node)


def circle(surface, node, original=None):
    r = size(surface, node.get("r"))
    cx = size(surface, node.get("cx"), "x")
    cy = size(surface, node.get("cy"), "y")

    if r and cx is not None and cy is not None:
        command = CircleCommand((cx, cy), r)
        handle_command(surface, node, command)

    return original(surface, node)


def cubicbezier_mid(p0, p1, p2, p3, min_dist, l, r):
    t = (r[0] + l[0]) / 2
    a = (1.0 - t) ** 3
    b = 3.0 * t * (1.0 - t) ** 2
    c = 3.0 * t**2 * (1.0 - t)
    d = t**3

    p = (
        a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0],
        a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1],
    )

    def pt_dist(p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

    if pt_dist(l[1], p) <= min_dist or pt_dist(r[1], p) <= min_dist:
        return []

    left = cubicbezier_mid(p0, p1, p2, p3, min_dist=min_dist, l=l, r=(t, p))
    right = cubicbezier_mid(p0, p1, p2, p3, min_dist=min_dist, l=(t, p), r=r)

    return left + [p] + right


def cubicbezier(p0, p1, p2, p3, min_dist):
    return [p0] + cubicbezier_mid(p0, p1, p2, p3, min_dist, (0.0, p0), (1.0, p3)) + [p3]


class PathSurfaceContext(cairo.Context):
    def __init__(self, target, node, approximate_bezier):
        super().__init__(target)
        self.points = []
        self.path_open = True
        self.node = node
        self.approximate_bezier = approximate_bezier
        self.grouped_annotations = {}

    def get_grouped_description(self, description, *args, **kwargs):
        if description not in self.grouped_annotations:
            self.grouped_annotations[description] = self.add_annotation(
                description, *args, **kwargs
            )

        return self.grouped_annotations[description]

    def add_annotation(self, *args, **kwargs):
        return Annotation(self.node, *args, **kwargs)

    def add_current_point(self):
        self.points.append(self.get_current_point())

    def create_command(self):
        return PathCommand(self.points, self.path_open, precise=True)

    def move_to(self, x, y):
        super().move_to(x, y)
        self.add_current_point()

    def line_to(self, x, y):
        super().line_to(x, y)
        self.add_current_point()

    def rel_line_to(self, dx, dy):
        super().rel_line_to(dx, dy)
        self.add_current_point()

    def curve_to(self, x1, y1, x2, y2, x3, y3):
        first = self.get_current_point()
        super().curve_to(x1, y1, x2, y2, x3, y3)
        last = self.get_current_point()

        # approximate bezier curve
        points = cubicbezier(first, (x1, y1), (x2, y2), last, 5)
        box = bounding_box_around_points(points)

        if self.approximate_bezier:
            self.points.extend(points[1:])
        else:
            self.add_current_point()

        link = "https://pebbletechnology.atlassian.net/wiki/display/DEV/Pebble+Draw+Commands#PebbleDrawCommands-issue-bezier"
        self.get_grouped_description(
            "Element contains unsupported curved command(s).", link=link
        ).add_highlight(*box)

    def rel_curve_to(self, dx1, dy1, dx2, dy2, dx3, dy3):
        cur = self.get_current_point()
        x1 = dx1 + cur[0]
        y1 = dy1 + cur[1]
        x2 = dx2 + cur[0]
        y2 = dy2 + cur[1]
        x3 = dx3 + cur[0]
        y3 = dy3 + cur[1]
        self.curve_to(x1, y1, x2, y2, x3, y3)

    def arc(self, xc, yc, radius, angle1, angle2):
        self.add_annotation_arc_unsupported(xc, yc, radius)
        super().arc(xc, yc, radius, angle1, angle2)

    def arc_negative(self, xc, yc, radius, angle1, angle2):
        self.add_annotation_arc_unsupported(xc, yc, radius)
        super().arc_negative(xc, yc, radius, angle1, angle2)

    def close_path(self):
        super().close_path()
        self.path_open = False

    def add_annotation_arc_unsupported(self, xc, yc, radius):
        # caller uses context transforms to express arcs
        points = [
            (xc - radius, yc - radius),  # top-left
            (xc + radius, yc - radius),  # top-right
            (xc + radius, yc + radius),  # bottom-right
            (xc - radius, yc + radius),  # bottom-left
        ]
        box = bounding_box_around_points([self.user_to_device(*p) for p in points])
        self.get_grouped_description(
            "Element contains unsupported arc command(s)."
        ).add_highlight(*box)


class PathSurface:
    def __init__(self, target, node, approximate_bezier):
        self.context = PathSurfaceContext(target, node, approximate_bezier)


def path(surface, node, original=None):
    # unfortunately, the original implementation has a side-effect on node
    # but as it's rather complex, we'd rather reuse it and as it doesn't change the surface
    # it's ok to call it with the fake surface we create here
    collecting_surface = PathSurface(surface.cairo, node, surface.approximate_bezier)
    result = original(collecting_surface, node)
    command = collecting_surface.context.create_command()
    if len(command.points) > 1:
        handle_command(surface, node, command)
    else:
        Annotation(node, "Path needs at least two points.")

    return result


class Transformer:
    def __init__(self, cairo, node):
        self.context = cairo
        self.node = node

    def transform_point(self, pt):
        return self.context.user_to_device(pt[0], pt[1])

    def transform_distance(self, dx, dy):
        return self.context.user_to_device_distance(dx, dy)

    def add_annotation(self, *args, **kwargs):
        return Annotation(self.node, *args, **kwargs)


def handle_command(surface, node, command):
    opacity = float(node.get("opacity", 1))
    # Get stroke and fill opacity
    stroke_opacity = float(node.get("stroke-opacity", 1))
    fill_opacity = float(node.get("fill-opacity", 1))
    if opacity < 1:
        stroke_opacity *= opacity
        fill_opacity *= opacity
    default_fill = "black" if node.get("fill-rule") == "evenodd" else None
    command.fill_color = svg_color(surface, node, fill_opacity, "fill", default_fill)
    command.stroke_color = svg_color(surface, node, stroke_opacity, "stroke")
    if gcolor8_is_visible(command.stroke_color):
        command.stroke_width = int(size(surface, node.get("stroke-width")))
    if command.stroke_width == 0:
        command.stroke_color = 0

    # transform
    transformer = Transformer(surface.context, node)
    command.transform(transformer)
    if command.stroke_width and node.get("vector-effect") != "non-scaling-stroke":
        transformed_stroke = transformer.transform_distance(command.stroke_width, 0)
        transformed_stroke_width = (
            transformed_stroke[0] ** 2 + transformed_stroke[1] ** 2
        ) ** 0.5
        command.stroke_width = transformed_stroke_width
    for annotation in node.annotations:
        annotation.transform(transformer)

    command.finalize(transformer)

    # Manage display and visibility
    display = node.get("display", "inline") != "none"
    visible = (
        display
        and (node.get("visibility", "visible") != "hidden")
        and (
            gcolor8_is_visible(command.fill_color)
            or gcolor8_is_visible(command.stroke_color)
        )
    )

    if visible:
        surface.pdc_commands.append(command)


def surface_from_svg(url=None, bytestring=None, approximate_bezier=False):
    try:
        tree = Tree(url=url, bytestring=bytestring)
    except etree.XMLSyntaxError as e:
        raise Svg2PdcFormatError(e.args[0])
    output = io.BytesIO()
    return PDCSurface(tree, output, 96, approximate_bezier=approximate_bezier)
