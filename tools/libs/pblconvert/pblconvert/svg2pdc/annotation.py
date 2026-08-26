from xml.etree import ElementTree

from pdc import bounding_box_around_points

NS_ANNOTATION = "http://www.pebble.com/2015/pdc"
PREFIX_ANNOTATION = "pdc"

TAG_HIGHLIGHT = "{%s}highlight" % NS_ANNOTATION
TAG_ANNOTATION = "{%s}annotation" % NS_ANNOTATION


def to_str(value):
    return "%.2f" % float(value)


class Annotation:
    def __init__(self, node, text, transformed=False, link=None):
        self.node = node
        self.text = text
        self.href = link
        self.element = ElementTree.SubElement(self.node.node, TAG_ANNOTATION)
        if text is not None:
            self.element.set("description", text)
        if link is not None:
            self.element.set("href", link)
        self.highlights = []
        self.transformed = transformed
        self.node.annotations.append(self)

    def add_highlight(self, x, y, width=None, height=None, details=None):
        highlight = ElementTree.SubElement(self.element, TAG_HIGHLIGHT)
        self.set_highlight(
            highlight, x=x, y=y, width=width, height=height, details=details
        )
        self.highlights.append(highlight)
        return highlight

    def set_highlight(self, highlight, **kwargs):
        for k, v in kwargs.iteritems():
            if v is None:
                if k in highlight:
                    highlight.pop(k)
            else:
                value = to_str(v) if k in ["x", "y", "width", "height"] else v
                highlight.set(k, value)

    def transform(self, transformer):
        if self.transformed:
            return
        self.transformed = True

        for highlight in self.highlights:
            top_left = (float(highlight.get("x")), float(highlight.get("y")))
            size = (float(highlight.get("width")), float(highlight.get("height")))
            top_right = (top_left[0] + size[0], top_left[1])
            bottom_right = (top_left[0] + size[0], top_left[1] + size[1])
            bottom_left = (top_left[0], top_left[1] + size[1])

            # we needs to transform all four points instead of origin + diagonal
            # e.g. 45° on a square would otherwise become a line
            top_left = transformer.transform_point(top_left)
            top_right = transformer.transform_point(top_right)
            bottom_right = transformer.transform_point(bottom_right)
            bottom_left = transformer.transform_point(bottom_left)

            box = bounding_box_around_points(
                [top_left, top_right, bottom_right, bottom_left]
            )
            self.set_highlight(
                highlight, x=box[0], y=box[1], width=box[2], height=box[3]
            )
