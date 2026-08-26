from abc import *

from exceptions import PblConvertFormatError
from gif2apng.exceptions import Gif2ApngFormatError
from gif2apng.gif import convert_to_apng, read_gif
from svg2pdc.exceptions import Svg2PdcFormatError
from svg2pdc.pdc import convert_to_png, serialize_image
from svg2pdc.svg import surface_from_svg


class Handler:
    __metaclass__ = ABCMeta

    @classmethod
    def handler_for_format(cls, fmt):
        if cls is Handler:
            for C in cls.__subclasses__():
                if fmt == C.format():
                    return C()
            return None
        raise NotImplementedError

    @abstractmethod
    def read(self, in_obj):
        return None

    @abstractmethod
    def format(self):
        return ""

    @abstractmethod
    def write_pdc(self, out_obj, data):
        return None

    @abstractmethod
    def write_apng(self, out_obj, data):
        return None

    @abstractmethod
    def write_png(self, out_obj, data):
        return None

    @abstractmethod
    def write_svg(self, out_obj, data):
        return None


class SvgHandler(Handler):
    @classmethod
    def format(cls):
        return "svg"

    def read(self, in_obj):
        try:
            surface = surface_from_svg(
                bytestring=in_obj.read(), approximate_bezier=True
            )
        except Svg2PdcFormatError as e:
            raise PblConvertFormatError(e.args[0])
        return surface

    def write_apng(self, out_obj, data):
        raise NotImplementedError

    def write_pdc(self, out_obj, surface):
        commands = surface.pdc_commands
        pdci = serialize_image(commands, surface.size())
        with out_obj as o:
            o.write(pdci)

    def write_png(self, out_obj, surface):
        commands = surface.pdc_commands
        pdci = serialize_image(commands, surface.size())
        with out_obj as o:
            o.write(convert_to_png(pdci))

    def write_svg(self, out_obj, surface):
        with out_obj as o:
            et = surface.element_tree()
            et.write(o, pretty_print=True)


class GifHandler(Handler):
    @classmethod
    def format(cls):
        return "gif"

    def read(self, in_obj):
        try:
            gif = read_gif(in_obj)
        except Gif2ApngFormatError as e:
            raise PblConvertFormatError(e.args[0])
        return gif

    def write_pdc(self, out_obj, data):
        raise NotImplementedError

    def write_svg(self, out_obj, data):
        raise NotImplementedError

    def write_png(self, out_obj, gif):
        raise NotImplementedError

    def write_apng(self, out_obj, gif):
        apng_data = convert_to_apng(gif)
        with out_obj as o:
            o.write(apng_data)


Handler.register(GifHandler)
Handler.register(SvgHandler)
