import unittest
from argparse import Namespace
from unittest import mock

from pblconvert.pblconvert import logic


class LogicTests(unittest.TestCase):
    def setUp(self):
        super().setUp()

    def test_pdc(self):
        parsed = Namespace(infile=1, outfile=2, outformat="pdc")
        handler = mock.Mock(spec=["read", "write_pdc"])
        handler.read.return_value = "surface"

        logic(handler, parsed)

        handler.read.assert_called_once_with(parsed.infile)
        handler.write_pdc.assert_called_once_with(parsed.outfile, "surface")

    def test_png(self):
        parsed = Namespace(infile=1, outfile=2, outformat="png")
        handler = mock.Mock(spec=["read", "write_png"])
        handler.read.return_value = "surface"

        logic(handler, parsed)

        handler.read.assert_called_once_with(parsed.infile)
        handler.write_png.assert_called_once_with(parsed.outfile, "surface")

    def test_annotated_svg(self):
        parsed = Namespace(infile=1, outfile=2, outformat="annotated_svg")
        handler = mock.Mock(spec=["read", "write_annotated_svg"])
        handler.read.return_value = "surface"

        logic(handler, parsed)

        handler.read.assert_called_once_with(parsed.infile)
        handler.write_annotated_svg.assert_called_once_with(parsed.outfile, "surface")

    def test_annotated_png(self):
        parsed = Namespace(infile=1, outfile=2, outformat="annotated_png")
        handler = mock.Mock(spec=["read", "write_annotated_png"])
        handler.read.return_value = "surface"

        logic(handler, parsed)

        handler.read.assert_called_once_with(parsed.infile)
        handler.write_annotated_png.assert_called_once_with(parsed.outfile, "surface")


if __name__ == "__main__":
    unittest.main()
