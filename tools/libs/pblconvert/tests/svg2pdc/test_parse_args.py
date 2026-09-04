import argparse
import os
import sys
import unittest

from pblconvert import pblconvert
from pblconvert.pblconvert import parse_args


class FakeFile:
    def __init__(self, name):
        self.name = name


class FakeFileType:
    def __init__(self, mode="r", bufsize=-1):
        self._mode = mode
        self._bufsize = bufsize

    def __call__(self, string):
        # the special argument "-" means sys.std{in,out}
        if string == "-":
            if "r" in self._mode:
                return sys.stdin
            elif "w" in self._mode:
                return sys.stdout
            else:
                msg = f'argument "-" with mode {self._mode!r}'
                raise ValueError(msg)

        return FakeFile(string)


class ParseArgsTests(unittest.TestCase):
    def fake_path_exists(self, path):
        return path in self.files

    def setUp(self):
        self.files = []
        argparse.FileType = FakeFileType
        os.path.exists = self.fake_path_exists
        self.files.append("temp.svg")

    def test_requires_in_file(self):
        with self.assertRaises(SystemExit):
            parse_args(["--outformat", "pdc"])

    def test_outfile_defaults_to_stdout(self):
        parsed = parse_args(["-i", "-", "--outformat", "pdc"])
        self.assertEqual(parsed.infile, sys.stdin)
        self.assertEqual(parsed.outfile, sys.stdout)

    def test_outfile_defaults_to_file(self):
        parsed = parse_args(["--infile", "temp.svg", "--outformat", "pdc"])
        self.assertEqual(parsed.infile.name, "temp.svg")
        expected = os.path.splitext("temp.svg")[0] + ".pdc"
        self.assertEqual(parsed.outfile.name, expected)

    def test_explicit_outfile_influences_format(self):
        parsed = parse_args(["-i", "-", "--outfile", "test.png"])
        self.assertEqual(parsed.outformat, "png")
        parsed = parse_args(["-i", "-", "--outfile", "test.svg"])
        self.assertEqual(parsed.outformat, "svg")
        parsed = parse_args(["-i", "-", "--outfile", "test.pdc"])
        self.assertEqual(parsed.outformat, "pdc")
        parsed = parse_args(["-i", "-", "--outfile", "test.any"])
        self.assertEqual(parsed.outformat, "pdc")

    def test_explicit_format_overrules_derived_format(self):
        parsed = parse_args(["-i", "-", "--outformat", "svg", "--outfile", "test.png"])
        self.assertEqual(parsed.outformat, "svg")

    def test_implicit_filename_avoids_override(self):
        self.files.append("exists.svg")
        self.files.append("exists.pdc")
        self.files.append("exists_3.pdc")
        parsed = parse_args(["-i", "exists.svg", "--outformat", "pdc"])
        self.assertEqual(os.path.basename(parsed.outfile.name), "exists_2.pdc")
        self.files.append("exists_2.pdc")
        parsed = parse_args(["-i", "exists.svg", "--outformat", "pdc"])
        self.assertEqual(os.path.basename(parsed.outfile.name), "exists_4.pdc")

    def test_error_if_out_of_alternatives_for_implicit_name(self):
        old_value = pblconvert.LIMIT_WHEN_AVOIDING_OVERRIDE
        try:
            self.files.append("exists.svg")
            self.files.append("exists.pdc")
            pblconvert.LIMIT_WHEN_AVOIDING_OVERRIDE = 2
            with self.assertRaises(IOError):
                parse_args(["-i", "exists.svg", "--outformat", "pdc"])
        finally:
            pblconvert.LIMIT_WHEN_AVOIDING_OVERRIDE = old_value


if __name__ == "__main__":
    unittest.main()
