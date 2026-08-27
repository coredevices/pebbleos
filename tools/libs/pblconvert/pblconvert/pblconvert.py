import argparse
import os
import sys

from handlers import *

SUPPORTED_FORMATS_MAP = {
    "in": {
        "gif": ".gif",
        "svg": ".svg",
    },
    "out": {
        "pdc": ".pdc",
        "png": ".png",
        "svg": ".svg",
        "apng": ".apng",
    },
}

FORMAT_TO_EXT = dict(SUPPORTED_FORMATS_MAP["in"], **SUPPORTED_FORMATS_MAP["out"])
EXT_TO_FORMAT = {v: k for k, v in FORMAT_TO_EXT.items()}

OUT_FORMATS = SUPPORTED_FORMATS_MAP["out"].keys()
IN_FORMATS = SUPPORTED_FORMATS_MAP["in"].keys()

LIMIT_WHEN_AVOIDING_OVERRIDE = 100


def parse_args(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--infile", type=argparse.FileType("r"), required=True)
    parser.add_argument("-if", "--informat", type=str, choices=IN_FORMATS)
    parser.add_argument("-o", "--outfile", type=argparse.FileType("w"))
    parser.add_argument("-of", "--outformat", type=str, choices=OUT_FORMATS)

    parsed = parser.parse_args(args)

    assert parsed.infile is not None
    assert parsed.outformat is not None or parsed.outfile is not None

    if parsed.informat is None:
        parsed.informat = EXT_TO_FORMAT.get(
            os.path.splitext(parsed.infile.name)[1], "svg"
        )

    if parsed.outformat is None:
        parsed.outformat = (
            "pdc"
            if parsed.outfile is None
            else EXT_TO_FORMAT.get(os.path.splitext(parsed.outfile.name)[1], "pdc")
        )

    if parsed.outfile is None:
        if parsed.infile == sys.stdin:
            parsed.outfile = sys.stdout
        else:
            # look at format
            outfile_path = (
                os.path.splitext(parsed.infile.name)[0]
                + FORMAT_TO_EXT[parsed.outformat]
            )
            if os.path.exists(outfile_path):
                avoiding_path = None
                # avoid accidental overrides
                for i in range(2, LIMIT_WHEN_AVOIDING_OVERRIDE):
                    split = os.path.splitext(outfile_path)
                    avoiding_path = f"{split[0]}_{i:d}{split[1]}"
                    if not os.path.exists(avoiding_path):
                        outfile_path = avoiding_path
                        break

                if outfile_path != avoiding_path:
                    raise OSError(
                        f"File {outfile_path} and ({LIMIT_WHEN_AVOIDING_OVERRIDE:d} similar alternatives) "
                        "already exists"
                    )

            parsed.outfile = open(outfile_path, "w")  # noqa: SIM115

    return parsed


def logic(handler, parsed):
    data = handler.read(parsed.infile)
    method = getattr(handler, "write_" + parsed.outformat)
    method(parsed.outfile, data)


def main():
    parsed = parse_args(sys.argv[1:])
    handler = Handler.handler_for_format(parsed.informat)
    logic(handler, parsed)
