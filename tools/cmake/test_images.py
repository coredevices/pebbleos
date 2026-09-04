#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Convert the unit tests' image fixtures.

The graphics tests render against baselines kept in tests/test_images and
compare the result, so every one of them has to be converted into the
format the firmware would load: PBIs, Pebble PNGs and PDCs. There are a
couple of thousand, converted in batches CMake hands out so that the
build's own scheduler decides how many run at once.

``convert`` builds the fixtures named in a list file; ``pdc`` runs the
step that needs the pdc2png host tool, which is kept apart so that a
change to the graphics code does not invalidate every other fixture.
"""

import argparse
import fnmatch
import os
import re
import shutil
import subprocess
import sys

TOOLS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOLS_DIR)

import bitmapgen
import png2pblpng
from generate_pdcs import pdc_gen

# Fixtures the tests load as they are rather than converted.
COPY_AS_IS = ("test_bitblt_circular__*.png", "test_gbitmap_sequence__*.apng",
              "test_kino_reel__*.apng", "test_graphics_draw_text_flow__*.png",
              "*.pfo", "*.pdc")


def _convert_pbi(src, dst):
    bitdepth = None
    if any(word in dst for word in (".8bit.", "~obelix", "~gabbro")):
        img_fmt = "color_raw"
    elif any(word in dst for word in (".1bit.", "~asterix")):
        img_fmt = "bw"
    else:
        img_fmt = "color"  # raw and palettized color images
        bit_suffix = re.search(r"(\d)bitpalette\.png", dst)
        if bit_suffix:
            bitdepth = int(bit_suffix.group(1))

    pb = bitmapgen.PebbleBitmap(src, bitmap_format=img_fmt, crop=False, bitdepth=bitdepth)
    pb.convert_to_pbi_file(dst)


def _convert_pblpng(src, dst):
    # Some baselines are compared against as-is.
    if dst.endswith(".raw.png"):
        shutil.copyfile(src, dst)
        return

    palette_name = "pebble64"
    bitdepth = None
    bit_suffix = re.search(r"(\d)bit(palette)?\.png", dst)
    if bit_suffix:
        bitdepth = int(bit_suffix.group(1))
    elif any(word in dst for word in ("~obelix", "~gabbro")):
        bitdepth = 8
    elif any(word in dst for word in ("~asterix",)):
        bitdepth = 1
        palette_name = "pebble2"

    png2pblpng.convert_png_to_pebble_png(src, dst, palette_name=palette_name, bitdepth=bitdepth)


def _convert_pdc(src, dst):
    # A directory of SVGs is one animation; a lone SVG is a still image.
    if os.path.isdir(src):
        pdc_gen.create_pdc_from_path(src, dst, viewbox_size=(0, 0), verbose=False,
                                     duration=33, play_count=1)
    else:
        pdc_gen.create_pdc_from_path(src, dst, viewbox_size=(0, 0), verbose=False,
                                     duration=0, play_count=0)


def _plan(src_dir, out_dir, entries):
    """The conversions the given fixtures call for."""
    available = set(os.listdir(src_dir))
    jobs = []

    def out(name):
        return os.path.join(out_dir, name)

    for entry in entries:
        name = os.path.basename(entry)
        if os.path.isdir(entry):
            jobs.append((_convert_pdc, entry, out(name + ".pdc")))
            continue

        stem, ext = os.path.splitext(name)
        if ext == ".png":
            if ".Xbit." in name:
                jobs.append((_convert_pbi, entry, out(name.replace(".Xbit.png", ".1bit.pbi"))))
                # Only emit the 8bit PBI when there is no platform-specific
                # .8bit.png baseline alongside; that one takes precedence.
                if name.replace(".Xbit.", ".8bit.") not in available:
                    jobs.append((_convert_pbi, entry, out(name.replace(".Xbit.png", ".8bit.pbi"))))
            else:
                jobs.append((_convert_pbi, entry, out(stem + ".pbi")))
            if fnmatch.fnmatch(name, "test_png__*.png"):
                jobs.append((_convert_pblpng, entry, out(name)))
        elif ext == ".svg":
            jobs.append((_convert_pdc, entry, out(stem + ".pdc")))

        if any(fnmatch.fnmatch(name, pattern) for pattern in COPY_AS_IS):
            jobs.append((shutil.copyfile, entry, out(name)))

    return jobs


def _stamp(path):
    with open(path, "w") as f:
        f.write("")


def cmd_convert(args):
    os.makedirs(args.out, exist_ok=True)
    with open(args.files) as f:
        entries = [line.strip() for line in f if line.strip()]
    for convert, src, dst in _plan(args.src, args.out, entries):
        convert(src, dst)
    _stamp(args.stamp)


def cmd_pdc(args):
    """The reference PDCs, rendered by pdc2png and converted back."""
    pdcs = []
    for name in sorted(os.listdir(args.src)):
        if not fnmatch.fnmatch(name, "test_pdc__*.pdc"):
            continue
        copy = os.path.join(args.out, name[: -len(".pdc")] + ".pdc.pdc")
        shutil.copyfile(os.path.join(args.out, name), copy)
        pdcs.append(copy)

    if pdcs:
        # pdc2png renders each one next to itself and shells out to pbi2png.
        subprocess.run([args.pdc2png] + pdcs, check=True, stdout=subprocess.DEVNULL)
        for pdc in pdcs:
            base = pdc[: -len(".pdc")]
            _convert_pbi(base + ".png", base + ".pbi")
    _stamp(args.stamp)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("convert")
    p.add_argument("--src", required=True, help="tests/test_images")
    p.add_argument("--out", required=True, help="where the converted fixtures go")
    p.add_argument("--files", required=True, help="file listing this batch's fixtures")
    p.add_argument("--stamp", required=True)
    p.set_defaults(func=cmd_convert)

    p = sub.add_parser("pdc")
    p.add_argument("--src", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--pdc2png", required=True, help="the pdc2png host tool")
    p.add_argument("--stamp", required=True)
    p.set_defaults(func=cmd_pdc)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
