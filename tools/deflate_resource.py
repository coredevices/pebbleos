#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Compress a raw resource for the weather app's inflate-at-load path.

Output layout: [u32 LE inflated_size][raw DEFLATE stream]
The stream is consumed on-watch by tinflate_uncompress() (raw deflate, no zlib
header), see src/fw/apps/system/weather/globe_view.c prv_load_inflated().

--pdc-payload strips the 8-byte PDC file header ("PDCS"/"PDCI" magic + u32 size)
first: gdraw_command_sequence pointers address the PAYLOAD directly, so the
loader inflates straight into the buffer it hands to
gdraw_command_sequence_validate(). Regenerate with:
  python3 tools/deflate_resource.py IN OUT [--pdc-payload]

Searches window sizes / strategies for the smallest stream (matches what
tinflate accepts: any raw-deflate window).
"""

import argparse
import struct
import sys
import zlib


def best_deflate(data: bytes) -> bytes:
    best = None
    for wbits in range(-9, -16, -1):
        for strategy in (zlib.Z_DEFAULT_STRATEGY, zlib.Z_FILTERED):
            co = zlib.compressobj(9, zlib.DEFLATED, wbits, 9, strategy)
            out = co.compress(data) + co.flush()
            if best is None or len(out) < len(best):
                best = out
    return best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("infile")
    ap.add_argument("outfile")
    ap.add_argument(
        "--pdc-payload",
        action="store_true",
        help="strip the 8-byte PDC header before compressing",
    )
    args = ap.parse_args()

    data = open(args.infile, "rb").read()
    if args.pdc_payload:
        if data[:4] not in (b"PDCS", b"PDCI"):
            print(f"error: {args.infile} lacks a PDC magic", file=sys.stderr)
            return 1
        data = data[8:]

    stream = best_deflate(data)
    # round-trip check (tinflate-equivalent: raw stream, exact size)
    assert zlib.decompress(stream, wbits=-15) == data, "round-trip mismatch"

    blob = struct.pack("<I", len(data)) + stream
    open(args.outfile, "wb").write(blob)
    print(
        f"{args.infile}: {len(data)} -> {len(blob)} bytes "
        f"({len(blob) * 100 // len(data)}%)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
