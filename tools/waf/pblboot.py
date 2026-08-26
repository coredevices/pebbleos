#!/usr/bin/env python3
# Copyright (c) 2025 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import datetime
import re
import struct
import zlib

from intelhex import IntelHex

MAGIC = 0x96F3B83D

# Boot priority bands (bits 63..56 of the header priority field, formerly a
# plain build timestamp). The bootloader boots the valid slot with the highest
# 64-bit value, so bands order as: dev > release > legacy timestamp.
# 0x02-0x7f and 0x81-0xff are reserved for future schemes/overrides.
PRIORITY_BAND_RELEASE = 0x01
PRIORITY_BAND_DEV = 0x80

_RELEASE_TAG_RE = re.compile(r"^v(\d+)(?:\.(\d+))?(?:\.(\d+))?(?:-(?:beta|rc)\d+)?$")


def boot_priority(tag=None, commit_timestamp=None):
    """Compute the boot priority stamped into the pblboot header.

    Exact release tags (vX[.Y[.Z]], optionally -betaN/-rcN) encode the version
    in the high bits with the commit timestamp as tie-break, so the bootloader
    always picks the highest version regardless of build order. Anything else
    is a dev build: a higher band with the build wall-clock time, so the most
    recently built dev image always boots.
    """
    m = _RELEASE_TAG_RE.match(tag) if tag else None
    if m:
        major, minor, patch = (int(x) if x else 0 for x in m.groups())
        if max(major, minor, patch) > 0xFF:
            raise ValueError(f"version components must fit in 8 bits: {tag}")
        if commit_timestamp is None:
            raise ValueError(f"commit timestamp required for release tag: {tag}")
        return (
            (PRIORITY_BAND_RELEASE << 56)
            | (major << 48)
            | (minor << 40)
            | (patch << 32)
            | (commit_timestamp & 0xFFFFFFFF)
        )

    now = datetime.datetime.now(datetime.timezone.utc)
    return (PRIORITY_BAND_DEV << 56) | (int(now.timestamp()) & 0xFFFFFFFF)


def _insert_header_hex(fin, fout, offset, priority):
    # Load the hex file
    ih = IntelHex(fin)

    # Get the binary content from the hex file
    content = ih.tobinarray().tobytes()
    base_addr = ih.minaddr() - offset

    # Prepare the header
    crc = zlib.crc32(content)

    fwdesc = struct.pack("<LLQLLL", MAGIC, 28, priority, offset, len(content), crc)

    # Create new IntelHex object for output
    out_ih = IntelHex()

    # Add header at the beginning
    for i, byte in enumerate(fwdesc):
        out_ih[base_addr + i] = byte

    # Add padding
    pad_start = len(fwdesc)
    pad_end = offset
    for i in range(pad_start, pad_end):
        out_ih[base_addr + i] = 0xFF

    # Add original content at offset
    for i, byte in enumerate(content):
        out_ih[base_addr + offset + i] = byte

    # Write output hex file
    out_ih.write_hex_file(fout)


def _insert_header_bin(fin, fout, offset, priority):
    # Read the input binary file
    with open(fin, "rb") as f:
        content = f.read()

    # Prepare the header
    crc = zlib.crc32(content)

    fwdesc = struct.pack("<LLQLLL", MAGIC, 28, priority, offset, len(content), crc)

    # Write output binary file
    with open(fout, "wb") as f:
        f.write(fwdesc)
        f.write(b"\xff" * (offset - len(fwdesc)))
        f.write(content)


def _env_priority(bld):
    if bld.env.PBLBOOT_PRIORITY:
        return int(bld.env.PBLBOOT_PRIORITY)
    return boot_priority()


def insert_header_hex(task):
    bld = task.generator.bld
    _insert_header_hex(
        task.inputs[0].abspath(),
        task.outputs[0].abspath(),
        bld.env.FIRMWARE_OFFSET,
        _env_priority(bld),
    )


def insert_header_bin(task):
    bld = task.generator.bld
    _insert_header_bin(
        task.inputs[0].abspath(),
        task.outputs[0].abspath(),
        bld.env.FIRMWARE_OFFSET,
        _env_priority(bld),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate a firmware with pblboot header"
    )
    parser.add_argument("input", help="Input firmware hex file")
    parser.add_argument("output", help="Output firmware hex file")
    parser.add_argument(
        "--offset", type=int, default=512, help="Offset to apply to input file"
    )
    parser.add_argument("--tag", help="Git tag (exact release tags encode the version)")
    parser.add_argument(
        "--commit-timestamp",
        type=int,
        help="Commit timestamp (required with a release --tag)",
    )
    args = parser.parse_args()

    priority = boot_priority(args.tag, args.commit_timestamp)

    if args.input.endswith(".bin"):
        _insert_header_bin(args.input, args.output, args.offset, priority)
    else:
        _insert_header_hex(args.input, args.output, args.offset, priority)
