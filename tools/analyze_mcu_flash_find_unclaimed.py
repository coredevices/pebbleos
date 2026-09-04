#!/usr/bin/python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse

import binutils
import sh
from analyze_mcu_flash_config import *


def contains(a, b):
    """True if b is inside a"""
    return b[0] >= a[0] and b[1] <= a[1]


def claim(c, unclaimed_regions, symbol):
    """Removes region (c_start, c_end) from the set of unclaimed_regions
    Return True if the region was sucessfully removed, False if it was
    already claimed.

    """
    if c[0] == c[1]:
        raise RuntimeError(f"Invalid region: 0 size! {c}")

    for u in unclaimed_regions:
        if contains(u, c):
            unclaimed_regions.remove(u)

            # Defensive programming:
            if c[0] < u[0]:
                raise RuntimeError(f"WTF! {u} {c}")
            if c[1] > u[1]:
                raise RuntimeError(f"WTF! {u} {c}")

            if u[0] != c[0]:
                # Lower edge of the claimed region does not overlap with
                # the unclaimed region. Add a piece of unclaimed padding:
                unclaimed_regions.add((u[0], c[0]))
            if u[1] != c[1]:
                # Upper edge of the claimed region does not overlap with
                # the unclaimed region. Add a piece of unclaimed padding:
                unclaimed_regions.add((c[1], u[1]))
            return True

    print(f"Warning: doubly claimed {symbol}, 0x{c[0]:08x} - 0x{c[1]:08x}?")
    return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--dump", action="store_true", help="objdump unclaimed regions")
    parser.add_argument("--fast", action="store_true")
    parser.add_argument("--config", default="tintin", choices=CONFIG_CLASSES.keys())
    parser.add_argument("elf_file", nargs="?")
    args = parser.parse_args()

    config_class = CONFIG_CLASSES[args.config]
    config = config_class()

    elf_file = args.elf_file
    if not elf_file:
        elf_file = config.default_elf_abs_path()

    # The set of (addr_start, addr_end) tuples that we use to keep track of
    # unclaimed space in the flash:
    unclaimed_regions = {config.memory_region_to_analyze()}

    # Using arm-none-eabi-nm, 'claim' all .text symbols by removing the regions
    # from the unclaimed_regions set
    symbols = binutils.nm_generator(elf_file, args.fast)
    bytes_claimed = 0
    for addr, section, symbol, src_path, line, size in symbols:
        if section != "t":
            continue
        c = (addr, addr + size)
        if not contains(config.memory_region_to_analyze(), c):
            raise RuntimeError(
                f"Not in memory region: {symbol} 0x{c[0]:08x} - 0x{c[1]:08x}"
            )
        claim(c, unclaimed_regions, symbol)
        bytes_claimed += size

    # Using the resulting map of unused space,
    # calculate the total unclaimed space:
    bytes_unclaimed = 0
    for u in unclaimed_regions:
        bytes_unclaimed += u[1] - u[0]

    # Print out the results
    text_size = binutils.size(elf_file)[0]
    region = config.memory_region_to_analyze()
    print("------------------------------------------------------------")
    print(f".text:                            {text_size:d}")
    print(f"unclaimed memory:                 {bytes_unclaimed:d}")
    print(f"claimed memory:                   {bytes_claimed:d}")
    print(f"unknown .text regions             {text_size - bytes_claimed:d}")
    print()
    print("These should add up:")
    print(f"bytes_unclaimed + bytes_claimed = {bytes_unclaimed + bytes_claimed:d}")
    print(f"REGION_END - REGION_START =       {region[1] - region[0]:d}")
    print()

    num = 30
    print("------------------------------------------------------------")
    print(f"Top {num:d} unclaimed memory regions:")

    def comparator(a, b):
        return cmp(a[1] - a[0], b[1] - b[0])

    unclaimed_sorted_by_size = sorted(unclaimed_regions, cmp=comparator, reverse=True)
    for x in xrange(0, num):
        region = unclaimed_sorted_by_size[x]
        size = region[1] - region[0]
        if args.dump:
            print("-----------------------------------------------------------")
            print(f"{size:d} bytes @ 0x{region[0]:08x}")
            print()
            print(
                sh.arm_none_eabi_objdump(
                    "-S",
                    f"--start-address=0x{region[0]:x}",
                    f"--stop-address=0x{region[1]:x}",
                    elf_file,
                )
            )
        else:
            print(f"{size:d} bytes @ 0x{region[0]:08x}")

    print("------------------------------------------------------------")
    print("Unclaimed regions are regions that did map to symbols in the .elf.")
