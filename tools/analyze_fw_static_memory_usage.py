# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import re
import sys

sys.path.append(os.path.dirname(__file__))
import analyze_static_memory_usage
from binutils import analyze_elf, nm_generator


def cleanup_path(f):
    f = os.path.normpath(f)

    # Check for .c.3.o style suffixes and strip them back to just .c
    if len(f) > 6 and f[-6:-3] == ".c." and f[-2:] == ".o":
        f = f[:-4]

    f = f.removeprefix("src/")

    newlib_index = f.rfind("/newlib/libc")
    if newlib_index != -1:
        f = f[newlib_index + 1 :]

    libgcc_index = f.rfind("/libgcc/")
    if libgcc_index != -1:
        f = f[libgcc_index + 1 :]

    libc_index = f.rfind("/arm-none-eabi/lib/")
    if libc_index != -1:
        f = f[libc_index + 1 :]

    tintin_index = f.rfind("/tintin/src/")
    if tintin_index != -1:
        f = f[tintin_index + len("/tintin/src/") :]

    tintin_build_index = f.rfind("/tintin/build/src/")
    if tintin_build_index != -1:
        f = f[tintin_build_index + len("/tintin/") :]

    return f


analyze_static_memory_usage.cleanup_path_func = cleanup_path


def analyze_map(map_file, sections):
    # Now that we have a list of all the symbols listed in the nm output, we need to go back
    # and dig through the map file to find filenames for the symbols with an "Unknown" filename

    # We only care about the .text section here
    if not "t" in sections:
        return

    text_section = sections["t"]

    def line_generator(map_file):
        with open(map_file) as f:
            yield from f

    lines = line_generator(map_file)
    for line in lines:
        if line.startswith("Linker script and memory map"):
            break

    for line in lines:
        if line.startswith(".text"):
            break

    # We're looking for groups of lines like the following...
    #
    # .text.do_tap_handle
    #            0x0000000008010e08       0x28 src/fw/applib/accel_service.c.3.o

    symbol_pattern = re.compile(r""" \.?[^\.\s]*\.(\S+)""")
    for line in lines:
        match = symbol_pattern.match(line.rstrip())
        if match is None:
            continue

        symbol = match.group(1)

        line = lines.next()

        cols = line.split()
        if len(cols) < 3:
            continue

        filename = cols[2]

        symbol_with_unknown_file = text_section.remove_unknown_entry(symbol)
        if symbol_with_unknown_file is None:
            continue
        text_section.add_entry(symbol, filename, symbol_with_unknown_file.size)


def analyze_libs(root_directory, sections, use_fast):
    def analyze_lib(lib_filename):
        for _, section, symbol_name, filename, line, size in nm_generator(
            lib_filename, use_fast
        ):
            if not section in sections:
                continue

            section_info = sections[section]

            symbol_with_unknown_file = section_info.remove_unknown_entry(symbol_name)
            if symbol_with_unknown_file is None:
                continue

            section_info.add_entry(symbol_name, lib_filename, size)

    for dirpath, dirnames, filenames in os.walk(root_directory):
        for f in filenames:
            if f.endswith(".a"):
                analyze_lib(os.path.join(dirpath, f))


def print_groups(text_section, verbose):
    mappings = [
        ("third_party/freertos/FreeRTOS-Kernel/", "FreeRTOS"),
        ("core/vendor/STM32F2xx_StdPeriph_Lib_V1.0.0", "STM32"),
        ("newlib/", "newlib"),
        ("libgcc/", "libgcc"),
        ("arm-none-eabi/lib/", "libc"),
        ("fw/applib/", "FW Applib"),
        ("fw/apps/", "FW Apps"),
        ("fw/comm/ble/", "FW Comm LE"),
        ("fw/comm/", "FW Comm"),
        ("fw/kernel/services/", "FW Kernel Services"),
        ("fw/", "FW Other"),
        ("core/", "FW Other"),
        ("build/src/fw", "FW Other"),
    ]

    class Group:
        def __init__(self, name):
            self.name = name
            self.total_size = 0
            self.files = []

        def add_file(self, f):
            self.total_size += f.size
            self.files.append(f)

    group_sizes = {}

    for f in text_section.get_files():
        found = False

        for prefix, value in mappings:
            if f.filename.startswith(prefix):
                if not value in group_sizes:
                    group_sizes[value] = Group(f.filename)
                group_sizes[value].add_file(f)
                found = True
                break

        if not found:
            if not "Unknown" in group_sizes:
                group_sizes["Unknown"] = Group(f.filename)
            group_sizes["Unknown"].add_file(f)

    sorted_items = sorted(group_sizes.iteritems(), key=lambda x: -x[1].total_size)
    for group_name, group in sorted_items:
        print(f"{group_name:<20} {group.total_size:d}")
        if verbose:
            sorted_files = sorted(group.files, key=lambda x: -x.size)
            for f in sorted_files:
                print(f"  {f.size:6d} {f.filename:<20}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--text_groups", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--sections", default="bdt")
    parser.add_argument("--fast", action="store_true")
    args = parser.parse_args()

    if args.text_groups:
        args.sections = "t"

    tintin_dir = os.path.join(os.path.dirname(__file__), "..")
    elf_path = os.path.join(tintin_dir, "build", "pebbleos.elf")

    sections = analyze_elf(elf_path, args.sections, args.fast)

    analyze_map(os.path.join(tintin_dir, "build", "pebbleos.map"), sections)

    if args.text_groups:
        print_groups(sections["t"], args.verbose)
    else:
        for s in args.sections:
            sections[s].pprint(args.summary, args.verbose)
