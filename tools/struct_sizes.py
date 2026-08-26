#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse
import logging
import sys

from elftools.common.utils import preserve_stream_pos
from elftools.dwarf.die import DIE
from elftools.elf.elffile import ELFFile


def _extract_struct_sizes(die, struct_names_by_size):
    def add(name, size):
        if size not in struct_names_by_size:
            struct_names_by_size[size] = set()
        struct_names_by_size[size].add(name)
        logging.debug("%s => %s Bytes" % (name, size))

    # Handle typedef'd anonymous structs:
    if die.tag == "DW_TAG_typedef":
        assert die.attributes["DW_AT_type"].form == "DW_FORM_ref4"
        offset = die.attributes["DW_AT_type"].value
        ref_die_offset = offset + die.cu.cu_offset
        with preserve_stream_pos(die.stream):
            ref_die = DIE(die.cu, die.stream, ref_die_offset)
            if ref_die.tag == "DW_TAG_structure_type":
                if "DW_AT_byte_size" in ref_die.attributes:
                    name = die.attributes["DW_AT_name"].value
                    size = int(ref_die.attributes["DW_AT_byte_size"].value)
                    add(name, size)
                else:
                    # Most likely a fwd declaration has been used
                    pass
        return

    # Handle named structs:
    if die.tag == "DW_TAG_structure_type":
        if "DW_AT_name" in die.attributes and "DW_AT_byte_size" in die.attributes:
            size = int(die.attributes["DW_AT_byte_size"].value)
            name = die.attributes["DW_AT_name"].value
            add(name, size)


def get_struct_names_by_size(elffile, print_progress=False):
    if not elffile.has_dwarf_info():
        logging.error("File has no DWARF info")
        return None

    dwarfinfo = elffile.get_dwarf_info()
    struct_names_by_size = dict()

    for CU in dwarfinfo.iter_CUs():
        logging.debug(
            "Found a compile unit at offset %s, length %s"
            % (CU.cu_offset, CU["unit_length"])
        )
        if print_progress:
            sys.stdout.write(".")
            sys.stdout.flush()

        def die_info_recurse(die):
            _extract_struct_sizes(die, struct_names_by_size)
            for child in die.iter_children():
                die_info_recurse(child)

        die_info_recurse(CU.get_top_DIE())

    if print_progress:
        sys.stdout.write("\n")

    return struct_names_by_size


def _process_elf(filename, verbose=False):
    logging.info("Processing .elf file: %s" % filename)

    with open(filename, "rb") as f:
        elffile = ELFFile(f)

        print_progress = not (verbose)
        struct_names_by_size = get_struct_names_by_size(elffile, print_progress)
        for size in sorted(struct_names_by_size.keys(), reverse=True):
            logging.info("%6u bytes: %s", size, " ".join(struct_names_by_size[size]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument(
        "-o",
        "--logfile",
        help="Log file to output to. Output printed to console otherwise.",
    )
    parser.add_argument("elf_file", help="Extracts all struct sizes from elf file.")
    args = parser.parse_args()

    level = logging.INFO
    if args.verbose:
        level = logging.DEBUG
    logging.basicConfig(level=level, filename=args.logfile)

    _process_elf(args.elf_file, verbose=args.verbose)
