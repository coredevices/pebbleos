#! /usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


# elf_sections.py
# This script analyses the specified .elf and provides output similar to readelf -S.
# The sections are sorted by start address and gaps/overlaps in the memory map are displayed.

import argparse

from elftools.elf.elffile import ELFFile

EXCLUDED_SECTIONS = [".log_strings"]

BT_DIALOG_SECTION_START = ("** RAM Start **", 0x7FC0000, 0)
BT_DIALOG_SECTION_END = ("** RAM End **", 0x7FE3FFF, 0)


# Return a list of sections as a tuple (name, start address, size)
def _get_sections(elf, all_sections):
    headers = []

    for nsec, section in enumerate(elf.iter_sections()):
        if not all_sections and (
            section["sh_addr"] == 0
            or section["sh_size"] == 0
            or section.name in EXCLUDED_SECTIONS
        ):
            continue
        headers.append((section.name, section["sh_addr"], section["sh_size"]))

    return headers


def _process_elf(filename, verbose=False, all_sections=False, bt=False):
    with open(filename, "rb") as f:
        elffile = ELFFile(f)

        # Sort by the second element (start address)
        sections = sorted(_get_sections(elffile, all_sections), key=lambda x: x[1])
        if bt:
            sections.insert(0, BT_DIALOG_SECTION_START)
            sections.append(BT_DIALOG_SECTION_END)

        print(
            "{:<20}   {:>10}   {:>7}   {:>16}\n".format("Section Name", "Start Addr", "Size", "Gap Before Section")
        )
        previous_section_end_addr = None
        for s in sections:
            # Handle the first sections
            if previous_section_end_addr is None:
                previous_section_end_addr = s[1] - 1
                gap = 0
            else:
                gap = s[1] - previous_section_end_addr - 1

            if gap == 0:
                print(f"{s[0]:<20}   0x{s[1]:08X}   0x{s[2]:05X}")
            elif gap > 0:
                print(
                    f"{s[0]:<20}   0x{s[1]:08X}   0x{s[2]:05X}             0x{gap:06X}"
                )
            elif gap < 0:
                gap *= -1
                print(
                    f"{s[0]:<20}   0x{s[1]:08X}   0x{s[2]:05X}             0x{gap:06X} *** OVERLAP ***"
                )

            previous_section_end_addr = s[1] + s[2] - 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("-a", "--all", action="store_true", help="Show all sections")
    parser.add_argument("-b", "--bt", action="store_true", help="Dialog BT .elf")
    parser.add_argument("elf_file", help="Extracts section info from elf file.")
    args = parser.parse_args()

    _process_elf(args.elf_file, verbose=args.verbose, all_sections=args.all, bt=args.bt)
