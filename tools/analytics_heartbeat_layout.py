#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Extract the native_heartbeat_record struct layout from an ELF file using DWARF
debug info. Useful for building parsers for the analytics DLS blob.

Usage:
    python3 tools/analytics_heartbeat_layout.py <elf_file>
"""

import argparse
import struct
import sys

from elftools.elf.elffile import ELFFile

STRUCT_NAME = "native_heartbeat_record"
HEADER_FIELDS = {"version", "timestamp", "build_id"}
METRIC_PREFIX = "metric_"

# Map DWARF base type encodings to human-readable names
_DW_ATE_NAMES = {
    0x05: "signed",
    0x06: "char",
    0x07: "unsigned",
    0x08: "unsigned char",
}


def _get_type_die(die):
    """Follow DW_AT_type references until we reach a base or array type."""
    while True:
        if "DW_AT_type" not in die.attributes:
            return die
        die = die.get_DIE_from_attribute("DW_AT_type")
    return die


def _resolve_type_info(die):
    """Resolve a member's type to (type_name, is_array, array_count)."""
    type_die = (
        die.get_DIE_from_attribute("DW_AT_type")
        if "DW_AT_type" in die.attributes
        else die
    )

    is_array = False
    array_count = 0

    # Walk through typedefs, const, volatile qualifiers
    while type_die.tag in (
        "DW_TAG_typedef",
        "DW_TAG_const_type",
        "DW_TAG_volatile_type",
    ):
        if "DW_AT_type" not in type_die.attributes:
            break
        type_die = type_die.get_DIE_from_attribute("DW_AT_type")

    # Handle arrays
    if type_die.tag == "DW_TAG_array_type":
        is_array = True
        elem_type = type_die.get_DIE_from_attribute("DW_AT_type")
        # Get array bound from subrange child
        for child in type_die.iter_children():
            if child.tag == "DW_TAG_subrange_type":
                if "DW_AT_upper_bound" in child.attributes:
                    array_count = child.attributes["DW_AT_upper_bound"].value + 1
                elif "DW_AT_count" in child.attributes:
                    array_count = child.attributes["DW_AT_count"].value
        # Resolve element type through typedefs
        while elem_type.tag in (
            "DW_TAG_typedef",
            "DW_TAG_const_type",
            "DW_TAG_volatile_type",
        ):
            if "DW_AT_type" not in elem_type.attributes:
                break
            elem_type = elem_type.get_DIE_from_attribute("DW_AT_type")
        type_die = elem_type

    # Get base type name
    if type_die.tag == "DW_TAG_base_type":
        encoding = type_die.attributes.get("DW_AT_encoding")
        byte_size = type_die.attributes.get("DW_AT_byte_size")
        if encoding and byte_size:
            enc_val = encoding.value
            size_val = byte_size.value
            enc_name = _DW_ATE_NAMES.get(enc_val, f"enc_{enc_val}")
            if is_array and enc_val in (0x06, 0x08) and size_val == 1:  # char array
                return f"char[{array_count}]", True, array_count
            else:
                type_name = f"{enc_name}{size_val * 8}"
                if is_array:
                    type_name = f"{type_name}[{array_count}]"
                return type_name, is_array, array_count
        name = type_die.attributes.get("DW_AT_name")
        if name:
            type_name = (
                name.value.decode()
                if isinstance(name.value, bytes)
                else str(name.value)
            )
            if is_array:
                type_name = f"{type_name}[{array_count}]"
            return type_name, is_array, array_count

    return "unknown", is_array, array_count


def _die_name(die):
    """Extract the name string from a DIE, or None."""
    attr = die.attributes.get("DW_AT_name")
    if attr is None:
        return None
    return attr.value.decode() if isinstance(attr.value, bytes) else str(attr.value)


def _find_cu_offset(elf_file, dwarf_info):
    """Find the CU offset containing STRUCT_NAME using a raw .debug_str +
    .debug_info binary search, avoiding full DWARF parsing of every CU.
    """
    debug_str = elf_file.get_section_by_name(".debug_str")
    debug_info = elf_file.get_section_by_name(".debug_info")
    if not debug_str or not debug_info:
        return None

    str_offset = debug_str.data().find(STRUCT_NAME.encode() + b"\x00")
    if str_offset == -1:
        return None

    # Find where this string offset is referenced in .debug_info
    needle = struct.pack("<I", str_offset)
    info_data = debug_info.data()
    pos = info_data.find(needle)
    if pos == -1:
        return None

    # Identify which CU contains this offset
    for cu in dwarf_info.iter_CUs():
        cu_end = (
            cu.cu_offset + cu["unit_length"] + cu.structs.initial_length_field_size()
        )
        if cu.cu_offset <= pos < cu_end:
            return cu.cu_offset

    return None


def _find_struct(elf_file):
    """Find the native_heartbeat_record structure DIE in DWARF info.

    Uses a fast binary search in .debug_str/.debug_info to locate the right CU,
    then only parses that single CU's top-level children.
    """
    dwarf_info = elf_file.get_dwarf_info()
    if dwarf_info is None:
        print("Error: ELF file has no DWARF debug info", file=sys.stderr)
        sys.exit(1)

    target_cu_offset = _find_cu_offset(elf_file, dwarf_info)

    for cu in dwarf_info.iter_CUs():
        if target_cu_offset is not None and cu.cu_offset != target_cu_offset:
            continue
        for die in cu.get_top_DIE().iter_children():
            if die.tag == "DW_TAG_structure_type" and _die_name(die) == STRUCT_NAME:
                return die

    return None


def _get_build_id(elf_file):
    """Extract the GNU build ID from the ELF .note.gnu.build-id section."""
    sect = elf_file.get_section_by_name(".note.gnu.build-id")
    if not sect:
        return None
    data = sect.data()
    namesz, descsz, _ = struct.unpack("<III", data[:12])
    desc_off = 12 + ((namesz + 3) & ~3)
    return data[desc_off : desc_off + descsz].hex()


def _get_field_info(member_die, struct_die):
    """Extract (name, offset, byte_size, type_name) for a struct member DIE."""
    field_name = _die_name(member_die)
    if not field_name:
        return None

    offset = 0
    if "DW_AT_data_member_location" in member_die.attributes:
        loc = member_die.attributes["DW_AT_data_member_location"]
        offset = loc.value if isinstance(loc.value, int) else 0

    type_name, is_array, _ = _resolve_type_info(member_die)

    byte_size = None
    if "DW_AT_type" in member_die.attributes:
        type_die = member_die.get_DIE_from_attribute("DW_AT_type")
        while type_die.tag in (
            "DW_TAG_typedef",
            "DW_TAG_const_type",
            "DW_TAG_volatile_type",
        ):
            if "DW_AT_type" not in type_die.attributes:
                break
            type_die = type_die.get_DIE_from_attribute("DW_AT_type")
        if "DW_AT_byte_size" in type_die.attributes:
            byte_size = type_die.attributes["DW_AT_byte_size"].value

    if byte_size is None and is_array:
        struct_size = struct_die.attributes.get("DW_AT_byte_size")
        siblings = [c for c in struct_die.iter_children() if c.tag == "DW_TAG_member"]
        for i, sib in enumerate(siblings):
            if _die_name(sib) == field_name and i + 1 < len(siblings):
                next_loc = siblings[i + 1].attributes.get("DW_AT_data_member_location")
                if next_loc:
                    next_off = next_loc.value if isinstance(next_loc.value, int) else 0
                    byte_size = next_off - offset
                break
            elif _die_name(sib) == field_name:
                if struct_size:
                    byte_size = struct_size.value - offset
                break

    return field_name, offset, byte_size, type_name


def dump_layout(elf_path):
    with open(elf_path, "rb") as f:
        elf_file = ELFFile(f)

        build_id = _get_build_id(elf_file)
        struct_die = _find_struct(elf_file)

        if struct_die is None:
            print(
                f"Error: struct '{STRUCT_NAME}' not found in DWARF info",
                file=sys.stderr,
            )
            sys.exit(1)

        struct_size = struct_die.attributes.get("DW_AT_byte_size")
        size_str = f" ({struct_size.value} bytes)" if struct_size else ""

        if build_id:
            print(f"Build ID: {build_id}")

        print(f"\n{STRUCT_NAME}{size_str}")

        # Collect all fields
        members = [c for c in struct_die.iter_children() if c.tag == "DW_TAG_member"]
        fields = [_get_field_info(m, struct_die) for m in members]
        fields = [f for f in fields if f is not None]

        header_fields = [f for f in fields if f[0] in HEADER_FIELDS]
        metric_fields = [f for f in fields if f[0] not in HEADER_FIELDS]

        sep = "─" * 60

        if header_fields:
            print("\nHeader:")
            print(sep)
            print(f"{'Offset':>6}  {'Size':>4}  {'Type':<16}  {'Field'}")
            print(sep)
            for name, offset, byte_size, type_name in header_fields:
                size_str = str(byte_size) if byte_size is not None else "?"
                print(f"{offset:>6}  {size_str:>4}  {type_name:<16}  {name}")
            print(sep)

        if metric_fields:
            print("\nMetrics:")
            print(sep)
            print(f"{'Offset':>6}  {'Size':>4}  {'Type':<16}  {'Field'}")
            print(sep)
            prev_display_name = None
            for name, offset, byte_size, type_name in metric_fields:
                size_str = str(byte_size) if byte_size is not None else "?"
                display_name = (
                    name.removeprefix(METRIC_PREFIX)
                )
                if display_name.endswith("_scale"):
                    scaled_field = prev_display_name or display_name
                    print(
                        f"{offset:>6}  {size_str:>4}  {type_name:<16}  ↳ scale for {scaled_field}"
                    )
                else:
                    print(
                        f"{offset:>6}  {size_str:>4}  {type_name:<16}  {display_name}"
                    )
                    prev_display_name = display_name
            print(sep)


def main():
    parser = argparse.ArgumentParser(
        description=f"Extract {STRUCT_NAME} layout from ELF DWARF info"
    )
    parser.add_argument("elf_file", help="Path to the ELF file with debug info")
    args = parser.parse_args()

    dump_layout(args.elf_file)


if __name__ == "__main__":
    main()
