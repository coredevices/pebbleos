#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import os
import os.path
import sys
from shutil import copy2
from struct import pack, unpack
from subprocess import PIPE, Popen

import stm32_crc
from pbpack import ResourcePack

# Pebble App Metadata Struct
# These are offsets of the PebbleProcessInfo struct in src/fw/app_management/pebble_process_info.h
HEADER_ADDR = 0x0  # 8 bytes
STRUCT_VERSION_ADDR = 0x8  # 2 bytes
SDK_VERSION_ADDR = 0xA  # 2 bytes
APP_VERSION_ADDR = 0xC  # 2 bytes
LOAD_SIZE_ADDR = 0xE  # 2 bytes
OFFSET_ADDR = 0x10  # 4 bytes
CRC_ADDR = 0x14  # 4 bytes
NAME_ADDR = 0x18  # 32 bytes
COMPANY_ADDR = 0x38  # 32 bytes
ICON_RES_ID_ADDR = 0x58  # 4 bytes
JUMP_TABLE_ADDR = 0x5C  # 4 bytes
FLAGS_ADDR = 0x60  # 4 bytes
NUM_RELOC_ENTRIES_ADDR = 0x64  # 4 bytes
UUID_ADDR = 0x68  # 16 bytes
RESOURCE_CRC_ADDR = 0x78  # 4 bytes
RESOURCE_TIMESTAMP_ADDR = 0x7C  # 4 bytes
VIRTUAL_SIZE_ADDR = 0x80  # 2 bytes
STRUCT_SIZE_BYTES = 0x82

# Pebble App Flags
# These are PebbleAppFlags from src/fw/app_management/pebble_process_info.h
PROCESS_INFO_STANDARD_APP = 0
PROCESS_INFO_WATCH_FACE = 1 << 0
PROCESS_INFO_VISIBILITY_HIDDEN = 1 << 1
PROCESS_INFO_VISIBILITY_SHOWN_ON_COMMUNICATION = 1 << 2
PROCESS_INFO_ALLOW_JS = 1 << 3
PROCESS_INFO_HAS_WORKER = 1 << 4

# Max app size, including the struct and reloc table. Fallback for callers that pass no limit; the
# build passes the platform's MAX_APP_BINARY_SIZE from tools/pebble_sdk_platform.py.
# Note that even if the app is smaller than this, it still may be too big, as it needs to share this
# space with applib/ which changes in size from release to release.
MAX_APP_BINARY_SIZE = 0x10000

# PebbleProcessInfo.load_size and .virtual_size are uint16_t, so neither can exceed this whatever
# the platform allows for the total. Only the reloc table, stored past load_size, can use the rest.
MAX_PROCESS_INFO_SIZE_FIELD = 0xFFFF

# This number is a rough estimate, but should not be less than the available space.
# Currently, app_state uses up a small part of the app space.
# See also APP_RAM in stm32f2xx_flash_fw.ld and APP in pebble_app.ld.
MAX_APP_MEMORY_SIZE = 24 * 1024

# This number is a rough estimate, but should not be less than the available space.
# Currently, worker_state uses up a small part of the worker space.
# See also WORKER_RAM in stm32f2xx_flash_fw.ld
MAX_WORKER_MEMORY_SIZE = 10 * 1024

ENTRY_PT_SYMBOL = "main"
JUMP_TABLE_ADDR_SYMBOL = "pbl_table_addr"
DEBUG = False


class InvalidBinaryError(Exception):
    pass


def inject_metadata(
    target_binary,
    target_elf,
    resources_file,
    timestamp,
    allow_js=False,
    has_worker=False,
    max_binary_size=None,
):
    if max_binary_size is None:
        max_binary_size = MAX_APP_BINARY_SIZE

    if target_binary[-4:] != ".bin":
        raise Exception(
            "Invalid filename <%s>! The filename should end in .bin" % target_binary
        )

    def get_nm_output(elf_file):
        nm_process = Popen(["arm-none-eabi-nm", elf_file], stdout=PIPE)
        # Popen.communicate returns a tuple of (stdout, stderr)
        nm_output = nm_process.communicate()[0].decode("utf8")

        if not nm_output:
            raise InvalidBinaryError()

        nm_output = [line.split() for line in nm_output.splitlines()]
        return nm_output

    def get_symbol_addr(nm_output, symbol):
        # nm output looks like the following...
        #
        #          U _ITM_registerTMCloneTable
        # 00000084 t jump_to_pbl_function
        #          U _Jv_RegisterClasses
        # 0000009c T main
        # 00000130 T memset
        #
        # We don't care about the lines that only have two columns, they're not functions.

        for sym in nm_output:
            if symbol == sym[-1] and len(sym) == 3:
                return int(sym[0], 16)

        raise Exception(
            "Could not locate symbol <%s> in binary! Failed to inject app metadata"
            % (symbol)
        )

    def get_virtual_size(elf_file):
        """returns the virtual size (static memory usage, .text + .data + .bss) in bytes"""

        readelf_bss_process = Popen(
            "arm-none-eabi-readelf -S '%s'" % elf_file, shell=True, stdout=PIPE
        )
        readelf_bss_output = readelf_bss_process.communicate()[0].decode("utf8")

        # readelf -S output looks like the following...
        #
        # [Nr] Name              Type            Addr     Off    Size   ES Flg Lk Inf Al
        # [ 0]                   NULL            00000000 000000 000000 00      0   0  0
        # [ 1] .header           PROGBITS        00000000 008000 000082 00   A  0   0  1
        # [ 2] .text             PROGBITS        00000084 008084 0006be 00  AX  0   0  4
        # [ 3] .rel.text         REL             00000000 00b66c 0004d0 08     23   2  4
        # [ 4] .data             PROGBITS        00000744 008744 000004 00  WA  0   0  4
        # [ 5] .bss              NOBITS          00000748 008748 000054 00  WA  0   0  4

        last_section_end_addr = 0

        # Find the .bss section and calculate the size based on the end of the .bss section
        for line in readelf_bss_output.splitlines():
            if len(line) < 10:
                continue

            # Carve off the first column, since it sometimes has a space in it which screws up the
            # split.
            if "]" not in line:
                continue
            line = line[line.index("]") + 1 :]

            columns = line.split()
            if len(columns) < 6:
                continue

            if columns[0] == ".bss" or columns[0] == ".data" and last_section_end_addr == 0:
                addr = int(columns[2], 16)
                size = int(columns[4], 16)
                last_section_end_addr = addr + size

        if last_section_end_addr != 0:
            return last_section_end_addr

        sys.stderr.writeline(
            "Failed to parse ELF sections while calculating the virtual size\n"
        )
        sys.stderr.write(readelf_bss_output)
        raise Exception(
            "Failed to parse ELF sections while calculating the virtual size"
        )

    def get_relocate_entries(elf_file):
        """returns a list of all the locations requiring an offset"""
        # TODO: insert link to the wiki page I'm about to write about PIC and relocatable values
        entries = []

        # get the .data locations
        readelf_relocs_process = Popen(
            ["arm-none-eabi-readelf", "-r", elf_file], stdout=PIPE
        )
        readelf_relocs_output = readelf_relocs_process.communicate()[0].decode("utf8")
        lines = readelf_relocs_output.splitlines()

        i = 0
        reading_section = False
        while i < len(lines):
            if not reading_section:
                # look for the next section
                if lines[i].startswith("Relocation section '.rel.data"):
                    reading_section = True
                    i += 1  # skip the column title section
            else:
                if len(lines[i]) == 0:
                    # end of the section
                    reading_section = False
                else:
                    entries.append(int(lines[i].split(" ")[0], 16))
            i += 1

        # get any Global Offset Table (.got) entries
        readelf_relocs_process = Popen(
            ["arm-none-eabi-readelf", "--sections", elf_file], stdout=PIPE
        )
        readelf_relocs_output = readelf_relocs_process.communicate()[0].decode("utf8")
        lines = readelf_relocs_output.splitlines()
        for line in lines:
            # We shouldn't need to do anything with the Procedure Linkage Table since we don't
            # actually export functions
            if ".got" in line and ".got.plt" not in line:
                words = line.split(" ")
                while "" in words:
                    words.remove("")
                section_label_idx = words.index(".got")
                addr = int(words[section_label_idx + 2], 16)
                length = int(words[section_label_idx + 4], 16)
                for i in range(addr, addr + length, 4):
                    entries.append(i)
                break

        return entries

    nm_output = get_nm_output(target_elf)

    try:
        app_entry_address = get_symbol_addr(nm_output, ENTRY_PT_SYMBOL)
    except:
        raise Exception("Missing app entry point! Must be `int main(void) { ... }` ")
    jump_table_address = get_symbol_addr(nm_output, JUMP_TABLE_ADDR_SYMBOL)

    reloc_entries = get_relocate_entries(target_elf)

    statinfo = os.stat(target_binary)
    app_load_size = statinfo.st_size

    if resources_file is not None:
        with open(resources_file, "rb") as f:
            pbpack = ResourcePack.deserialize(f, is_system=False)
            resource_crc = pbpack.get_content_crc()
    else:
        resource_crc = 0

    if DEBUG:
        copy2(target_binary, target_binary + ".orig")

    with open(target_binary, "r+b") as f:
        total_app_image_size = app_load_size + (len(reloc_entries) * 4)
        if total_app_image_size > max_binary_size:
            raise Exception(
                "App image size is %u (app %u relocation table %u). Must be smaller "
                "than %u bytes"
                % (
                    total_app_image_size,
                    app_load_size,
                    len(reloc_entries) * 4,
                    max_binary_size,
                )
            )

        # Checked here so the pack() below cannot raise a bare struct.error.
        if app_load_size > MAX_PROCESS_INFO_SIZE_FIELD:
            raise Exception(
                "App load size is %u bytes. The loaded image must be %u bytes or smaller, "
                "because PebbleProcessInfo.load_size is a uint16_t. The relocation table is "
                "stored past the loaded image and does not count towards this."
                % (app_load_size, MAX_PROCESS_INFO_SIZE_FIELD)
            )

        def read_value_at_offset(offset, format_str, size):
            f.seek(offset)
            return unpack(format_str, f.read(size))

        app_bin = f.read()
        app_crc = stm32_crc.crc32(app_bin[STRUCT_SIZE_BYTES:])

        [app_flags] = read_value_at_offset(FLAGS_ADDR, "<L", 4)

        if allow_js:
            app_flags = app_flags | PROCESS_INFO_ALLOW_JS

        if has_worker:
            app_flags = app_flags | PROCESS_INFO_HAS_WORKER

        app_virtual_size = get_virtual_size(target_elf)

        # Same uint16_t ceiling as load_size, on the .text + .data + .bss total this time.
        if app_virtual_size > MAX_PROCESS_INFO_SIZE_FIELD:
            raise Exception(
                "App virtual size is %u bytes (.text + .data + .bss). Must be %u bytes or "
                "smaller, because PebbleProcessInfo.virtual_size is a uint16_t."
                % (app_virtual_size, MAX_PROCESS_INFO_SIZE_FIELD)
            )

        struct_changes = {
            "load_size": app_load_size,
            "entry_point": "0x%08x" % app_entry_address,
            "symbol_table": "0x%08x" % jump_table_address,
            "flags": app_flags,
            "crc": "0x%08x" % app_crc,
            "num_reloc_entries": "0x%08x" % len(reloc_entries),
            "resource_crc": "0x%08x" % resource_crc,
            "timestamp": timestamp,
            "virtual_size": app_virtual_size,
        }

        def write_value_at_offset(offset, format_str, value):
            f.seek(offset)
            f.write(pack(format_str, value))

        write_value_at_offset(LOAD_SIZE_ADDR, "<H", app_load_size)
        write_value_at_offset(OFFSET_ADDR, "<L", app_entry_address)
        write_value_at_offset(CRC_ADDR, "<L", app_crc)

        write_value_at_offset(RESOURCE_CRC_ADDR, "<L", resource_crc)
        write_value_at_offset(RESOURCE_TIMESTAMP_ADDR, "<L", timestamp)

        write_value_at_offset(JUMP_TABLE_ADDR, "<L", jump_table_address)

        write_value_at_offset(FLAGS_ADDR, "<L", app_flags)

        write_value_at_offset(NUM_RELOC_ENTRIES_ADDR, "<L", len(reloc_entries))

        write_value_at_offset(VIRTUAL_SIZE_ADDR, "<H", app_virtual_size)

        # Write the reloc_entries past the end of the binary. This expands the size of the binary,
        # but this new stuff won't actually be loaded into ram.
        f.seek(app_load_size)
        f.writelines(pack("<L", entry) for entry in reloc_entries)

        f.flush()

    return struct_changes
