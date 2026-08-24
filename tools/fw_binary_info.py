#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import os

from pebble.commander.util.fw_binary_info import (
    PebbleFirmwareBinaryInfo as _PebbleFirmwareBinaryInfo,
)


class PebbleFirmwareBinaryInfo(_PebbleFirmwareBinaryInfo):
    """Extends the pebble-commander implementation with .elf input support,
    which needs the repo-local binutils helper."""

    def _get_footer_data_from_elf(self, path):
        import binutils

        fw_version_data = binutils.section_bytes(path, ".fw_version")
        # The GNU Build ID has 16 bytes of header data, strip it off:
        build_id_data = binutils.section_bytes(path, ".note.gnu.build-id")[16:]
        return build_id_data + fw_version_data

    def _get_footer_data(self, path):
        _, ext = os.path.splitext(path)
        if ext == ".elf":
            return self._get_footer_data_from_elf(path)
        if ext == ".bin":
            return self._get_footer_data_from_bin(path)
        raise ValueError('Unexpected extension. Must be ".bin" or ".elf"')


if __name__ == "__main__":
    import argparse
    import pprint

    parser = argparse.ArgumentParser()
    parser.add_argument("fw_bin_or_elf_path")
    args = parser.parse_args()

    fw_bin_info = PebbleFirmwareBinaryInfo(args.fw_bin_or_elf_path)

    pprint.pprint(fw_bin_info)
