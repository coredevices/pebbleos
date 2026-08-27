#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0



import struct
import sys

from fw_binary_info import PebbleFirmwareBinaryInfo

# typedef struct ATTR_PACKED FirmwareDescription {
#   uint32_t description_length;
#   uint32_t firmware_length;
#   uint32_t checksum;
# } FirmwareDescription;
FW_DESCR_FORMAT = "<III"
FW_DESCR_SIZE = struct.calcsize(FW_DESCR_FORMAT)


def _generate_firmware_description_struct(firmware_length, firmware_crc):
    return struct.pack(FW_DESCR_FORMAT, FW_DESCR_SIZE, firmware_length, firmware_crc)


def insert_firmware_description_struct(input_binary, output_binary=None):
    fw_bin_info = PebbleFirmwareBinaryInfo(input_binary)
    with open(input_binary, "rb") as inf:
        fw_bin = inf.read()
        fw_crc = fw_bin_info.get_crc()

    if output_binary:
        with open(output_binary, "wb") as outf:
            outf.write(_generate_firmware_description_struct(len(fw_bin), fw_crc))
            outf.write(fw_bin)
    else:
        return _generate_firmware_description_struct(len(fw_bin), fw_crc) + fw_bin


def usage_and_exit():
    print(f"Usage: {sys.argv[0]} INPUT_FILE OUTPUT_FILE")
    sys.exit(1)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        usage_and_exit()
    input_binary = sys.argv[1]
    output_binary = sys.argv[2]

    insert_firmware_description_struct(input_binary, output_binary)
