#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import sys

from pebble.commander.util.fw_binary_info import insert_firmware_description_struct


def usage_and_exit():
    print("Usage: %s INPUT_FILE OUTPUT_FILE" % sys.argv[0])
    sys.exit(1)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        usage_and_exit()
    input_binary = sys.argv[1]
    output_binary = sys.argv[2]

    insert_firmware_description_struct(input_binary, output_binary)
