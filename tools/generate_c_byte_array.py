# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


def write(output_file, bytes, var_name):
    output_file.write(
        f"static const uint8_t {var_name}[] = {{\n  "
    )
    for byte, index in zip(bytes, list(range(len(bytes)))):
        if index != 0 and index % 16 == 0:
            output_file.write(f"/* bytes {index - 16} - {index} */\n  ")
        output_file.write(f"0x{byte:02x}, ")
    output_file.write("\n};\n")
