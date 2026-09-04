#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0



CRC_POLY = 0xEDB88320


def crc_table(bits):
    lookup_table = []
    for i in range(2**bits):
        rr = i * 16
        for x in range(8):
            rr = (rr >> 1) ^ (-(rr & 1) & CRC_POLY)
        lookup_table.append(rr & 0xFFFFFFFF)
    return lookup_table


table = [f"0x{entry:08x}," for entry in crc_table(4)]
chunks = zip(*[iter(table)] * 4)

print("static const uint32_t s_lookup_table[] = {")
for chunk in chunks:
    print("  " + " ".join(chunk))
print("};")
