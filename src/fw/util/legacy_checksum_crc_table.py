#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0



CRC_POLY = 0x04C11DB7
CRC_WIDTH = 32


def precompute_table(bits):
    lookup_table = []
    for i in xrange(2**bits):
        rr = i << (CRC_WIDTH - bits)
        for x in xrange(bits):
            if rr & 0x80000000:
                rr = (rr << 1) ^ CRC_POLY
            else:
                rr <<= 1
        lookup_table.append(rr & 0xFFFFFFFF)
    return lookup_table


print("static const uint32_t s_lookup_table[] = {")
for entry in precompute_table(4):
    print(f"  0x{entry:08x},")
print("};")
