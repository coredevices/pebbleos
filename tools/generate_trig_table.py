#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import math


def print_sin_table():
    print("{")
    for a in range(0, 0xFFFF + 1, 0xFF):
        print(str(round(math.sin(a * math.pi / 0xFFFF / 2.0) * 0xFFFF)) + ",")
    print("}")


def print_atan_table():
    print("{")
    for a in range(0, 0xFFFF + 1, 0xFF):
        b = float(a) / float(0xFFFF)
        print(str(round(math.atan(b) * 0x8000 / math.pi) * 8) + ",")
    print("}")


print_sin_table()
print_atan_table()
