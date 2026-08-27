#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


# Little tool to generate rounded rect insets, stored as 4 bit uints, packed together into a uint32.


def calc_lookup(radius, is_bottom):
    insets = [0] * (radius + 1)

    f = 1 - radius
    ddF_x = 1
    ddF_y = -2 * radius
    x = 0
    y = radius
    while x < y:
        if f >= 0:
            y -= 1
            ddF_y += 2
            f += ddF_y

        x += 1
        ddF_x += 2
        f += ddF_x

        insets[radius - y] = radius - x
        insets[radius - x] = radius - y

    pack = 0
    rng = range(radius) if (is_bottom) else range(radius - 1, -1, -1)
    for i in rng:
        pack = (pack << 4) | insets[i]
    return pack


def main():
    with open("roundrect.h", "w") as f:
        f.write("static const uint32_t round_top_corner_lookup[] = {\n\t0x0, ")
        f.writelines(f"0x{calc_lookup(radius, False):02x}, " for radius in range(1, 9))
        f.write("\n};\n")
        f.write("static const uint32_t round_bottom_corner_lookup[] = {\n\t0x0, ")
        f.writelines(f"0x{calc_lookup(radius, True):02x}, " for radius in range(1, 9))
        f.write("\n};\n")


if __name__ == "__main__":
    main()
