# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


# These constants are borrowed from animation_timing.h:
ANIMATION_NORMALIZED_MAX = 65535
ANIMATION_NORMALIZED_MIN = 0

# "Standard" animation curves, borrowed from the interwebs:
d = ANIMATION_NORMALIZED_MAX
b = ANIMATION_NORMALIZED_MIN
c = ANIMATION_NORMALIZED_MAX


def easeInOut(t):
    t = t / (d / 2)
    if t < 1.0:
        return c / 2 * t * t + b
    t -= 1
    return -c / 2 * (t * (t - 2) - 1) + b


def easeOut(t):
    t /= d
    return -c * t * (t - 2) + b


def easeIn(t):
    t /= d
    return c * t * t + b


def print_table(name, func):
    nums_per_row = 4
    table = [func(float(t)) for t in range(0, 65537, 2048)]
    print(f"static const uint16_t {name}_table[33] = {{")
    for i in range(0, len(table), nums_per_row):
        print(
            "    " + ", ".join(str(int(n)) for n in table[i : i + nums_per_row]) + ","
        )
    print("};\n")


print_table("ease_in", easeIn)
print_table("ease_out", easeOut)
print_table("ease_in_out", easeInOut)
