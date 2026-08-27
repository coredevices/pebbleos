# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

# Quick n dirty script to add up hci_active_mode_XXX logs, by debug tag.
# It prints a list of found logs, and a list of the sums per tag.

import re

f = open("log.txt")  # noqa: SIM115
r = re.compile("[^\n]+(\\+\\+|\\-\\-)no_sniff_count : [0-9]+ \\(([A-z]+)\\)")
d = {}
for line in f:
    m = r.search(line)
    if m != None:
        is_add = m.group(1) == "++"
        tag = m.group(2)
        print("tag: {} {}".format(tag, "+" if is_add else "-"))
        if not tag in d:
            d[tag] = 0
        if is_add:
            d[tag] += 1
        else:
            d[tag] -= 1
print()
for tag, count in d.items():
    print(f"{tag} -> {count}")
