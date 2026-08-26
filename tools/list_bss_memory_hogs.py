# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os
from operator import itemgetter

bash = r"""arm-none-eabi-objdump -x pebbleos.elf | grep '\.bss' | tail -n+2 | awk '{print $5, $6}' > /tmp/bss_symbols.txt"""
print(bash)
os.system(bash)

with open("/tmp/bss_symbols.txt", "r") as f:
    syms = f.readlines()

cleaned = [sym.strip().split() for sym in syms]

parsed = [(int(clean[0], 16), clean[1]) for clean in cleaned if len(clean) > 1]

parsed.sort(key=itemgetter(0))

parsed.reverse()

print("Top 25 BSS Memory Hogs")
print("~~~~~~~~~~~~~~~~~~~~~~")
for hog in parsed[:25]:
    print("\t", hog[0], "\t", hog[1])
