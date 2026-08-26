#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import sys
import uuid

if len(sys.argv) > 1:
    u = uuid.UUID(sys.argv[1])
else:
    u = uuid.uuid4()
uuid_array = "{{{:s}}}".format(
    ", ".join([f"0x{ord(b):x}" for b in u.get_bytes()])
)

print(f"\tBytes: '{uuid_array}'")
print(f"\tString: '{u!s}'")
