#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import os
import tempfile

import sh

TZDATA_DIR = os.path.dirname(os.path.abspath(__file__))

tmp_dir = tempfile.mkdtemp()
os.chdir(tmp_dir)

print(f"Downloading timezone data to {tmp_dir}")

sh.wget("https://ftp.iana.org/tz/tzdata-latest.tar.gz")

print("Download complete")
print(sh.ls("-la", "tzdata-latest.tar.gz").strip())

sh.tar("-xvzf", "tzdata-latest.tar.gz")

tz_file = os.path.join(TZDATA_DIR, "timezones_olson.txt")


# Process each file to convert Vanguard sections to Rearguard sections as we concatenate
# Our parser doesn't support negative DST, so we need to use the Rearguard format
def process_tzfile(filename):
    """Process a timezone file to use Rearguard format instead of Vanguard."""
    in_vanguard = False
    in_rearguard = False

    with open(filename, "r") as f:
        for line in f:
            # Detect section markers
            if "# Vanguard section" in line:
                in_vanguard = True
                in_rearguard = False
                yield line
                continue
            elif "# Rearguard section" in line:
                in_vanguard = False
                in_rearguard = True
                yield line
                continue
            elif "# End of rearguard section" in line or (
                in_rearguard
                and line.strip()
                and not line.strip().startswith("#")
                and not line.startswith("\t")
                and not line.startswith(" ")
            ):
                # End of the vanguard/rearguard block
                in_vanguard = False
                in_rearguard = False
                yield line
                continue

            if in_vanguard:
                # Comment out vanguard lines
                if line.strip() and not line.strip().startswith("#"):
                    yield "#" + line
                else:
                    yield line
            elif in_rearguard:
                # Uncomment rearguard lines
                if line.strip().startswith("#\t") or line.strip().startswith("# \t"):
                    # Remove the leading '# ' or '#\t' to uncomment
                    yield line.replace("#\t", "\t", 1).replace("# \t", "\t", 1)
                else:
                    yield line
            else:
                # Normal line, write as-is
                yield line


print("Processing timezone data to use Rearguard format...")
# backward goes last so we can just always do backreferences for links
with open(tz_file, "w") as outfile:
    for filename in [
        "africa",
        "antarctica",
        "asia",
        "australasia",
        "europe",
        "etcetera",
        "northamerica",
        "southamerica",
        "backward",
    ]:
        outfile.writelines(process_tzfile(filename))

print(f"Updated database written to {tz_file}")
