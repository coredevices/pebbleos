#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Export the configured build's settings for the ./pbl developer CLI.

``pbl`` drives flashing, QEMU and the language packs from the build
directory alone, without asking CMake anything.
"""

import argparse
import json


def read_config(path):
    config = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            if value == "y":
                value = True
            elif value.startswith('"') and value.endswith('"'):
                value = value[1:-1]
            else:
                try:
                    value = int(value, 0)
                except ValueError:
                    pass
            config[key] = value
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    args = parser.parse_args()

    env = read_config(args.config)
    for entry in args.set:
        key, value = entry.split("=", 1)
        if ";" in value:
            value = [v for v in value.split(";") if v]
        env[key] = value

    with open(args.output, "w") as f:
        json.dump(env, f, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
