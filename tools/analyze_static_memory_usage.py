#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse

from binutils import analyze_elf

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--fast", action="store_true")
    parser.add_argument("--sections", default="bdt")
    parser.add_argument("elf_file")
    args = parser.parse_args()

    sections = analyze_elf(args.elf_file, args.sections, args.fast)

    for s in sections.values():
        s.pprint(args.summary, args.verbose)
