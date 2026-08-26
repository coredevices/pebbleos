# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import codecs
import json
import os
import re


def generate_codepoint_requirements(path, encoding="utf-8", controlchars=False):
    latin_start = 0x20
    latin_end = 0x2AF
    lang = None
    codepoints = set()

    with codecs.open(path, encoding=encoding, mode="r") as fin:
        for line in fin:
            if lang is None:
                langstr = re.search(r"^\"Language: (\w*)", line)
                lang = langstr.group(1) if langstr else None
                continue

            msgstr = re.search('^msgstr "(.*)"$', line)
            if msgstr and len(msgstr.group(1)) > 0:
                for char in msgstr.group(1):
                    codepoints.update(char)

        required_codepoints = [
            ord(c)
            for c in codepoints
            if ord(c) > latin_end or (ord(c) < latin_start and controlchars)
        ]
        return {"language": lang, "codepoints": required_codepoints}


def main():
    parser = argparse.ArgumentParser(
        description="Given a PO file, generate a JSON file containing the codepoints required to display the translated strings"
    )
    parser.add_argument("input", help="Path to PO file containing translated strings")
    parser.add_argument(
        "--output", help="Path to output JSON file containing codepoints"
    )
    parser.add_argument(
        "--encoding",
        help="Set encoding of input file (default is utf-8)",
        default="utf-8",
    )
    parser.add_argument(
        "--controlchars",
        help="If set, control characters (U+0000 - U+001F) will not be excluded",
        action="store_true",
    )
    args = parser.parse_args()

    if args.output is None:
        args.output = os.path.splitext(args.input)[0] + ".json"

    fout = open(args.output, mode="w")
    fout.write(
        json.dumps(
            generate_codepoint_requirements(
                args.input, args.encoding, args.controlchars
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
