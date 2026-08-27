# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os
import re
import sys

import freetype

MIN_CODEPOINT = 0x20
MAX_CODEPOINT = 0xFFFF
# Set a codepoint that the font doesn't know how to render
# The watch will use this glyph as the wildcard character
WILDCARD_CODEPOINT = 0x3456


class Font:
    def __init__(self, ttf_path):
        self.version = 1
        self.ttf_path = ttf_path
        # Get the font's size from the filename:
        self.basename = os.path.basename(self.ttf_path)
        m = re.search("([0-9]+)", self.basename)
        if m == None:
            sys.stderr.write(
                f"Font {self.basename}: no size found in file name...\n"
            )
            return
        self.max_height = int(m.group(0))
        self.face = freetype.Face(self.ttf_path)
        self.face.set_pixel_sizes(0, self.max_height)
        self.name = self.face.family_name + "_" + self.face.style_name
        self.wildcard_codepoint = WILDCARD_CODEPOINT
        self.number_of_glyphs = 0
        return

    def is_supported_glyph(self, codepoint):
        return self.face.get_char_index(codepoint) > 0 or (
            codepoint == chr(self.wildcard_codepoint)
        )

    def emit_codepoints(self):
        to_file = os.path.splitext(self.ttf_path)[0] + ".codepoints"
        with open(to_file, "wb") as f:
            for codepoint in range(MIN_CODEPOINT, MAX_CODEPOINT + 1):
                self.face.load_char(chr(codepoint))
                if self.is_supported_glyph(codepoint):
                    f.write(f"U+{codepoint:08d}\n".encode())

    def emit_codepoints_as_utf8(self):
        to_file = os.path.splitext(self.ttf_path)[0] + ".utf8"
        with open(to_file, "wb") as f:
            for codepoint in range(MIN_CODEPOINT, MAX_CODEPOINT + 1):
                self.face.load_char(chr(codepoint))
                if self.is_supported_glyph(codepoint):
                    f.write(chr(codepoint).encode("utf-8"))


def main():
    font_directory = "ttf"
    font_paths = []
    for _, _, filenames in os.walk(font_directory):
        for filename in filenames:
            if os.path.splitext(filename)[1] == ".ttf":
                font_paths.append(os.path.join(font_directory, filename))

    for font_path in font_paths:
        f = Font(font_path)
        f.emit_codepoints()
        f.emit_codepoints_as_utf8()


if __name__ == "__main__":
    main()
