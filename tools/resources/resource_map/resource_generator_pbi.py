# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os
import subprocess
import tempfile

import bitmapgen
from pebble_sdk_platform import pebble_platforms
from resources.resource_map.resource_generator import ResourceGenerator
from resources.types.resource_object import ResourceObject


def _pbi_generator(task, definition, format_str):
    input_path = task.inputs[0].abspath()

    # Check if the input is an SVG file
    if input_path.lower().endswith(".svg"):
        # Convert SVG to PNG using rsvg-convert with white background
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp_png:
            tmp_png_path = tmp_png.name

        try:
            # Use rsvg-convert to convert SVG to PNG with white background (no transparency)
            subprocess.check_call(
                [
                    "rsvg-convert",
                    input_path,
                    "--background-color=white",
                    "-o",
                    tmp_png_path,
                ]
            )

            # Use the temporary PNG file for bitmap generation
            pb = bitmapgen.PebbleBitmap(tmp_png_path, bitmap_format=format_str)
            result = ResourceObject(definition, pb.convert_to_pbi())
        finally:
            # Clean up temporary PNG file
            if os.path.exists(tmp_png_path):
                os.unlink(tmp_png_path)

        return result
    else:
        # Original PNG handling
        pb = bitmapgen.PebbleBitmap(input_path, bitmap_format=format_str)
        return ResourceObject(definition, pb.convert_to_pbi())


class PbiResourceGenerator(ResourceGenerator):
    type = "pbi"

    @staticmethod
    def generate_object(task, definition):
        return _pbi_generator(task, definition, "bw")


class Pbi8ResourceGenerator(ResourceGenerator):
    type = "pbi8"

    @staticmethod
    def generate_object(task, definition):
        env = task.generator.env

        format = (
            "color" if "color" in pebble_platforms[env.PLATFORM_NAME]["TAGS"] else "bw"
        )

        return _pbi_generator(task, definition, format)


# This implementation is in the "pbi" file because it's implemented using pbis, even though it's
# named with "png" prefix and we have a "png" file.
class PngTransResourceGenerator(ResourceGenerator):
    type = "png-trans"

    @staticmethod
    def generate_object(task, definition):
        if "WHITE" in definition.name:
            color_map = bitmapgen.WHITE_COLOR_MAP
        elif "BLACK" in definition.name:
            color_map = bitmapgen.BLACK_COLOR_MAP
        else:
            task.generator.bld.fatal(
                "png-trans with neither white nor black in the name: "
                + definition.name
            )

        pb = bitmapgen.PebbleBitmap(task.inputs[0].abspath(), color_map=color_map)
        return ResourceObject(definition, pb.convert_to_pbi())
