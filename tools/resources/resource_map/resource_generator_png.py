# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from resources.types.resource_object import ResourceObject
from resources.resource_map.resource_generator import ResourceGenerator

from pebble_sdk_platform import pebble_platforms

import png2pblpng
from resources.resource_map import pblpng_optimize


class PngResourceGenerator(ResourceGenerator):
    type = "png"

    @staticmethod
    def generate_object(task, definition):
        env = task.generator.env

        is_color = "color" in pebble_platforms[env.PLATFORM_NAME]["TAGS"]
        palette_name = png2pblpng.get_ideal_palette(is_color=is_color)
        image_bytes = png2pblpng.convert_png_to_pebble_png_bytes(
            task.inputs[0].abspath(), palette_name
        )
        # Pixel-identical shrink (palette/tRNS trim + refilter + zopfli IDAT);
        # self-verifying — returns the input unchanged on any doubt.
        image_bytes = pblpng_optimize.optimize_png_bytes(bytes(image_bytes))
        return ResourceObject(definition, image_bytes)
