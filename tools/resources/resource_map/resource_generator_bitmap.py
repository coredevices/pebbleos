# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import re

import bitmapgen
import png2pblpng
from pebble_sdk_platform import pebble_platforms
from resources.resource_map.resource_generator import ResourceGenerator
from resources.types.resource_object import ResourceObject

PNG_MIN_APP_MEMORY = 0x8000  # 32k, fairly arbitrarily


class BitmapResourceGenerator(ResourceGenerator):
    type = "bitmap"

    @staticmethod
    def definitions_from_dict(bld, definition_dict, resource_source_path):
        definitions = ResourceGenerator.definitions_from_dict(
            bld, definition_dict, resource_source_path
        )
        for d in definitions:
            d.memory_format = definition_dict.get("memoryFormat", "smallest")
            d.space_optimization = definition_dict.get("spaceOptimization", None)
            d.storage_format = definition_dict.get("storageFormat", None)
        return definitions

    @classmethod
    def generate_object(cls, task, definition):
        env = task.generator.env

        memory_format = definition.memory_format.lower()
        # Some options are mutually exclusive.
        if (
            definition.space_optimization == "memory"
            and definition.storage_format == "png"
        ):
            task.generator.bld.fatal(
                f"{definition.name}: spaceOptimization: memory and storageFormat: "
                "png are mutually exclusive."
            )
        if (
            definition.space_optimization == "storage"
            and definition.storage_format == "pbi"
        ):
            task.generator.bld.fatal(
                f"{definition.name}: spaceOptimization: storage and storageFormat: "
                "pbi are mutually exclusive."
            )
        if definition.storage_format == "png" and memory_format == "1bit":
            task.generator.bld.fatal(
                f"{definition.name}: PNG storage does not support non-palettised 1-bit images."
            )

        # If storage_format is not specified, it is completely determined by space_optimization.
        if (
            definition.storage_format is None
            and definition.space_optimization is not None
        ):
            format_mapping = {
                "storage": "png",
                "memory": "pbi",
            }
            try:
                definition.storage_format = format_mapping[
                    definition.space_optimization
                ]
            except KeyError:
                task.generator.bld.fatal(
                    "{}: Invalid spaceOptimization '{}'. Use one of {}.".format(
                        definition.name,
                        definition.space_optimization,
                        ", ".join(format_mapping.keys()),
                    )
                )

        if definition.storage_format is None:
            if (
                pebble_platforms[env.PLATFORM_NAME]["MAX_APP_MEMORY_SIZE"]
                < PNG_MIN_APP_MEMORY
            ):
                definition.storage_format = "pbi"
            else:
                definition.storage_format = "png"

        # At this point, what we want to do should be completely determined (though, depending on
        # image content, not necessarily actually possible); begin figuring out what to actually do.

        is_color = "color" in pebble_platforms[env.PLATFORM_NAME]["TAGS"]
        palette_name = png2pblpng.get_ideal_palette(is_color)
        _, _, bitdepth, _ = png2pblpng.get_palette_for_png(
            task.inputs[0].abspath(), palette_name, png2pblpng.DEFAULT_COLOR_REDUCTION
        )

        formats = {
            "1bit",
            "8bit",
            "smallest",
            "smallestpalette",
            "1bitpalette",
            "2bitpalette",
            "4bitpalette",
        }
        if memory_format not in formats:
            task.generator.bld.fatal(
                "{}: Invalid memoryFormat {} (pick one of {}).".format(
                    definition.name, definition.memory_format, ", ".join(formats)
                )
            )

        # "smallest" is always palettised, unless the image has too many colours, in which case it
        # cannot be.
        if memory_format == "smallest":
            if bitdepth <= 4:
                memory_format = "smallestpalette"
            else:
                memory_format = "8bit"

        if "palette" in memory_format:
            if memory_format == "smallestpalette":
                # If they asked for "smallestpalette", replace that with its actual value.
                if bitdepth > 4:
                    task.generator.bld.fatal(
                        f"{definition.name} has too many colours for a palettised image"
                        "(max 16), but 'SmallestPalette' specified."
                    )
                else:
                    memory_format = f"{bitdepth}bitpalette"

            # Pull out however many bits we're supposed to use (which is exact, not a min or max)
            bits = int(re.match(r"^(\d+)bitpalette$", memory_format).group(1))
            if bits < bitdepth:
                task.generator.bld.fatal(
                    f"{definition.name}: requires at least {bitdepth} bits."
                )
            if bits > 2 and not is_color:
                task.generator.bld.fatal(
                    f"{definition.name}: can't use more than two bits on a black-and-white"
                    "platform."
                )

            if definition.storage_format == "pbi":
                pb = bitmapgen.PebbleBitmap(
                    task.inputs[0].abspath(),
                    bitmap_format="color",
                    bitdepth=bits,
                    crop=False,
                    palette_name=palette_name,
                )
                return ResourceObject(definition, pb.convert_to_pbi())
            else:
                image_bytes = png2pblpng.convert_png_to_pebble_png_bytes(
                    task.inputs[0].abspath(), palette_name, bitdepth=bits
                )
                return ResourceObject(definition, image_bytes)
        else:
            if memory_format == "1bit":
                pb = bitmapgen.PebbleBitmap(
                    task.inputs[0].abspath(), bitmap_format="bw", crop=False
                )
                return ResourceObject(definition, pb.convert_to_pbi())
            elif memory_format == "8bit":
                if not is_color:
                    task.generator.bld.fatal(
                        f"{definition.name}: can't use more than two bits on a black-and-white"
                        "platform."
                    )
                # generate an 8-bit pbi or png, as appropriate.
                if definition.storage_format == "pbi":
                    pb = bitmapgen.PebbleBitmap(
                        task.inputs[0].abspath(),
                        bitmap_format="color_raw",
                        crop=False,
                        palette_name=palette_name,
                    )
                    return ResourceObject(definition, pb.convert_to_pbi())
                else:
                    image_bytes = png2pblpng.convert_png_to_pebble_png_bytes(
                        task.inputs[0].abspath(), palette_name, bitdepth=8
                    )
                    return ResourceObject(definition, image_bytes)

        raise RuntimeError("Got to the end without doing anything?")
