# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os
import re
from pathlib import Path
from threading import Lock

from font.fontgen import MAX_GLYPHS, MAX_GLYPHS_EXTENDED, Font
from pebble_sdk_platform import pebble_platforms
from resources.resource_map.resource_generator import ResourceGenerator
from resources.types.resource_definition import ResourceDefinition
from resources.types.resource_object import ResourceObject


class FontResourceGenerator(ResourceGenerator):
    """
    ResourceGenerator for the 'font' type
    """

    type = "font"
    lock = Lock()

    @staticmethod
    def _max_glyph_size(env):
        return pebble_platforms[env.PLATFORM_NAME]["MAX_FONT_GLYPH_SIZE"]

    @staticmethod
    def _apply_font_fields(definition, definition_dict, max_glyph_size):
        """Set the font-specific attributes that build_font_data() reads."""
        definition.max_glyph_size = max_glyph_size
        definition.character_list = definition_dict.get("characterList")
        definition.character_regex = definition_dict.get("characterRegex")
        definition.compatibility = definition_dict.get("compatibility")
        definition.compress = definition_dict.get("compress")
        definition.extended = bool(definition_dict.get("extended"))
        definition.tracking_adjust = definition_dict.get("trackingAdjust")
        definition.pixel_height = definition_dict.get("pixelHeight")
        return definition

    @staticmethod
    def definitions_from_dict(bld, definition_dict, resource_source_path):
        definitions = ResourceGenerator.definitions_from_dict(
            bld, definition_dict, resource_source_path
        )

        max_glyph_size = FontResourceGenerator._max_glyph_size(bld.env)
        for d in definitions:
            FontResourceGenerator._apply_font_fields(d, definition_dict, max_glyph_size)

        return definitions

    @staticmethod
    def font_definition_from_dict(env, definition_dict, file=""):
        """Build a font ResourceDefinition without a waf build context.

        For callers that already know the font path and codepoint list (e.g.
        language-pack generation) and only need build_font_data(); skips the
        build-context-dependent filename resolution definitions_from_dict()
        does. Only the configured env (for the platform's max glyph size) is
        required.
        """
        d = ResourceDefinition(definition_dict["type"], definition_dict["name"], file)
        FontResourceGenerator._apply_font_fields(
            d, definition_dict, FontResourceGenerator._max_glyph_size(env)
        )
        return d

    @classmethod
    def generate_object(cls, task, definition):
        font_path = task.inputs[0].abspath()
        font_ext = os.path.splitext(font_path)[-1]
        if font_ext.lower() in (".ttf", ".otf", ".bdf"):
            font_data = cls.build_font_data(font_path, definition)
        elif font_ext.lower() == ".pbf":
            font_data = Path(font_path).read_bytes()
        else:
            raise RuntimeError(f"Unsupported font format: {font_ext}")

        return ResourceObject(definition, font_data)

    @classmethod
    def build_font_data(cls, ttf_path, definition):
        # PBL-23964: it turns out that font generation is not thread-safe with freetype
        # 2.4 (and possibly later versions). To avoid running into this, we use a lock.
        with cls.lock:
            name_height = cls._get_font_height_from_name(definition.name)
            height = getattr(definition, "pixel_height", None) or name_height
            is_legacy = definition.compatibility == "2.7"
            max_glyphs = MAX_GLYPHS_EXTENDED if definition.extended else MAX_GLYPHS

            # Extended fonts sit on the base font's baseline, not their render size.
            baseline = name_height if definition.extended else None

            font = Font(
                ttf_path,
                height,
                max_glyphs,
                definition.max_glyph_size,
                is_legacy,
                baseline=baseline,
            )

            if definition.character_regex is not None:
                font.set_regex_filter(definition.character_regex)

            if definition.character_list is not None:
                font.set_codepoint_list(definition.character_list)

            if definition.compress:
                font.set_compression(definition.compress)

            if definition.tracking_adjust is not None:
                font.set_tracking_adjust(definition.tracking_adjust)

            font.build_tables()
            return font.bitstring()

    @staticmethod
    def _get_font_height_from_name(name):
        """
        Search the name of the font for an integer which will be used as the
        pixel height of the generated font
        """

        match = re.search("([0-9]+)", name)

        if match is None:
            if name != "FONT_FALLBACK" and name != "FONT_FALLBACK_INTERNAL":
                raise ValueError(f"Font {name}: no height found in name\n")

            return 14

        return int(match.group(0))
