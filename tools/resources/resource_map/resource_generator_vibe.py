# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from io import BytesIO

import json2vibe
from pebble_sdk_platform import pebble_platforms  # noqa: F401
from resources.resource_map.resource_generator import ResourceGenerator
from resources.types.resource_object import ResourceObject


class VibeResourceGenerator(ResourceGenerator):
    type = "vibe"

    @staticmethod
    def generate_object(task, definition):
        out = BytesIO()
        json2vibe.convert_to_file(task.inputs[0].abspath(), out)

        return ResourceObject(definition, out.getvalue())
