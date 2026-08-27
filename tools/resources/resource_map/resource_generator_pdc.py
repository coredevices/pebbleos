# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os

from resources.resource_map.resource_generator import ResourceGenerator
from resources.types.resource_object import ResourceObject

from tools.generate_pdcs import pdc_gen


class ResourceGeneratorPdc(ResourceGenerator):
    type = "pdc"

    @staticmethod
    def definitions_from_dict(bld, definition_dict, resource_source_path):
        definitions = ResourceGenerator.definitions_from_dict(
            bld, definition_dict, resource_source_path
        )

        source_root = bld.path.make_node(".")

        for d in definitions:
            node = bld.path.find_node(d.file)

            # PDCS (animated vectors) are described as a folder path. Adjust the sources so if any
            # frame in the animation changes we regenerate the pdcs
            if os.path.isdir(node.abspath()):
                source_nodes = bld.path.find_node(d.file).ant_glob("*.svg")
                d.sources = [n.path_from(source_root) for n in source_nodes]

        return definitions

    @staticmethod
    def generate_object(task, definition):
        node = task.generator.path.make_node(definition.file)

        if os.path.isdir(node.abspath()):
            output, _errors = pdc_gen.create_pdc_data_from_path(
                node.abspath(),
                viewbox_size=(0, 0),
                verbose=False,
                duration=33,
                play_count=1,
                precise=False,
            )
        else:
            output, _errors = pdc_gen.create_pdc_data_from_path(
                task.inputs[0].abspath(),
                viewbox_size=(0, 0),
                verbose=False,
                duration=0,
                play_count=0,
                precise=True,
            )

        return ResourceObject(definition, output)
