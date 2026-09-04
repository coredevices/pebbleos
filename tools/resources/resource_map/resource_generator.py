# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os

from resources.find_resource_filename import find_most_specific_filename
from resources.types.resource_definition import ResourceDefinition, StorageType

# This is all the classes that have been registered by inheriting from the ResourceGenerator
# metaclass with a 'type' attribute set.
_ResourceGenerators = {}


class ResourceGeneratorMetaclass(type):
    type = None

    def __init__(cls, name, bases, dict):
        super().__init__(name, bases, dict)

        if cls.type:
            _ResourceGenerators[cls.type] = cls


# Instatiate the metaclass into a baseclass we can use elsewhere.
ResourceGeneratorBase = ResourceGeneratorMetaclass("ResourceGenerator", (object,), {})


class ResourceGenerator(ResourceGeneratorBase):
    @staticmethod
    def definitions_from_dict(bld, definition_dict, resource_source_path):
        """
        Default implementation of definitions_from_dict. Subclasses of ResourceGenerator can
        override this implementation if they'd like to customize this. Returns a list of definitions.
        """
        resource = {
            "name": definition_dict["name"],
            "filename": str(
                definition_dict.get("file", None)
            ),
        }
        resources = [resource]

        # Now generate ResourceDefintion objects for each resource
        target_platforms = definition_dict.get("targetPlatforms", None)
        aliases = definition_dict.get("aliases", [])
        builtin = (
            False if bld.variant == "applib" else definition_dict.get("builtin", False)
        )

        definitions = []
        for r in resources:
            if resource["filename"] is not None:
                filename_path = os.path.join(resource_source_path, r["filename"])
                # Handle paths that reference files outside the resources directory (e.g. ../third_party/...)
                if filename_path.startswith(".."):
                    filename_path = os.path.normpath(filename_path)
                else:
                    filename_path = find_most_specific_filename(
                        bld, bld.env, bld.path, filename_path
                    )
            else:
                filename_path = ""

            storage = StorageType.builtin if builtin else StorageType.pbpack

            d = ResourceDefinition(
                definition_dict["type"],
                r["name"],
                filename_path,
                storage=storage,
                target_platforms=target_platforms,
                aliases=aliases,
            )

            if "size" in r:
                d.size = r["size"]

            definitions.append(d)

        return definitions

    @classmethod
    def generate_object(cls, task, definition):
        """
        Stub implementation of generate_object. Subclasses must override this method.
        """
        raise NotImplementedError(f"{cls!r} missing a generate_object implementation")


def definitions_from_dict(bld, definition_dict, resource_source_path):
    cls = _ResourceGenerators[definition_dict["type"]]
    return cls.definitions_from_dict(bld, definition_dict, resource_source_path)


def generate_object(task, definition):
    cls = _ResourceGenerators[definition.type]
    return cls.generate_object(task, definition)
