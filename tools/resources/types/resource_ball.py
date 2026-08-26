# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import pickle


class ResourceBall:
    """A object meant to be serialized to the filesystem that represents the complete set of
    resources for a firmware variant. Resources that are distributed with the firmware are present
    as ResourceObject instances where resources that are not (such as language packs) are present
    as ResourceDeclaration instances. This data structure is ordered, with the resource_objects
    conceptually coming first followed by the resource_declarations.
    """

    def __init__(self, resource_objects, resource_declarations):
        self.resource_objects = resource_objects
        self.resource_declarations = resource_declarations

    def get_all_declarations(self):
        return [
            o.definition for o in self.resource_objects
        ] + self.resource_declarations

    def dump(self, output_node):
        output_node.parent.mkdir()
        with open(output_node.abspath(), "wb") as f:
            pickle.dump(self, f, pickle.HIGHEST_PROTOCOL)

    @classmethod
    def load(cls, path):
        with open(path, "rb") as f:
            return pickle.load(f)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("resball")

    args = parser.parse_args()

    rb = ResourceBall.load(args.resball)

    for i, o in enumerate(rb.resource_objects, start=1):
        print(
            "%4u: %-50s %-10s %6u"
            % (i, o.definition.name, o.definition.type, len(o.data))
        )
