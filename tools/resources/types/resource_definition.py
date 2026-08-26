# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


from resources.types.resource_declaration import ResourceDeclaration


class StorageType:
    pbpack = 1
    builtin = 2
    pfs = 3


class ResourceDefinition(ResourceDeclaration):
    def __init__(
        self,
        type,
        name,
        file,
        storage=StorageType.pbpack,
        target_platforms=None,
        aliases=(),
    ):
        self.type = type
        self.name = name

        self.file = file

        self.storage = storage

        # A list of platforms this resource is valid for. None means all platforms. [] means none.
        self.target_platforms = target_platforms
        self.aliases = list(aliases)

        self.sources = [self.file]

    def is_in_target_platform(self, bld):
        if self.target_platforms is None:
            return True

        return bld.env.PLATFORM_NAME in self.target_platforms

    def find_specific_filename(self, task_gen):
        return find_most_specific_filename(
            task_gen.bld, task_gen.env, task_gen.bld.path, self.file
        )

    def __repr__(self):
        return "<ResourceDefinition %r>" % self.__dict__
