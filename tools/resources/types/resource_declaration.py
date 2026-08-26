# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


class ResourceDeclaration:
    """A resource that has a name and nothing else.

    Used for resources that are stored on the filesystem. We don't know anything about these
    resources other than that they exist at firmware compile time.
    """

    def __init__(self, name):
        self.name = name
