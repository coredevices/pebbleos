# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import logging


class TaggedAdapter(logging.LoggerAdapter):
    """Annotates all log messages with a "[tag]" prefix.

    The value of the tag is specified in the dict argument passed into
    the adapter's constructor.

    >>> logger = logging.getLogger(__name__)
    >>> adapter = TaggedAdapter(logger, {'tag': 'tag value'})
    """

    def process(self, msg, kwargs):
        return "[%s] %s" % (self.extra["tag"], msg), kwargs
