# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import os
import os.path
from glob import glob

from pebble_sdk_platform import pebble_platforms

__author__ = "katharine"


def find_most_specific_filename(bld, env, root_node, general_filename):
    if "~" in general_filename:
        bld.fatal("Generic resource filenames cannot contain a tilde (~).")

    basename, extension = os.path.splitext(general_filename)

    # The filenames we get will have extra bits of folder at the start, so trim those.
    root_len = len(root_node.relpath()) + 1
    if root_node.relpath() == ".":
        root_len = 2

    glob_result = glob(
        f"{os.path.join(root_node.relpath(), basename)}*{extension}"
    )
    options = [x[root_len:] for x in glob_result if os.path.isfile(x)]

    specificities = {}
    try:
        valid_tags = set(pebble_platforms[env.PLATFORM_NAME]["TAGS"])
    except KeyError:
        bld.fatal(
            f"Unrecognized platform {env.PLATFORM_NAME}. Did you mean to configure with --internal_sdk_build?"
        )

    for option in options:
        # We can get names that don't match if the name we have is a prefix of other files. Drop those.
        if option.split("~", 1)[0] != basename:
            continue
        tags = set(os.path.splitext(option)[0].split("~")[1:])
        # If there exist tags that aren't valid, we skip this.
        if len(tags - valid_tags) > 0:
            continue
        # If it's valid, the specificity is the number of tags that exist in both sets.
        specificities[option] = len(valid_tags & tags)

    if len(specificities) == 0:
        return general_filename

    top_score = max(specificities.values())
    top_candidates = [k for k, v in specificities.items() if v == top_score]
    if len(top_candidates) > 1:
        bld.fatal(
            "The correct file for {general} on {platform} is ambiguous: {count} files have "
            "specificity {score}:\n\t{files}".format(
                general=general_filename,
                count=len(top_candidates),
                score=top_score,
                platform=env.PLATFORM_NAME,
                files="\n\t".join(top_candidates),
            )
        )

    return top_candidates[0]
