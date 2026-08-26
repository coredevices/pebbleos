# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import os

from pbpack import ResourcePack


def main():
    parser = argparse.ArgumentParser(
        description="Unpack pbpacked data to recover original file content."
    )
    parser.add_argument("pbpack", type=str, help="app_resources.pbpack file to unpack")
    args = parser.parse_args()

    if os.path.exists(args.pbpack):
        resource_pack = ResourcePack().deserialize(open(args.pbpack, "rb"))
        for idx, resource_data in enumerate(resource_pack.contents):
            with open(str(idx) + ".dat", "wb") as outfile:
                outfile.write(resource_data)


if __name__ == "__main__":
    main()
