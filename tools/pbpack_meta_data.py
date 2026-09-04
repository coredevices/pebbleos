#!/usr/bin/env python
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse
from pathlib import Path

from pbpack import ResourcePack


def cmd_manifest(args):
    pack = ResourcePack(args.is_system)
    for f in args.pack_file_list:
        pack.add_resource(Path(f).read_bytes())

    pack.finalize()
    with open(args.manifest_file, "wb") as manifest:
        manifest_bytes = pack.serialize_manifest(timestamp=args.timestamp)
        manifest.write(manifest_bytes)


def cmd_table(args):
    pack = ResourcePack(args.is_system)
    for f in args.pack_file_list:
        pack.add_resource(Path(f).read_bytes())

    pack.finalize()
    with open(args.table_file, "wb") as table_file:
        table_bytes = pack.serialize_table()
        table_file.write(table_bytes)


def cmd_content(args):
    pack = ResourcePack(args.is_system)
    for f in args.pack_file_list:
        pack.add_resource(Path(f).read_bytes())

    pack.finalize()
    with open(args.content_file, "wb") as content_file:
        content_bytes = pack.serialize_content()
        content_file.write(content_bytes)


def main():
    # process an individual file
    parser = argparse.ArgumentParser(
        description="Generate the meta datachunks of the pbpack"
    )

    parser.add_argument(
        "--system",
        dest="is_system",
        action="store_true",
        default=False,
        help="whether this is the system resources pbpack",
    )

    subparsers = parser.add_subparsers(help="commands", dest="which")

    manifest_parser = subparsers.add_parser("manifest", help="make the manifest file")
    manifest_parser.add_argument(
        "manifest_file", metavar="MANIFEST_FILE", help="File to write the manifest to"
    )
    manifest_parser.add_argument(
        "timestamp",
        metavar="TIMESTAMP",
        help="timestamp to label this pack with",
        type=int,
    )
    manifest_parser.add_argument(
        "pack_file_list",
        metavar="PACK_FILE_LIST",
        nargs="*",
        help="a list of <pack_file_path>s",
    )
    manifest_parser.set_defaults(func=cmd_manifest)

    table_parser = subparsers.add_parser("table", help="make the metadata table")
    table_parser.add_argument(
        "table_file", metavar="TABLE_FILE", help="file to write the table chunk to"
    )
    table_parser.add_argument(
        "pack_file_list",
        metavar="PACK_FILE_LIST",
        nargs="*",
        help="a list of <pack_file_path>s",
    )
    table_parser.set_defaults(func=cmd_table)

    content_parser = subparsers.add_parser("content", help="serialize the content")
    content_parser.add_argument(
        "content_file", metavar="CONTENT_FILE", help="file to write the content to"
    )
    content_parser.add_argument(
        "pack_file_list",
        metavar="PACK_FILE_LIST",
        nargs="*",
        help="a list of <pack_file_path>s",
    )
    content_parser.set_defaults(func=cmd_content)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
