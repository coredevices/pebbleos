# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import json
import struct


def main(pack_path, manifest_path):
    with open(manifest_path, "r") as f:
        manifest = json.load(f)

    resource_names = []
    for r in manifest["media"]:
        if r["type"] == "png-trans":
            resource_names.append(r["defName"] + "_WHITE")
            resource_names.append(r["defName"] + "_BLACK")
        else:
            resource_names.append(r["defName"])

    with open(pack_path, "rb") as f:
        header = f.read(4116)

    def resource_generator(tbl, num):
        for i in xrange(0, num * 16, 16):
            yield struct.unpack("<IIII", tbl[i : i + 16])

    (num_resources, res_version) = struct.unpack("<I16s", header[:20])

    print(f"number of resources: {num_resources}")
    print(f"resource pack version: {res_version}")
    print("resource entries:")
    print()
    print("{:<32s}\t{:>8s}\t{:>8s}\t{:>8s}".format("name", "offset", "size", "crc"))
    print("{:<32s}\t{:>8s}\t{:>8s}\t{:>8s}".format("----", "------", "----", "---"))
    for x in resource_generator(header[20:], num_resources):
        (index, offset, size, crc) = x
        print(
            f"{resource_names[index]:<32s}\t{offset:>8d}\t{size:>8d}\t{crc:>08x}"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("resource_pack")
    parser.add_argument("resource_map")
    args = parser.parse_args()

    main(args.resource_pack, args.resource_map)
