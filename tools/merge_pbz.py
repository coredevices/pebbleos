#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import tempfile
import zipfile


def merge_pbz_files(slot0_pbz, slot1_pbz, output_pbz):
    """Merge two .pbz files into a single .pbz file with slot0 and slot1 folders."""

    with tempfile.TemporaryDirectory() as temp_dir:
        slot0_dir = os.path.join(temp_dir, "slot0")
        slot1_dir = os.path.join(temp_dir, "slot1")

        os.makedirs(slot0_dir)
        os.makedirs(slot1_dir)

        # Extract slot0 pbz
        with zipfile.ZipFile(slot0_pbz, "r") as zip_ref:
            zip_ref.extractall(slot0_dir)

        # Extract slot1 pbz
        with zipfile.ZipFile(slot1_pbz, "r") as zip_ref:
            zip_ref.extractall(slot1_dir)

        # Create output pbz with both slots
        with zipfile.ZipFile(output_pbz, "w", zipfile.ZIP_DEFLATED) as zip_out:
            # Add slot0 files
            for root, _, files in os.walk(slot0_dir):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, temp_dir)
                    zip_out.write(file_path, arcname)

            # Add slot1 files
            for root, _, files in os.walk(slot1_dir):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, temp_dir)
                    zip_out.write(file_path, arcname)


def main():
    parser = argparse.ArgumentParser(
        description="Merge two .pbz files into one with slot0 and slot1 folders"
    )
    parser.add_argument("--slot0-pbz", required=True, help="Path to slot0 .pbz file")
    parser.add_argument("--slot1-pbz", required=True, help="Path to slot1 .pbz file")
    parser.add_argument("--output", required=True, help="Path to output .pbz file")

    args = parser.parse_args()

    merge_pbz_files(args.slot0_pbz, args.slot1_pbz, args.output)
    print(
        f"Successfully merged {args.slot0_pbz} and {args.slot1_pbz} into {args.output}"
    )


if __name__ == "__main__":
    main()
