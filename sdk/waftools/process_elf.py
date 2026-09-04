# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import objcopy
import pebble_sdk_gcc


# TODO: PBL-33841 Make this a feature
def generate_bin_file(task_gen, bin_type, elf_file, has_pkjs, has_worker):
    """
    Generate bin file by injecting metadata from elf file and resources file
    :param task_gen: the task generator instance
    :param bin_type: the type of binary being built (app, worker, lib)
    :param elf_file: the path to the compiled elf file
    :param has_pkjs: boolean for whether the build contains PebbleKit JS files
    :param has_worker: boolean for whether the build contains a worker binary
    :return: the modified binary file with injected metadata
    """
    platform_build_node = task_gen.bld.path.get_bld().find_node(
        task_gen.bld.env.BUILD_DIR
    )

    packaged_files = [elf_file]
    resources_file = None
    if bin_type != "worker":
        resources_file = platform_build_node.find_or_declare("app_resources.pbpack")
        packaged_files.append(resources_file)

    raw_bin_file = platform_build_node.make_node(f"pebble-{bin_type}.raw.bin")
    bin_file = platform_build_node.make_node(f"pebble-{bin_type}.bin")

    task_gen.bld(rule=objcopy.objcopy_bin, source=elf_file, target=raw_bin_file)
    pebble_sdk_gcc.gen_inject_metadata_rule(
        task_gen.bld,
        src_bin_file=raw_bin_file,
        dst_bin_file=bin_file,
        elf_file=elf_file,
        resource_file=resources_file,
        timestamp=task_gen.bld.env.TIMESTAMP,
        has_pkjs=has_pkjs,
        has_worker=has_worker,
        # Read now: bld.env is only this platform's ConfigSet during the caller's loop.
        max_binary_size=task_gen.bld.env.PLATFORM["MAX_APP_BINARY_SIZE"],
    )
    return bin_file
