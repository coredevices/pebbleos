# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import inject_metadata
from waflib.Configure import conf
from waflib.Errors import BuildError


def configure(conf):
    """
    This method is called from the configure method of the pebble_sdk waftool to setup the
    environment variables for compiling a 3rd party app
    :param conf: the ConfigContext
    :return: None
    """
    CROSS_COMPILE_PREFIX = "arm-none-eabi-"

    conf.env.AS = CROSS_COMPILE_PREFIX + "gcc"
    conf.env.AR = CROSS_COMPILE_PREFIX + "ar"
    conf.env.CC = CROSS_COMPILE_PREFIX + "gcc"
    conf.env.LD = CROSS_COMPILE_PREFIX + "ld"
    conf.env.SIZE = CROSS_COMPILE_PREFIX + "size"

    optimize_flag = "-Os"

    conf.load("gcc")

    pebble_cflags = [
        "-std=c99",
        "-mcpu=cortex-m3",
        "-mthumb",
        "-ffunction-sections",
        "-fdata-sections",
        "-fcommon",
        "-g",
        "-fPIE",
        optimize_flag,
    ]
    c_warnings = [
        "-Wall",
        "-Wextra",
        "-Werror",
        "-Wno-unused-parameter",
        "-Wno-error=unused-function",
        "-Wno-error=unused-variable",
        "-Wno-error=builtin-declaration-mismatch",
        "-Wno-error=format-truncation",
        "-Wno-error=expansion-to-defined",
        "-Wno-error=zero-length-bounds",
        "-Wno-error=cast-function-type",
        "-Wno-error=unused-value",
    ]

    if (conf.env.SDK_VERSION_MAJOR == 5) and (conf.env.SDK_VERSION_MINOR > 19):
        pebble_cflags.append("-D_TIME_H_")
        # Override time_t to be 32-bit for Pebble compatibility (newer toolchains default to 64-bit)
        pebble_cflags.append("-Dtime_t=long")
    pebble_cflags.extend(c_warnings)

    pebble_linkflags = [
        "-mcpu=cortex-m3",
        "-mthumb",
        "-Wl,--gc-sections",
        "-Wl,--warn-common",
        "-fPIE",
        optimize_flag,
    ]

    conf.env.prepend_value("CFLAGS", pebble_cflags)
    conf.env.prepend_value("LINKFLAGS", pebble_linkflags)
    conf.env.SHLIB_MARKER = None
    conf.env.STLIB_MARKER = None


@conf
def pbl_suppress_newer_gcc_warnings(conf):
    """Call from your wscript's configure() to suppress warnings that newer
    GCC versions (11+/14+) treat as errors. This is a workaround -- consider
    fixing the underlying issues in your code instead."""
    workaround_flags = [
        "-Wno-error=unused-but-set-variable",
        "-Wno-error=unused-const-variable",
        # Newer GCC got stricter about these common patterns
        "-Wno-error=format-overflow",
        "-Wno-error=stringop-truncation",
        "-Wno-error=stringop-overflow",
        "-Wno-error=array-bounds",
        "-Wno-error=restrict",
        # GCC 14 promoted these from warnings to errors
        "-Wno-error=implicit-function-declaration",
        "-Wno-error=implicit-int",
        "-Wno-error=int-conversion",
        "-Wno-error=incompatible-pointer-types",
        "-Wno-error=return-mismatch",
        # Common in embedded code with packed structs
        "-Wno-error=address-of-packed-member",
        "-Wno-error=packed-not-aligned",
        # Old code patterns
        "-Wno-error=maybe-uninitialized",
        "-Wno-error=misleading-indentation",
        "-Wno-error=implicit-fallthrough",
        "-Wno-error=enum-conversion",
        "-Wno-error=enum-int-mismatch",
        "-Wno-error=sign-compare",
        "-Wno-error=type-limits",
        "-Wno-error=missing-field-initializers",
        "-Wno-error=override-init",
        "-Wno-error=missing-braces",
        "-Wno-error=old-style-declaration",
        "-Wno-error=pointer-sign",
        "-Wno-error=dangling-pointer",
        "-Wno-error=nonnull",
        "-Wno-error=duplicate-decl-specifier",
        "-Wno-error=absolute-value",
        "-Wno-error=aggressive-loop-optimizations",
        "-Wno-error=address",
        "-Wno-error=sizeof-pointer-memaccess",
        "-Wno-error=zero-length-bounds",
        "-Wno-error=builtin-declaration-mismatch",
        "-Wno-error=expansion-to-defined",
    ]
    for platform in conf.env.TARGET_PLATFORMS:
        conf.all_envs[platform].append_value("CFLAGS", workaround_flags)
    # Also apply to the default env so that any task generators using a
    # non-platform env (or envs derived after this point) inherit the flags
    conf.env.append_value("CFLAGS", workaround_flags)


# -----------------------------------------------------------------------------------
def gen_inject_metadata_rule(
    bld,
    src_bin_file,
    dst_bin_file,
    elf_file,
    resource_file,
    timestamp,
    has_pkjs,
    has_worker,
    max_binary_size=None,
):
    """
    Copy from src_bin_file to dst_bin_file and inject the correct meta-data into the
    header of dst_bin_file
    :param bld: the BuildContext
    :param src_bin_file: the path to the pebble-app.raw.bin file
    :param dst_bin_file: the path to the pebble-app.bin
    :param elf_file: the path to the pebble-app.elf file
    :param resource_file: the path to the resource pack
    :param timestamp: the timestamp of the project build
    :param has_pkjs: boolean for whether the project contains code using PebbleKit JS
    :param has_worker: boolean for whether the project has a worker binary
    :param max_binary_size: the platform's maximum app image size, or None for the default
    """

    def inject_data_rule(task):
        bin_path = task.inputs[0].abspath()
        elf_path = task.inputs[1].abspath()
        if len(task.inputs) >= 3:
            res_path = task.inputs[2].abspath()
        else:
            res_path = None
        tgt_path = task.outputs[0].abspath()

        # First copy the raw bin that the compiler produced to a new location. This way we'll have
        # the raw binary around to inspect just in case anything went wrong while we were injecting
        # metadata.
        cp_result = task.exec_command(f'cp "{bin_path}" "{tgt_path}"')
        if cp_result < 0:
            raise BuildError(f"Failed to copy {bin_path} to {tgt_path}!")

        # Now actually inject the metadata into the new copy of the binary.
        inject_metadata.inject_metadata(
            tgt_path,
            elf_path,
            res_path,
            timestamp,
            allow_js=has_pkjs,
            has_worker=has_worker,
            max_binary_size=max_binary_size,
        )

    sources = [src_bin_file, elf_file]
    if resource_file is not None:
        sources.append(resource_file)
    bld(
        rule=inject_data_rule,
        name="inject-metadata",
        source=sources,
        target=dst_bin_file,
    )
