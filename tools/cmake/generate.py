#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Source and header generators invoked from the CMake build."""

import argparse
import json
import os
import sys

import wafshim
from wafshim import REPO_ROOT

wafshim.setup_path()

import gitinfo


def write_if_changed(path, content):
    """Leave the file (and its timestamp) alone when nothing changed, so
    dependents are not rebuilt for a no-op regeneration."""
    if os.path.exists(path):
        with open(path) as f:
            if f.read() == content:
                return False
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.write(content)
    return True


def read_config(path):
    config = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            if value.startswith('"') and value.endswith('"'):
                value = value[1:-1]
            config[key] = value
    return config


# --- git_version.auto.h ----------------------------------------------------


def cmd_git_version(args):
    revision = gitinfo.get_git_revision()
    # Truncate the commit to fit in the versions struct. It may become
    # ambiguous, but that beats failing the build over a long hash.
    revision["COMMIT"] = revision["COMMIT"][:7]
    if len(revision["TAG"]) > 31:
        print(f"Git tag {revision['TAG']} is too long, truncating", file=sys.stderr)
        revision["TAG"] = revision["TAG"][:31]

    with open(args.template) as f:
        content = f.read()
    for key, value in revision.items():
        content = content.replace(f"@{key}@", str(value))
    write_if_changed(args.output, content)


# --- System app registry (src/fw/shell) ------------------------------------


def uuid_to_byte_hex_str(uuid):
    uuid = uuid.replace("-", "")
    pieces = [uuid[i : i + 2] for i in range(0, len(uuid), 2)]
    return "0x" + ", 0x".join(pieces)


def cmd_app_registry(args):
    config = read_config(args.config)
    with open(args.input) as f:
        definition = json.load(f)

    system_apps = definition["system_apps"]
    resource_apps = definition["resource_apps"]

    def entry_enabled(entry):
        defines = entry.get("ifdefs") or []
        platforms = entry.get("target_platforms") or []

        # Disabled overrides any define.
        if "DISABLED" in defines:
            return False
        if platforms and args.board not in platforms:
            return False
        # Kept around for legacy purposes.
        if "DEFAULT" in defines:
            return True
        if defines:
            return all(config.get(d) == "y" for d in defines)
        return True

    # Zeroed app ids indicate disabled apps.
    for entry in system_apps:
        if not entry_enabled(entry):
            entry["id"] = 0

    enabled_system_apps = [e for e in system_apps if e["id"] != 0]

    out = ["// @" + "generated -- DO NOT EDIT\n\n"]
    out.append('#include "system_app_ids.auto.h"\n')
    out.append('#include "process_management/pebble_process_md.h"\n')
    out.append('#include "resource/resource_ids.auto.h"\n\n')
    out.extend(
        f"extern const PebbleProcessMd *{entry['md_fn']}(void);\n"
        for entry in enabled_system_apps
    )
    out.append("\n\nstatic const AppRegistryEntry APP_RECORDS[] = {\n")
    out.append("\n  // System Apps\n")
    for entry in enabled_system_apps:
        out.append(
            "  {{\n"
            "    .id = APP_ID_{enum},\n"
            "    .type = AppInstallStorageFw,\n"
            "    .md_fn = &{cb_str},\n"
            "    .color.argb = {color_argb8},\n"
            "  }},\n".format(
                enum=entry["enum"],
                cb_str=entry["md_fn"],
                color_argb8=entry.get("color_argb8", "GColorClearARGB8"),
            )
        )
    out.append("\n  // Resource (stored) Apps\n")
    for entry in resource_apps:
        out.append(
            "  {{\n"
            "    .id = APP_ID_{enum},\n"
            "    .type = AppInstallStorageResources,\n"
            '    .name = "{name}",\n'
            "    .uuid = {{ {uuid} }},\n"
            "    .bin_resource_id = {bin_id},\n"
            "    .icon_resource_id = {icon_id},\n"
            "    .color.argb = {color_argb8},\n"
            "  }},\n".format(
                enum=entry["enum"],
                name=entry["name"],
                uuid=uuid_to_byte_hex_str(entry["uuid"]),
                bin_id=entry["bin_resource_id"],
                icon_id=entry["icon_resource_id"],
                color_argb8=entry.get("color_argb8", "GColorClearARGB8"),
            )
        )
    out.append("};\n")
    write_if_changed(args.registry, "".join(out))

    # Apps the sources assume exist always need a define.
    default_app_enums = ["ALARMS", "GOLF", "MUSIC", "SETTINGS", "SPORTS"]
    out = ["// @" + "generated -- DO NOT EDIT\n\n"]
    out.append('#include "process_management/app_install_types.h"\n\n')
    built = []
    for entry in system_apps + resource_apps:
        built.append(entry["enum"])
        out.append(
            "#define APP_ID_{enum} ((AppInstallId) {id})\n".format(
                enum=entry["enum"], id=entry["id"]
            )
        )
    out.extend(
        f"#define APP_ID_{enum} ((AppInstallId) 0)\n"
        for enum in default_app_enums
        if enum not in built
    )
    write_if_changed(args.enum, "".join(out))


# --- Pebble protocol endpoint table (src/fw/services/comm_session) ---------


def cmd_endpoints_table(args):
    with open(args.input) as f:
        definition = json.load(f)

    endpoints = list(definition["prf_and_normal_fw"])
    if not args.recovery:
        endpoints.extend(definition["normal_fw_only"])
    endpoints.sort()

    def get_access_enum(access_str):
        if access_str == "private":
            return "PebbleProtocolAccessPrivate"
        if access_str == "any":
            return "PebbleProtocolAccessAny"
        raise ValueError(f"Unknown value: {access_str}")

    DEFAULT_RECV_IMPL = "g_default_kernel_receiver_implementation"
    DEFAULT_RECV_OPT = "g_default_kernel_receiver_opt_bg"
    recv_imp_set = {DEFAULT_RECV_IMPL}
    recv_opt_set = {DEFAULT_RECV_OPT}

    out = ["// GENERATED -- DO NOT EDIT\n\n", '#include "kernel/pebble_tasks.h"\n\n']
    for _eid, _eid_str, _access, cb_str, recv_imp, recv_opt in endpoints:
        if recv_imp:
            recv_imp_set.add(recv_imp)
        if recv_opt:
            recv_opt_set.add(recv_opt)
        if cb_str:
            out.append(
                f"extern void {cb_str}(CommSession *session, "
                "const uint8_t* data, size_t length);\n"
            )

    out.append("\n\n")
    out.extend(f"extern ReceiverImplementation {r};\n" for r in recv_imp_set)
    out.append("\n\n")
    out.extend(f"extern const PebbleTask {r};\n" for r in recv_opt_set)
    out.append("\n\nstatic const PebbleProtocolEndpoint s_protocol_endpoints[] = {\n")

    for eid, eid_str, access_str, cb_str, recv_imp, recv_opt in endpoints:
        if int(eid_str, base=16) != eid:
            raise ValueError(f"Endpoint IDs need to match: {eid} vs {eid_str}")
        if not cb_str:
            cb_str = "NULL"
        if not recv_imp:
            recv_imp = DEFAULT_RECV_IMPL
            if not recv_opt:
                recv_opt = DEFAULT_RECV_OPT
        recv_opt = "&" + recv_opt if recv_opt else "NULL"
        out.append(
            f"  {{ {eid}, {cb_str}, {get_access_enum(access_str)}, "
            f"&{recv_imp}, {recv_opt} }},\n"
        )
    out.append("};\n\n")
    write_if_changed(args.output, "".join(out))


# --- applib_malloc ---------------------------------------------------------


def cmd_applib_malloc(args):
    import applib_malloc

    applib_malloc.generate_files(args.input, args.header, args.impl, args.min_sdk, False)


# --- Hashed log strings ----------------------------------------------------


def cmd_loghash(args):
    from log_hashing.check_elf_log_strings import check_dict_log_strings
    from log_hashing.newlogging import get_log_dict_from_file

    log_dict = get_log_dict_from_file(args.elf)
    if not log_dict:
        sys.exit(f"Unable to get log strings from {args.elf}")

    output = check_dict_log_strings(log_dict)
    if output:
        sys.exit(output + "\nNewLogging string formatting error")

    with open(args.output, "w") as f:
        json.dump(log_dict, f, indent=2, sort_keys=True)


def cmd_sdk_fonts_header(args):
    """The font keys apps may use, from the whitelist in
    exported_symbols.json. A platform frozen at an SDK revision only gets
    the fonts that existed by then."""
    from pebble_sdk_platform import pebble_platforms

    frozen_revision = pebble_platforms[args.platform].get("FROZEN_AT_REVISION")
    with open(args.input) as f:
        fonts = json.load(f)["fonts"]

    out = ["#pragma once\n", "\n"]
    for entry in fonts:
        if isinstance(entry, dict):
            name = entry["name"]
            added = entry.get("addedRevision")
            if frozen_revision is not None and added is not None and added > frozen_revision:
                continue
        else:
            name = entry
        out.append(f'#define FONT_KEY_{name} "RESOURCE_ID_{name}"\n')
    write_if_changed(args.output, "".join(out))


# --- Stored apps ----------------------------------------------------------


def cmd_appinfo(args):
    import generate_appinfo

    generate_appinfo.generate_appinfo(args.input, args.output)


def cmd_inject_metadata(args):
    """Copy the raw app binary and stamp the app metadata header into the
    copy, keeping the untouched original around for inspection."""
    import shutil

    sys.path.insert(0, os.path.join(REPO_ROOT, "sdk", "tools"))
    import inject_metadata

    shutil.copy(args.input, args.output)
    # A fixed timestamp: it only describes the resource version, and stored
    # apps have no resources. A real one would change the app's CRC on every
    # build, and with it the pbpack's.
    inject_metadata.inject_metadata(
        args.output, args.elf, None, 0, allow_js=False, has_worker=False
    )


def cmd_app_reso(args):
    from resources.types.resource_definition import ResourceDefinition
    from resources.types.resource_object import ResourceObject
    from wafshim import Node

    with open(args.input, "rb") as f:
        data = f.read()
    reso = ResourceObject(
        ResourceDefinition("raw", f"STORED_APP_{args.name.upper()}", None), data
    )
    reso.dump(Node(args.output))


def cmd_pblboot_header(args):
    """Prepend the PBLBOOT image header. The boot priority is derived from
    the git tag so the bootloader always picks the newest image."""
    import pblboot

    revision = gitinfo.get_git_revision()
    priority = pblboot.boot_priority(revision["TAG"], int(revision["TIMESTAMP"]))
    if args.input.endswith(".bin"):
        pblboot.insert_header_bin(args.input, args.output, args.offset, priority)
    else:
        pblboot.insert_header_hex(args.input, args.output, args.offset, priority)


def cmd_loghash_merge(args):
    from log_hashing.newlogging import merge_loghash_dict_json_files

    merge_loghash_dict_json_files(args.output, args.inputs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("git-version")
    p.add_argument("--template", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_git_version)

    p = sub.add_parser("app-registry")
    p.add_argument("--input", required=True)
    p.add_argument("--registry", required=True)
    p.add_argument("--enum", required=True)
    p.add_argument("--config", required=True)
    p.add_argument("--board", required=True)
    p.set_defaults(func=cmd_app_registry)

    p = sub.add_parser("endpoints-table")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--recovery", action="store_true")
    p.set_defaults(func=cmd_endpoints_table)

    p = sub.add_parser("applib-malloc")
    p.add_argument("--input", required=True)
    p.add_argument("--header", required=True)
    p.add_argument("--impl", required=True)
    p.add_argument("--min-sdk", type=int, required=True)
    p.set_defaults(func=cmd_applib_malloc)

    p = sub.add_parser("loghash")
    p.add_argument("--elf", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_loghash)

    p = sub.add_parser("sdk-fonts-header")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--platform", required=True)
    p.set_defaults(func=cmd_sdk_fonts_header)

    p = sub.add_parser("appinfo")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_appinfo)

    p = sub.add_parser("inject-metadata")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--elf", required=True)
    p.set_defaults(func=cmd_inject_metadata)

    p = sub.add_parser("app-reso")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--name", required=True)
    p.set_defaults(func=cmd_app_reso)

    p = sub.add_parser("pblboot-header")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--offset", type=int, required=True)
    p.set_defaults(func=cmd_pblboot_header)

    p = sub.add_parser("loghash-merge")
    p.add_argument("--output", required=True)
    p.add_argument("inputs", nargs="+")
    p.set_defaults(func=cmd_loghash_merge)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
