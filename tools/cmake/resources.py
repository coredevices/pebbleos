#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Drive the resource build from CMake.

``manifest`` runs at configure time: it merges the resource maps for the
board and variant, and emits the CMake rules that turn every declared
resource into a .reso file. The remaining sub-commands are the build
steps those rules invoke.
"""

import argparse
import json
import os
import pickle

import wafshim
from wafshim import REPO_ROOT, Bld, Env, Node, Task

wafshim.setup_path()

from resources.resource_map import resource_generator
from resources.types.resource_declaration import ResourceDeclaration

RESOURCES_DIR = os.path.join(REPO_ROOT, "resources")

# How many resources one generator process builds. Small enough that the
# batches still fill the cores, large enough that start-up is not the bill.
BATCH_SIZE = 16

# Which module registers the generator for each resource type. Building one
# resource only imports the module it needs: pulling in freetype, the PDC
# tooling and libpebble2 for every one of the several hundred resources
# would cost more than generating them.
GENERATOR_MODULES = {
    "font": "resource_generator_font",
    "js": "resource_generator_js",
    "mo": "resource_generator_mo",
    "pbi": "resource_generator_pbi",
    "pbi8": "resource_generator_pbi",
    "png-trans": "resource_generator_pbi",
    "pdc": "resource_generator_pdc",
    "png": "resource_generator_png",
    "raw": "resource_generator_raw",
    "vibe": "resource_generator_vibe",
}


def load_generators(*types):
    """Register the generators for the given resource types (all of them
    when none is named)."""
    import importlib

    names = set(GENERATOR_MODULES.values()) if not types else {
        GENERATOR_MODULES[t] for t in types
    }
    for name in sorted(names):
        importlib.import_module(f"resources.resource_map.{name}")


def import_generators():
    from resources import generators

    return generators


def make_bld(args):
    env = Env()
    env.PLATFORM_NAME = args.platform
    env.BOARD_NAME = args.board_name
    env.VARIANT = args.variant
    env.NODE = getattr(args, "node", None) or "node"
    return Bld(env, RESOURCES_DIR, args.builddir)


# --- Resource map merging (was resources/wscript_build) --------------------


def _load_map(path):
    with open(path) as f:
        return json.load(f)


def get_resources_dict(bld, variant):
    """Merge common/base with the variant and platform overrides."""
    resource_nodes = []
    override_dicts = []

    common_node = bld.path.find_node("common/base/resource_map.json")
    resources_dict = _load_map(common_node.abspath())
    resource_nodes.append(common_node)

    is_recovery = variant == "prf"
    root_path = "prf/" if is_recovery else "normal/"
    specific_node = bld.path.find_node(root_path + "base/resource_map.json")
    specific_dict = _load_map(specific_node.abspath())
    resource_nodes.append(specific_node)
    override_dicts.append(specific_dict)

    # Add and override resources based on the platform
    for path in ("common/" + bld.env.BOARD_NAME + "/resource_map.json",
                 root_path + bld.env.BOARD_NAME + "/resource_map.json"):
        platform_node = bld.path.find_node(path)
        if platform_node:
            resource_nodes.append(platform_node)
            override_dicts.append(_load_map(platform_node.abspath()))

    def update_common_media_item(item_dict):
        for common_media_item in resources_dict["media"]:
            if common_media_item["name"] == item_dict["name"]:
                common_media_item.update(item_dict)
                return
        resources_dict["media"].append(item_dict)

    for override in override_dicts:
        for item in override["media"]:
            update_common_media_item(item)

    # "files" and "timeline" cannot exist in the common resource map.
    for key in ("files", "timeline"):
        if key in specific_dict:
            resources_dict[key] = specific_dict[key]

    # The font scripts want an absolute characterList path.
    for item in resources_dict["media"]:
        if "characterList" in item:
            item["characterList"] = os.path.abspath(
                os.path.join("resources", item["characterList"])
            )

    if is_recovery:
        # PRF needs every media item built into the firmware image.
        for item in resources_dict["media"]:
            if item["type"] == "font":
                item["extended"] = False
            item["builtin"] = True

    return resource_nodes, resources_dict


def definition_deps(definition):
    """Everything a resource is built from."""
    deps = [os.path.join(RESOURCES_DIR, s) for s in definition.sources]
    character_list = getattr(definition, "character_list", None)
    # Some resource maps name a codepoint list that does not exist; the font
    # generator only reads one for the formats that need it.
    if character_list and os.path.exists(character_list):
        deps.append(character_list)
    return deps


def reso_output(definition, builddir):
    """Where a resource's .reso lands: next to the source it is built
    from, under the build directory."""
    source = Node(os.path.join(RESOURCES_DIR, definition.sources[0]))
    return os.path.join(builddir, f"{source.relpath()}.{definition.name}.reso")


def cmd_manifest(args):
    load_generators()
    bld = make_bld(args)
    resource_nodes, resources_dict = get_resources_dict(bld, args.variant)

    definitions = []
    for item in resources_dict["media"]:
        definitions.extend(resource_generator.definitions_from_dict(bld, item, ""))

    declarations = []
    for entry in resources_dict.get("files", []):
        declarations.extend(ResourceDeclaration(r) for r in entry["resources"])

    manifest = {
        "definitions": definitions,
        "declarations": declarations,
        "files": resources_dict.get("files", []),
        "timeline": resources_dict.get("timeline", []),
        "platform": args.platform,
        "board_name": args.board_name,
        "variant": args.variant,
        "dynamic": args.dynamic,
        "builddir": args.builddir,
    }
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "wb") as f:
        pickle.dump(manifest, f)

    map_files = [n.abspath() for n in resource_nodes]
    reso_files = []
    lines = [
        "# Generated by tools/cmake/resources.py -- do not edit.\n",
        "set(PBL_RESOURCE_MAP_FILES\n",
    ]
    lines.extend(f'  "{path}"\n' for path in map_files)
    lines.append(")\n\n")

    # One process per resource would be almost all interpreter start-up, so
    # they are generated in batches. Batching by type keeps each process to
    # the one generator module it needs, and a batch small enough that the
    # several hundred resources still spread across the cores.
    by_type = {}
    for index, definition in enumerate(definitions):
        by_type.setdefault(definition.type, []).append(index)

    batches = 0
    for resource_type, indices in sorted(by_type.items()):
        for start in range(0, len(indices), BATCH_SIZE):
            batch = indices[start : start + BATCH_SIZE]
            outputs = [reso_output(definitions[i], args.builddir) for i in batch]
            reso_files.extend(outputs)
            deps = sorted({d for i in batch for d in definition_deps(definitions[i])})
            outputs_str = "\n    ".join(f'"{o}"' for o in outputs)
            deps_str = "\n    ".join(f'"{d}"' for d in deps)
            index_str = " ".join(str(i) for i in batch)
            lines.append(
                f"add_custom_command(\n"
                f"  OUTPUT\n    {outputs_str}\n"
                f"  COMMAND ${{PYTHON_EXECUTABLE}} ${{PBL_RESOURCES_PY}} reso\n"
                f'          --manifest "{args.output}" --index {index_str}\n'
                f"  DEPENDS\n    {deps_str}\n"
                f'    "{args.output}"\n'
                f"  WORKING_DIRECTORY ${{PBL_BASE}}\n"
                f'  COMMENT "Generating {len(batch)} {resource_type} resources"\n'
                f"  VERBATIM\n"
                f")\n"
            )
            batches += 1

    lines.append("set(PBL_RESO_FILES\n")
    lines.extend(f'  "{path}"\n' for path in reso_files)
    lines.append(")\n")

    with open(args.cmake_output, "w") as f:
        f.writelines(lines)

    print(
        f"{len(definitions)} resources in {batches} batches, "
        f"{len(declarations)} declarations"
    )


def load_manifest(path):
    wafshim.setup_path()
    with open(path, "rb") as f:
        return pickle.load(f)


def bld_from_manifest(manifest):
    args = argparse.Namespace(
        platform=manifest["platform"],
        board_name=manifest["board_name"],
        variant=manifest["variant"],
        builddir=manifest["builddir"],
    )
    return make_bld(args)


def ordered_resos(manifest):
    """Every .reso that goes into the ball: the declared resources first,
    then the ones other build steps generate (stored apps, timezones)."""
    return [
        reso_output(d, manifest["builddir"]) for d in manifest["definitions"]
    ] + list(manifest["dynamic"])


# --- Build steps -----------------------------------------------------------


def cmd_reso(args):
    manifest = load_manifest(args.manifest)
    bld = bld_from_manifest(manifest)
    definitions = [manifest["definitions"][i] for i in args.index]
    load_generators(*{d.type for d in definitions})
    for definition in definitions:
        inputs = [Node(os.path.join(RESOURCES_DIR, s)) for s in definition.sources]
        output = Node(reso_output(definition, manifest["builddir"]))
        output.parent.mkdir()
        task = Task(bld, inputs, [output])
        resource_generator.generate_object(task, definition).dump(output)


def cmd_ball(args):
    manifest = load_manifest(args.manifest)
    generators = import_generators()
    generators.build_resource_ball(
        ordered_resos(manifest), Node(args.output), manifest["declarations"]
    )


def cmd_pbpack(args):
    generators = import_generators()
    generators.build_pbpack(args.ball, args.output, is_system=args.system)


def cmd_builtin(args):
    generators = import_generators()
    generators.build_builtin(args.ball, args.output, args.include)


def cmd_resource_ids(args):
    generators = import_generators()
    generators.build_resource_id_header(args.ball, args.output)


def cmd_font_header(args):
    generators = import_generators()
    generators.build_font_header(args.ball, args.output)


def cmd_font_table(args):
    generators = import_generators()
    generators.build_font_table(args.ball, args.output)


def cmd_timeline_table(args):
    manifest = load_manifest(args.manifest)
    generators = import_generators()
    generators.build_timeline_table(manifest["timeline"], args.output)


def cmd_timeline_ids(args):
    manifest = load_manifest(args.manifest)
    generators = import_generators()
    generators.build_timeline_ids(manifest["timeline"], args.output)


def cmd_layouts(args):
    manifest = load_manifest(args.manifest)
    uris = {
        "system://images/" + r["name"]: r["id"]
        for r in manifest["timeline"]
        if not r.get("internal", False)
    }
    with open(args.template) as f:
        content = f.read()
    content = content.replace("@RESOURCE_URIS@", json.dumps(uris, indent=4))
    with open(args.output, "w") as f:
        f.write(content)


def cmd_pfs(args):
    generators = import_generators()
    manifest = load_manifest(args.manifest)
    generators.build_pfs_resource_table(manifest["files"], args.output, args.include)


def cmd_version_header(args):
    generators = import_generators()
    generators.build_version_header(args.output, args.pbpack)


def _tzdata(olson):
    from io import BytesIO

    import tools.timezones

    zoneinfo_list = tools.timezones.build_zoneinfo_list(olson)
    dstrule_list = tools.timezones.dstrules_parse(olson)
    zonelink_list = tools.timezones.zonelink_parse(olson)

    data_file = BytesIO()
    tools.timezones.zoneinfo_to_bin(
        zoneinfo_list, dstrule_list, zonelink_list, data_file
    )
    return data_file.getvalue()


def cmd_tzdata(args):
    from resources.types.resource_definition import ResourceDefinition
    from resources.types.resource_object import ResourceObject

    reso = ResourceObject(
        ResourceDefinition("raw", "TIMEZONE_DATABASE", None), _tzdata(args.input)
    )
    reso.dump(Node(args.output))


def cmd_tzdata_header(args):
    """The same database, as a C array the timezone tests link against."""
    import generate_c_byte_array

    with open(args.output, "w") as f:
        generate_c_byte_array.write(f, _tzdata(args.input), "s_timezone_database")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("manifest")
    p.add_argument("--builddir", required=True)
    p.add_argument("--platform", required=True)
    p.add_argument("--board-name", required=True)
    p.add_argument("--variant", required=True)
    p.add_argument("--dynamic", nargs="*", default=[])
    p.add_argument("--output", required=True)
    p.add_argument("--cmake-output", required=True)
    p.set_defaults(func=cmd_manifest)

    p = sub.add_parser("reso")
    p.add_argument("--manifest", required=True)
    p.add_argument("--index", type=int, nargs="+", required=True)
    p.set_defaults(func=cmd_reso)

    p = sub.add_parser("ball")
    p.add_argument("--manifest", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_ball)

    p = sub.add_parser("pbpack")
    p.add_argument("--ball", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--system", action="store_true")
    p.set_defaults(func=cmd_pbpack)

    p = sub.add_parser("builtin")
    p.add_argument("--ball", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--include", required=True)
    p.set_defaults(func=cmd_builtin)

    p = sub.add_parser("resource-ids")
    p.add_argument("--ball", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_resource_ids)

    p = sub.add_parser("font-header")
    p.add_argument("--ball", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_font_header)

    p = sub.add_parser("font-table")
    p.add_argument("--ball", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_font_table)

    p = sub.add_parser("timeline-table")
    p.add_argument("--manifest", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_timeline_table)

    p = sub.add_parser("timeline-ids")
    p.add_argument("--manifest", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_timeline_ids)

    p = sub.add_parser("layouts")
    p.add_argument("--manifest", required=True)
    p.add_argument("--template", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_layouts)

    p = sub.add_parser("pfs")
    p.add_argument("--manifest", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--include", required=True)
    p.set_defaults(func=cmd_pfs)

    p = sub.add_parser("version-header")
    p.add_argument("--output", required=True)
    p.add_argument("--pbpack")
    p.set_defaults(func=cmd_version_header)

    p = sub.add_parser("tzdata")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_tzdata)

    p = sub.add_parser("tzdata-header")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_tzdata_header)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
