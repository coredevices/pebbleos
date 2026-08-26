# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import copy

import resources.resource_map.resource_generator_bitmap
import resources.resource_map.resource_generator_font
import resources.resource_map.resource_generator_js
import resources.resource_map.resource_generator_pbi
import resources.resource_map.resource_generator_png
import resources.resource_map.resource_generator_raw  # noqa: F401
from resources.find_resource_filename import find_most_specific_filename
from resources.resource_map import resource_generator
from resources.types.resource_definition import ResourceDefinition
from resources.types.resource_object import ResourceObject
from sdk_helpers import is_sdk_2x, validate_resource_not_larger_than
from waflib import Node


def _preprocess_resource_ids(bld, resources_list, has_published_media=False):
    """
    This method reads all of the defined resources for the project and assigns resource IDs to
    them prior to the start of resource processing. This preprocessing step is necessary in order
    for the timeline lookup table to contain accurate resource IDs, while still allowing us the
    prepend the TLUT as a resource in the resource ball.
    :param bld: the BuildContext object
    :param resources_list: the list of resources defined for this project
    :param has_published_media: boolean for whether publishedMedia exists for the project
    :return: None
    """
    resource_id_mapping = {}

    next_id = 1
    if has_published_media:
        # The timeline lookup table must be the first resource if one exists
        resource_id_mapping["TIMELINE_LUT"] = next_id
        next_id += 1

    for res_id, res in enumerate(resources_list, start=next_id):
        if isinstance(res, Node.Node):
            if res.name == "timeline_resource_table.reso":
                continue
            res_name = ResourceObject.load(res.abspath()).definition.name
            resource_id_mapping[res_name] = res_id
        else:
            resource_id_mapping[res.name] = res_id

    bld.env.RESOURCE_ID_MAPPING = resource_id_mapping


def generate_resources(bld, resource_source_path):
    """
    This method creates all of the task generators necessary to handle every possible resource
    allowed by the SDK.
    :param bld: the BuildContext object
    :param resource_source_path: the path from which to retrieve resource files
    :return: N/A
    """
    resources_json = getattr(bld.env, "RESOURCES_JSON", [])
    published_media_json = getattr(bld.env, "PUBLISHED_MEDIA_JSON", [])
    if resource_source_path:
        resources_node = bld.path.find_node(resource_source_path)
    else:
        resources_node = bld.path.find_node("resources")

    resource_file_mapping = {}
    for resource in resources_json:
        resource_file_mapping[resource["name"]] = find_most_specific_filename(
            bld, bld.env, resources_node, resource["file"]
        )

    # Load the waftools that handle creating resource objects, a resource pack and the resource
    # ID header
    bld.load("generate_pbpack generate_resource_ball generate_resource_id_header")
    bld.load("process_timeline_resources")

    # Iterate over the resource definitions and do some processing to remove resources that
    # aren't relevant to the platform we're building for and to apply various backwards
    # compatibility adjustments
    resource_definitions = []
    max_menu_icon_dimensions = (25, 25)
    for r in resources_json:
        if r.get("menuIcon"):
            res_file = (
                resources_node.find_node(
                    find_most_specific_filename(
                        bld, bld.env, resources_node, str(r["file"])
                    )
                )
            ).abspath()
            if not validate_resource_not_larger_than(
                bld, res_file, dimensions=max_menu_icon_dimensions
            ):
                bld.fatal(
                    "menuIcon resource '{}' exceeds the maximum allowed dimensions of {}".format(
                        r["name"], max_menu_icon_dimensions
                    )
                )

        defs = resource_generator.definitions_from_dict(bld, r, resource_source_path)

        for d in defs:
            if not d.is_in_target_platform(bld):
                continue

            if d.type == "png-trans":
                # SDK hack for SDK compatibility
                # One entry in the media list with the type png-trans actually represents two
                # resources, one for the black mask and one for the white mask. They each have
                # their own resource ids, so we need two entries in our definitions list.
                for suffix in ("WHITE", "BLACK"):
                    new_definition = copy.deepcopy(d)
                    new_definition.name = "%s_%s" % (d.name, suffix)
                    resource_definitions.append(new_definition)

                continue

            if d.type == "png" and is_sdk_2x(
                bld.env.SDK_VERSION_MAJOR, bld.env.SDK_VERSION_MINOR
            ):
                # We don't have png support in the 2.x sdk, instead process these into a pbi
                d.type = "pbi"

            resource_definitions.append(d)

    # Auto-inject MOD resource for moddable projects
    if getattr(bld.env, "BUILD_TYPE", None) == "moddable":
        import os

        platform = bld.env.PLATFORM_NAME
        mod_path = f"build/mods/{platform}/mc.xsa"
        full_mod_path = os.path.join(bld.path.abspath(), mod_path)
        if os.path.exists(full_mod_path):
            mod_definition = ResourceDefinition(
                type="raw", name="MOD", file=mod_path, target_platforms=[platform]
            )
            resource_definitions.append(mod_definition)
            resource_file_mapping["MOD"] = mod_path
            print(
                f"Auto-injected MOD resource for {platform} from {mod_path}"
            )
        else:
            print(
                f"WARNING: MOD resource not found at {full_mod_path} for {platform}"
            )

    bld_dir = bld.path.get_bld().make_node(bld.env.BUILD_DIR)
    lib_resources = []
    for lib in bld.env.LIB_JSON:
        # Skip resource handling if not a Pebble library or if no resources are specified
        if "pebble" not in lib or "resources" not in lib["pebble"]:
            continue
        if (
            "media" not in lib["pebble"]["resources"]
            or not lib["pebble"]["resources"]["media"]
        ):
            continue

        lib_path = bld.path.find_node(lib["path"])

        try:
            resources_path = lib_path.find_node("resources").find_node(
                bld.env.PLATFORM_NAME
            )
        except AttributeError:
            bld.fatal("Library {} is missing resources".format(lib["name"]))
        else:
            if resources_path is None:
                bld.fatal(
                    "Library {} is missing resources for the {} platform".format(
                        lib["name"], bld.env.PLATFORM_NAME
                    )
                )

        for lib_resource in bld.env.LIB_RESOURCES_JSON.get(lib["name"], []):
            # Skip resources that specify targetPlatforms other than this one
            if "targetPlatforms" in lib_resource:
                if bld.env.PLATFORM_NAME not in lib_resource["targetPlatforms"]:
                    continue

            reso_file = "{}.{}.reso".format(lib_resource["file"], lib_resource["name"])
            resource_node = resources_path.find_node(reso_file)
            if resource_node is None:
                bld.fatal(
                    "Library {} is missing the {} resource for the {} platform".format(
                        lib["name"], lib_resource["name"], bld.env.PLATFORM_NAME
                    )
                )
            if lib_resource["name"] in resource_file_mapping:
                bld.fatal(
                    "Duplicate resource IDs are not permitted. Package resource {} uses the "
                    "same resource ID as another resource already in this project.".format(
                        lib_resource["name"]
                    )
                )
            resource_file_mapping[lib_resource["name"]] = resource_node
            lib_resources.append(resource_node)

    resources_list = []
    if resource_definitions:
        resources_list.extend(resource_definitions)
    if lib_resources:
        resources_list.extend(lib_resources)

    build_type = getattr(bld.env, "BUILD_TYPE", "app")
    resource_ball = bld_dir.make_node("system_resources.resball")

    # If this is a library, generate a resource ball containing only resources provided in this
    # project (not additional dependencies)
    project_resource_ball = None
    if build_type == "lib":
        project_resource_ball = bld_dir.make_node("project_resources.resball")
        bld.env.PROJECT_RESBALL = project_resource_ball

    if published_media_json:
        # Only create TLUT for non-packages
        if build_type != "lib":
            timeline_resource_table = bld_dir.make_node("timeline_resource_table.reso")
            resources_list.append(timeline_resource_table)
            _preprocess_resource_ids(bld, resources_list, True)

            bld(
                features="process_timeline_resources",
                published_media=published_media_json,
                timeline_reso=timeline_resource_table,
                layouts_json=bld_dir.make_node("layouts.json"),
                resource_mapping=resource_file_mapping,
                vars=["RESOURCE_ID_MAPPING", "PUBLISHED_MEDIA_JSON"],
            )

    # Create resource objects from a set of resource definitions and package them in a resource ball
    bld(
        features="generate_resource_ball",
        resources=resources_list,
        resource_ball=resource_ball,
        project_resource_ball=project_resource_ball,
        vars=["RESOURCES_JSON", "LIB_RESOURCES_JSON", "RESOURCE_ID_MAPPING"],
    )

    # Create a resource ID header for use during the linking step of the build
    # FIXME PBL-36458: Since pebble.h requires this file through a #include, this file must be
    # present for every project, regardless of whether or not resources exist for the project. At
    # this time, this means the `generate_resource_id_header` task generator must run for every
    # project. Since the input of the `generate_resource_id_header` task generator is the
    # resource ball created by the `generate_resource_ball` task generator, the
    # `generate_resource_ball` task generator must also run for every project.
    resource_id_header = bld_dir.make_node("src/resource_ids.auto.h")
    bld.env.RESOURCE_ID_HEADER = resource_id_header.abspath()
    bld(
        features="generate_resource_id_header",
        resource_ball=resource_ball,
        resource_id_header_target=resource_id_header,
        use_extern=build_type == "lib",
        use_define=build_type in ("app", "moddable"),
        published_media=published_media_json,
    )

    resource_id_definitions = bld_dir.make_node("src/resource_ids.auto.c")
    bld.env.RESOURCE_ID_DEFINITIONS = resource_id_definitions.abspath()
    bld(
        features="generate_resource_id_definitions",
        resource_ball=resource_ball,
        resource_id_definitions_target=resource_id_definitions,
        published_media=published_media_json,
    )

    if not bld.env.BUILD_TYPE or bld.env.BUILD_TYPE in ("app", "moddable"):
        # Create a resource pack for distribution with an application binary
        pbpack = bld_dir.make_node("app_resources.pbpack")
        bld(
            features="generate_pbpack",
            resource_ball=resource_ball,
            pbpack_target=pbpack,
            is_system=False,
        )
