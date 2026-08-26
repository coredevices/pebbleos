# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import json
import struct

from resources.types.resource_definition import ResourceDefinition
from resources.types.resource_object import ResourceObject
from sdk_helpers import validate_resource_not_larger_than
from waflib import Node, Task
from waflib.TaskGen import before_method, feature


class layouts_json(Task.Task):
    """
    Task class for generating a layouts JSON file with the timeline/glance resource id mapping for
    publishedMedia items
    """

    def run(self):
        """
        This method executes when the layouts JSON task runs
        :return: N/A
        """
        published_media_dict = {m["id"]: m["name"] for m in self.published_media}
        timeline_entries = [
            {"id": media_id, "name": media_name}
            for media_id, media_name in published_media_dict.items()
        ]
        image_uris = {
            "resources": {
                "app://images/" + r["name"]: r["id"] for r in timeline_entries
            }
        }

        # Write a dictionary (created from map output) to a json file in the build directory
        with open(self.outputs[0].abspath(), "w") as f:
            json.dump(image_uris, f, indent=8)


def _collect_lib_published_media(ctx):
    """
    Collects all lib-defined publishedMedia objects and provides a list for comparison with app
    aliases
    :param ctx: the current Context object
    :return: a list of all defined publishedMedia items from included packages
    """
    published_media = []
    for lib in ctx.env.LIB_JSON:
        if "pebble" not in lib or "resources" not in lib["pebble"]:
            continue
        if "publishedMedia" not in lib["pebble"]["resources"]:
            continue
        published_media.extend(lib["pebble"]["resources"]["publishedMedia"])
    return published_media


class timeline_reso(Task.Task):
    """
    Task class for generating a timeline lookup table for publishedMedia items, which is then
    packed and packaged as a ResourceObject for later inclusion in a ResourceBall and PBPack
    """

    def run(self):
        """
        This method executes when the timeline reso task runs
        :return: N/A
        """
        bld = self.generator.bld
        resource_id_mapping = self.env.RESOURCE_ID_MAPPING

        TIMELINE_RESOURCE_TABLE_ENTRY_FMT = "<III"
        TLUT_SIGNATURE = b"TLUT"

        timeline_resources = []
        published_media_from_libs = _collect_lib_published_media(self.generator)

        # Create a sparse table to represent a c-style array
        for item in self.published_media:
            timeline_id = item.get("id", None)
            published_media_name = item.get(
                "name", None
            )  # string representation of published_id
            build_type = self.env.BUILD_TYPE

            timeline_tiny_exists = "timeline" in item and "tiny" in item["timeline"]
            if "glance" in item:
                # Alias ['timeline']['tiny'] to ['glance'] if missing, or validate
                # ['timeline']['tiny'] == ['glance'] if both exist
                if not timeline_tiny_exists:
                    timeline = item.pop("timeline", {})
                    timeline.update({"tiny": item["glance"]})
                    item["timeline"] = timeline
                elif item["glance"] != item["timeline"]["tiny"]:
                    bld.fatal(
                        "Resource {} in publishedMedia specifies different values {} and {}"
                        "for ['glance'] and ['timeline']['tiny'] attributes, respectively. "
                        "Differing values for these fields are not supported.".format(
                            item["name"], item["glance"], item["timeline"]["tiny"]
                        )
                    )
            else:
                if not timeline_tiny_exists:
                    if "alias" in item and build_type != "lib":
                        # Substitute package-defined publishedMedia item for objects with `alias`
                        # defined
                        for definition in published_media_from_libs:
                            if definition["name"] == item["alias"]:
                                del item["alias"]
                                del definition["name"]
                                item.update(definition)
                                break
                        else:
                            bld.fatal(
                                "No resource for alias '{}' exists in installed packages".format(
                                    item["alias"]
                                )
                            )
                    else:
                        bld.fatal(
                            f"Resource {published_media_name} in publishedMedia is missing values for ['glance'] "
                            "and ['timeline']['tiny']."
                        )

            # Extend table if needed
            if timeline_id >= len(timeline_resources):
                timeline_resources.extend(
                    {"tiny": 0, "small": 0, "large": 0}
                    for x in range(len(timeline_resources), timeline_id + 1)
                )

            # Set the resource IDs for this timeline item
            for size, res_id in item["timeline"].items():
                if res_id not in resource_id_mapping:
                    bld.fatal(
                        f"Invalid resource ID {res_id} specified in publishedMedia"
                    )
                timeline_resources[timeline_id][size] = resource_id_mapping[res_id]

        # Serialize the table
        table = TLUT_SIGNATURE
        for r in timeline_resources:
            table += struct.pack(
                TIMELINE_RESOURCE_TABLE_ENTRY_FMT, r["tiny"], r["small"], r["large"]
            )

        r = ResourceObject(ResourceDefinition("raw", "TIMELINE_LUT", ""), table)
        r.dump(self.outputs[0])


def _get_resource_file(ctx, mapping, resource_id, resources_node=None):
    try:
        resource = mapping[resource_id]
    except KeyError:
        ctx.bld.fatal(
            f"No resource '{resource_id}' found for publishedMedia use."
        )
    if isinstance(resource, Node.Node):
        return resource.abspath()
    elif resources_node:
        return resources_node.find_node(str(resource)).abspath()
    else:
        return ctx.path.find_node("resources").find_node(str(resource)).abspath()


@feature("process_timeline_resources")
@before_method("generate_resource_ball")
def process_timeline_resources(task_gen):
    """
    Process all of the resources listed in the publishedMedia object in project JSON files.
    As applicable, generate a layouts.json file for mobile apps to do resource id lookups,
    and generate a timeline lookup table for FW to do resource id lookups.

    Keyword arguments:
    published_media -- A JSON object containing all of the resources defined as publishedMedia in a
                       project's JSON file
    timeline_resource_table -- The name of the file to be used to store the timeline lookup table
    layouts_json -- The name of the file to be used to store the JSON timeline/glance resource id
                    mapping

    :param task_gen: the task generator instance
    :return: N/A
    """
    bld = task_gen.bld
    build_type = task_gen.env.BUILD_TYPE
    published_media = task_gen.published_media
    timeline_resource_table = task_gen.timeline_reso
    layouts_json = task_gen.layouts_json
    mapping = task_gen.resource_mapping

    MAX_SIZES = {
        "glance": (25, 25),
        "tiny": (25, 25),
        "small": (50, 50),
        "large": (80, 80),
    }

    used_ids = []
    for item in published_media:
        if "id" not in item:
            # Pebble Package builds omit the ID
            if build_type == "lib":
                continue
            else:
                bld.fatal(
                    "Missing 'id' attribute for publishedMedia item '{}'".format(
                        item["name"]
                    )
                )

        # Check for duplicate IDs
        if item["id"] in used_ids:
            task_gen.bld.fatal(
                "Cannot specify multiple resources with the same publishedMedia ID. "
                "Please modify your publishedMedia items to only use the ID {} once".format(
                    item["id"]
                )
            )
        else:
            used_ids.append(item["id"])

        # Check for valid resource dimensions
        if "glance" in item:
            res_file = _get_resource_file(task_gen, mapping, item["glance"])
            if not validate_resource_not_larger_than(
                task_gen.bld, res_file, MAX_SIZES["glance"]
            ):
                bld.fatal(
                    "publishedMedia item '{}' specifies a resource '{}' for attribute "
                    "'glance' that exceeds the maximum allowed dimensions of {} x {} for "
                    "that attribute.".format(
                        item["name"],
                        mapping[item["glance"]],
                        MAX_SIZES["glance"][0],
                        MAX_SIZES["glance"][1],
                    )
                )
        if "timeline" in item:
            for size in ("tiny", "small", "large"):
                if size in item["timeline"]:
                    res_file = _get_resource_file(
                        task_gen, mapping, item["timeline"][size]
                    )
                    if not validate_resource_not_larger_than(
                        task_gen.bld, res_file, MAX_SIZES[size]
                    ):
                        bld.fatal(
                            "publishedMedia item '{}' specifies a resource '{}' for size '{}'"
                            " that exceeds the maximum allowed dimensions of {} x {} for "
                            " that size.".format(
                                item["name"],
                                mapping[item["timeline"][size]],
                                size,
                                MAX_SIZES[size][0],
                                MAX_SIZES[size][1],
                            )
                        )

    timeline_reso_task = task_gen.create_task(
        "timeline_reso", src=None, tgt=timeline_resource_table
    )
    timeline_reso_task.published_media = published_media

    layouts_json_task = task_gen.create_task("layouts_json", src=None, tgt=layouts_json)
    layouts_json_task.published_media = published_media
