# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from resources.resource_map import resource_generator
from resources.resource_map.resource_generator_js import (
    JsResourceGenerator,  # noqa: F401
)
from waflib import Node, Task, TaskGen

from resources import generators


class reso(Task.Task):
    def run(self):
        reso = resource_generator.generate_object(self, self.definition)
        reso.dump(self.outputs[0])


class resource_ball(Task.Task):
    def run(self):
        generators.build_resource_ball(
            [r.abspath() for r in self.inputs],
            self.outputs[0],
            declarations=getattr(self, "resource_declarations", []),
            resource_id_mapping=getattr(self.env, "RESOURCE_ID_MAPPING", {}),
        )


def process_resource_definition(task_gen, resource_definition):
    """
    Create a task that generates a .reso and returns the node pointing to
    the output
    """

    sources = []
    for s in resource_definition.sources:
        source_node = task_gen.path.make_node(s)
        if source_node is None:
            task_gen.bld.fatal(
                f"Could not find resource at {task_gen.bld.path.find_node(s).abspath()}"
            )
        sources.append(source_node)

    output_name = "{}.{}.{}".format(
        sources[0].relpath(),
        str(resource_definition.name),
        "reso",
    )

    # Build our outputs in a directory relative to where our final pbpack is going to go
    output = task_gen.resource_ball.parent.make_node(output_name)

    task = task_gen.create_task("reso", sources, output)
    task.definition = resource_definition
    task.dep_nodes = getattr(task_gen, "resource_dependencies", [])

    # Save JS bytecode filename for dependency calculation for SDK memory report
    if resource_definition.type == "js" and "PEBBLE_SDK_ROOT" in task_gen.env:
        task_gen.bld.all_envs[task_gen.env.PLATFORM_NAME].JS_RESO = output

    return output


@TaskGen.feature("generate_resource_ball")
@TaskGen.before_method("process_source", "process_rule")
def process_resource_ball(task_gen):
    """
    resources: a list of ResourceDefinitions objects and nodes pointing to
               .reso files
    resource_dependencies: node list that all our generated resources depend on
    resource_ball: a node to where the ball should be generated
    vars: a list of environment variables that generated resources should depend on as a source
    """
    resource_objects = []
    bundled_resos = []

    for r in task_gen.resources:
        if isinstance(r, Node.Node):
            # It's already a node, presumably pointing to a .reso file
            resource_objects.append(r)
        else:
            # It's a resource definition, we need to process it into a .reso
            # file. Note this is where the task that does the data conversion
            # gets created.
            processed_resource = process_resource_definition(task_gen, r)
            resource_objects.append(processed_resource)
            bundled_resos.append(processed_resource)

    if getattr(task_gen, "project_resource_ball", None):
        prb_task = task_gen.create_task(
            "resource_ball", bundled_resos, task_gen.project_resource_ball
        )
        prb_task.dep_node = getattr(task_gen, "resource_dependencies", [])
        prb_task.dep_vars = getattr(task_gen, "vars", [])

    task = task_gen.create_task(
        "resource_ball", resource_objects, task_gen.resource_ball
    )
    task.resource_declarations = getattr(task_gen, "resource_declarations", [])
    task.dep_nodes = getattr(task_gen, "resource_dependencies", [])
    task.dep_vars = getattr(task_gen, "vars", [])
