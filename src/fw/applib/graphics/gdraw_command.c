/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/applib_resource_private.h"
#include "gdraw_command.h"
#include "gdraw_command_private.h"

#include "applib/graphics/gpath.h"
#include "system/passert.h"
#include "syscall/syscall.h"
#include "util/net.h"

bool gdraw_command_resource_is_valid(ResAppNum app_num, uint32_t resource_id,
                                     uint32_t expected_signature, uint32_t *data_size) {
  // Load file signature, and check that it matches the expected_signature
  uint32_t data_signature;
  if (!(sys_resource_load_range(app_num, resource_id, 0, (uint8_t*)&data_signature,
        sizeof(data_signature)) == sizeof(data_signature) &&
        (ntohl(data_signature) == expected_signature))) {
    return NULL;
  }

  // Data is the second entry after the resource signature
  if (data_size) {
    uint32_t output_data_size;
    _Static_assert(PDCI_SIZE_OFFSET == PDCS_SIZE_OFFSET,
                   "code re-use between PDCI/PDCS requires same file format header");

    if (sys_resource_load_range(app_num, resource_id, sizeof(expected_signature),
          (uint8_t*)&output_data_size, sizeof(output_data_size)) != sizeof(output_data_size)) {
      return NULL;
    }
    *data_size = output_data_size;
  }
  return true;
}

bool gdraw_command_validate(GDrawCommand *command, size_t size) {
  if ((size < gdraw_command_get_data_size(command))) {
    return false;
  }

  return (((command->type == GDrawCommandTypeCircle) && (command->num_points == 1)) ||
          ((command->type == GDrawCommandTypePath) && (command->num_points > 1)) ||
          ((command->type == GDrawCommandTypePrecisePath) && (command->num_points > 1)));
}

static void prv_draw_path(GContext *ctx, GDrawCommand *command) {
  if (command->num_points <= 1) {
    return;
  }
  GPath path = {
    .num_points = command->num_points,
    .points = command->points
  };
  // draw all values of alpha, except fully transparent
  if ((command->fill_color.a != 0)) {
    graphics_context_set_fill_color(ctx, command->fill_color);
    gpath_draw_filled(ctx, &path);
  }
  if ((command->stroke_color.a != 0) && (command->stroke_width > 0)) {
    graphics_context_set_stroke_color(ctx, command->stroke_color);
    graphics_context_set_stroke_width(ctx, command->stroke_width);
    gpath_draw_stroke(ctx, &path, command->path_open);
  }
}

static void prv_draw_circle(GContext *ctx, GDrawCommand *command) {
  // draw all values of alpha, except fully transparent
  if ((command->fill_color.a != 0) && (command->radius > 0)) {
    graphics_context_set_fill_color(ctx, command->fill_color);
    graphics_fill_circle(ctx, command->points[0], command->radius);
  }
  if ((command->stroke_color.a != 0) && (command->stroke_width > 0)) {
    graphics_context_set_stroke_color(ctx, command->stroke_color);
    graphics_context_set_stroke_width(ctx, command->stroke_width);
    graphics_draw_circle(ctx, command->points[0], command->radius);
  }
}

static void prv_draw_precise_path(GContext *ctx, GDrawCommand *command) {
  if (command->num_points <= 1) {
    return;
  }

  // draw all values of alpha, except fully transparent
  if ((command->fill_color.a != 0)) {
    graphics_context_set_fill_color(ctx, command->fill_color);
    gpath_fill_precise_internal(ctx, command->precise_points, command->num_precise_points);
  }
  if ((command->stroke_color.a != 0) && (command->stroke_width > 0)) {
    graphics_context_set_stroke_color(ctx, command->stroke_color);
    graphics_context_set_stroke_width(ctx, command->stroke_width);
    gpath_draw_outline_precise_internal(ctx, command->precise_points,
                                        command->num_precise_points, command->path_open);
  }
}

void gdraw_command_draw(GContext *ctx, GDrawCommand *command) {
  if (!command || command->hidden) {
    return;
  }

  switch (command->type) {
    case GDrawCommandTypePath:
      prv_draw_path(ctx, command);
      break;
    case GDrawCommandTypePrecisePath:
      prv_draw_precise_path(ctx, command);
      break;
    case GDrawCommandTypeCircle:
      prv_draw_circle(ctx, command);
      break;
    default:
      WTF;
  }
}

size_t gdraw_command_get_data_size(GDrawCommand *command) {
  if (!command) {
    return 0;
  }
  return (sizeof(GDrawCommand) + (command->num_points * sizeof(GPoint)));
}

GDrawCommandType gdraw_command_get_type(GDrawCommand *command) {
  if (!command) {
    return GDrawCommandTypeInvalid;
  }
  return command->type;
}

void gdraw_command_set_fill_color(GDrawCommand *command, GColor fill_color) {
  if (!command) {
    return;
  }
  command->fill_color = fill_color;
}

GColor gdraw_command_get_fill_color(GDrawCommand *command) {
  if (!command) {
    return (GColor) {0};
  } else {
    return command->fill_color;
  }
}

void gdraw_command_set_stroke_color(GDrawCommand *command, GColor stroke_color) {
  if (!command) {
    return;
  } else  {
    command->stroke_color = stroke_color;
  }
}

GColor gdraw_command_get_stroke_color(GDrawCommand *command) {
  if (!command) {
    return (GColor) {0};
  } else {
    return command->stroke_color;
  }
}

void gdraw_command_set_stroke_width(GDrawCommand *command, uint8_t stroke_width) {
  if (!command) {
    return;
  }
  command->stroke_width = stroke_width;
}

uint8_t gdraw_command_get_stroke_width(GDrawCommand *command) {
  if (!command) {
    return 0;
  }
  return command->stroke_width;
}

uint16_t gdraw_command_get_num_points(GDrawCommand *command) {
  if (!command) {
    return 0;
  }
  return command->num_points;
}

void gdraw_command_set_point(GDrawCommand *command, uint16_t point_idx, GPoint point) {
  if (!command || (point_idx >= command->num_points)) {
    return;
  }
  command->points[point_idx] = point;
}

GPoint gdraw_command_get_point(GDrawCommand *command, uint16_t point_idx) {
  if (!command || (point_idx >= command->num_points)) {
    return GPointZero;
  } else {
    return command->points[point_idx];
  }
}

void gdraw_command_set_radius(GDrawCommand *command, uint16_t radius) {
  if (!command || (command->type != GDrawCommandTypeCircle)) {
    return;
  }
  command->radius = radius;
}

uint16_t gdraw_command_get_radius(GDrawCommand *command) {
  if (!command || (command->type != GDrawCommandTypeCircle)) {
    return 0;
  }
  return command->radius;
}

void gdraw_command_set_path_open(GDrawCommand *command, bool path_open) {
  if (!command ||
      ((command->type != GDrawCommandTypePath) &&
       (command->type != GDrawCommandTypePrecisePath))) {
    return;
  }
  command->path_open = path_open;
}

bool gdraw_command_get_path_open(GDrawCommand *command) {
  if (!command ||
      ((command->type != GDrawCommandTypePath) &&
       (command->type != GDrawCommandTypePrecisePath))) {
    return false;
  }
  return command->path_open;
}

void gdraw_command_set_hidden(GDrawCommand *command, bool hidden) {
  if (!command) {
    return;
  }
  command->hidden = hidden;
}

bool gdraw_command_get_hidden(GDrawCommand *command) {
  if (!command) {
    return false;
  }
  return command->hidden;
}

size_t gdraw_command_copy_points(GDrawCommand *command, GPoint *points, const size_t max_bytes) {
  const size_t actual_size = gdraw_command_get_num_points(command) * sizeof(GPoint);
  const size_t size = MIN(max_bytes, actual_size);
  memcpy(points, command->points, size);
  return size;
}
