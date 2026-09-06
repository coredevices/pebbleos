/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include "gtypes.h"
#include "gpath.h"

//! @addtogroup Graphics
//! @{
//!   @addtogroup PathBuilding Building Paths
//! \brief Functions to build GPath objects without using GPathInfo
//!
//! Code example:
//! \code{.c}
//! #define MAX_POINTS 256
//! static GPath *s_path;
//!
//! // .update_proc of my_layer:
//! void my_layer_update_proc(Layer *my_layer, GContext* ctx) {
//!   // Fill the path:
//!   graphics_context_set_fill_color(ctx, GColorWhite);
//!   gpath_draw_filled(ctx, s_path);
//!   // Stroke the path:
//!   graphics_context_set_stroke_color(ctx, GColorBlack);
//!   gpath_draw_outline(ctx, s_path);
//! }
//!
//! void build_my_path(void) {
//!   // Specify how many points builder can create
//!   GPathBuilder *builder = gpath_builder_create(MAX_POINTS);
//!   // Use gpath builder API to create path
//!   gpath_builder_move_to_point(builder, GPoint(0, -60));
//!   gpath_builder_curve_to_point(builder, GPoint(60, 0), GPoint(35, -60), GPoint(60, -35));
//!   gpath_builder_curve_to_point(builder, GPoint(0, 60), GPoint(60, 35), GPoint(35, 60));
//!   gpath_builder_curve_to_point(builder, GPoint(0, 0), GPoint(-50, 60), GPoint(-50, 0));
//!   gpath_builder_curve_to_point(builder, GPoint(0, -60), GPoint(50, 0), GPoint(50, -60));
//!   // Convert the result to GPath object for drawing
//!   s_path = gpath_builder_create_path(builder);
//!   // Destroy the builder
//!   gpath_builder_destroy(builder);
//! }
//! \endcode
//!   @{

//! Data structure used by gpath builder
//! @note This structure is being filled by gpath builder
typedef struct {
  //! Maximum number of points that builder can create and size of `points` array
  uint32_t max_points;
  //! The number of points in `points` array
  uint32_t num_points;
  //! Array containing points
  GPoint points[];
} GPathBuilder;

//! Creates new GPathBuilder object on the heap sized accordingly to maximum number
//! of points given
//!
//! @param max_points Size of the points buffer
//! @return A pointer to GPathBuilder. NULL if object couldn't be created
GPathBuilder *gpath_builder_create(uint32_t max_points);

//! Destroys GPathBuilder previously created with gpath_builder_create()
void gpath_builder_destroy(GPathBuilder *builder);

//! Sets starting point for GPath
//! @param builder GPathBuilder object to manipulate on
//! @param to_point starting point for the GPath
//! @return True if point was moved successfully False if there was no space in
//! GPathBuilder struct or there was segment added already
bool gpath_builder_move_to_point(GPathBuilder *builder, GPoint to_point);

//! Makes straight line from current point to point given and makes it new current point
//! @param builder GPathBuilder object to manipulate on
//! @param to_point ending point for the line
//! @return True if line was added successfully False if there was no space in GPathBuilder struct
bool gpath_builder_line_to_point(GPathBuilder *builder, GPoint to_point);

//! Makes bezier curve from current point to point given and makes it new current point,
//! quadratic bezier curve is created based on control points
//! @param builder GPathBuilder object to manipulate on
//! @param to_point ending point for bezier curve
//! @param control_point_1 control point for start of the bezier curve
//! @param control_point_2 control point for end of the bezier curve
//! @return True if curve was added successfully False if there was no space in GPathBuilder struct
bool gpath_builder_curve_to_point(GPathBuilder *builder, GPoint to_point,
                                  GPoint control_point_1, GPoint control_point_2);

//! Creates a new GPath on the heap based on a data from GPathBuilder
//!
//! Values after initialization:
//! * `num_points` and `points` pointer: copied from the GPathBuilder
//! * `rotation`: 0
//! * `offset`: (0, 0)
//! @return A pointer to the GPath. `NULL` if num_points less than 2 or not enough memory
GPath *gpath_builder_create_path(GPathBuilder *builder);

//!   @} // end addtogroup PathBuilding
//! @} // end addtogroup Graphics
