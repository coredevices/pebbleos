/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include "layout_layer.h"
#include "layout_node.h"

// TODO: PBL-28902 Timeline card layouts integration tests

#include "applib/graphics/gtypes.h"
#include "applib/graphics/gpath.h"
#include "applib/ui/status_bar_layer.h"
#include "applib/ui/kino/kino_layer.h"
#include "apps/system/timeline/text_node.h"
#include "pbl/services/clock.h"
#include "pbl/services/timeline/timeline.h"
#include "pbl/services/timeline/timeline_resources.h"

#define TIMELINE_MAX_BOX_HEIGHT 2500
#define TIMELINE_TOP_MARGIN 10

#define TIMELINE_CARD_ARROW_HEIGHT 13
#define TIMELINE_CARD_MARGIN PBL_IF_RECT_ELSE(7, 12)
#define TIMELINE_CARD_TRANSITION_MS (interpolate_moook_duration())
#define TIMELINE_CARD_BODY_HEADER_MARGIN_HEIGHT                    \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ -2,                                              \
      /* medium */ -2,                                             \
      /* large */ 2,                                               \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* extralarge */ 2                                           \
    )
#define TIMELINE_CARD_BODY_MARGIN_HEIGHT                           \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ 17,                                              \
      /* medium */ 17,                                             \
      /* large */ 15,                                              \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* extralarge */ 15                                          \
    )

typedef struct TimelineLayout TimelineLayout;

typedef struct TimelineLayoutImpl TimelineLayoutImpl;

typedef GTextNode *(*TimelineLayoutViewConstructor)(TimelineLayout *layout);

typedef void (*TimelineLayoutViewDeinitializer)(TimelineLayout *layout);

typedef enum {
  TimelineScrollDirectionUp, //!< Timeline Past
  TimelineScrollDirectionDown, //!< Timeline Future
} TimelineScrollDirection;

typedef struct {
  Uuid app_id;
  time_t timestamp;
  time_t current_day;
  time_t end_time;
  time_t pin_time;
  uint32_t duration_s;
  TimelineScrollDirection scroll_direction;
  bool all_day;
  //! Number of concurrent Timeline events not including the first event. If there is only one
  //! event, num_concurrent is 0. If there are two events overlapping, num_concurrent is 1.
  //! Only valid if the item is being used for the Timeline Peek.
  unsigned int num_concurrent;
} TimelineLayoutInfo;

struct TimelineLayout {
  LayoutLayer layout_layer;
  LayoutColors colors;

  KinoLayer icon_layer;
  GSize icon_size;
  AppResourceInfo icon_res_info;
  uint32_t icon_resource_id;
  int16_t page_break_height;
  bool has_page_break; //<! Used to enable special first scroll behavior

  const TimelineLayoutImpl *impl;
  TimelineLayoutInfo *info;

  TimelineResourceInfo icon_info; //!< Timeline id for icon

  //! Pin view node representing the visual layout
  GTextNode *view_node;
  GSize view_size;

  struct TimelineLayout *transition_layout; //!< The layout this is transitioning to
  Animation *transition_animation; //!< Transition animation for unscheduling

  KinoLayer **metric_icon_layers;
  unsigned int num_metric_icon_layers;

  bool is_being_destroyed; //!< Used to prevent animation stopped handlers to start more animations
};

struct TimelineLayoutImpl {
  struct {
    AttributeId primary_id;
    AttributeId secondary_id;
  } attributes;

  LayoutColors default_colors;

  TimelineResourceId default_icon;
  GAlign card_icon_align;
  TimelineResourceSize card_icon_size;

  TimelineLayoutViewConstructor card_view_constructor;
  TimelineLayoutViewDeinitializer card_view_deinitializer;
};

TimelineResourceId timeline_layout_get_icon_resource_id(
    LayoutLayerMode mode, const AttributeList *attributes, TimelineResourceSize icon_size,
    TimelineResourceId fallback_resource);

void timeline_layout_init(TimelineLayout *layout, const LayoutLayerConfig *config,
                          const TimelineLayoutImpl *timeline_layout_impl);

void timeline_layout_init_with_icon_id(TimelineLayout *layout, const LayoutLayerConfig *config,
                                       const TimelineLayoutImpl *timeline_layout_impl,
                                       TimelineResourceId icon_resource);

void timeline_layout_init_info(TimelineLayoutInfo *info, TimelineItem *item, time_t current_day);

void timeline_layout_deinit(TimelineLayout *timeline_layout);

void timeline_layout_get_icon_frame(const GRect *bounds, TimelineScrollDirection scroll_direction,
                                    GRect *frame);

////////////////////////
// Layout Impl
////////////////////////

GSize timeline_layout_get_content_size(GContext *ctx, LayoutLayer *layout);

void timeline_layout_destroy(LayoutLayer *layout);

void timeline_layout_change_mode(LayoutLayer *layout, LayoutLayerMode final_mode);

const LayoutColors *timeline_layout_get_colors(const LayoutLayer *layout);

////////////////////////
// View
////////////////////////

void timeline_layout_init_view(TimelineLayout *layout, LayoutLayerMode mode);

void timeline_layout_deinit_view(TimelineLayout *layout);

void timeline_layout_render_view(TimelineLayout *timeline_layout, GContext *ctx);

void timeline_layout_get_size(TimelineLayout *timeline_layout, GContext *ctx, GSize *size_out);

////////////////////////
// Card
////////////////////////

GTextNode *timeline_layout_create_card_view_from_config(const TimelineLayout *layout,
                                                        const LayoutNodeConfig *config);

GTextNodeCustom *timeline_layout_create_icon_node(const TimelineLayout *layout);

GTextNodeCustom *timeline_layout_create_page_break_node(const TimelineLayout *layout);

void timeline_layout_time_text_update(const LayoutLayer *layout,
                                      const LayoutNodeTextDynamicConfig *config,
                                      char *buffer, bool render);
