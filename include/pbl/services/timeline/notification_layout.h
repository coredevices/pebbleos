/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "item.h"
#include "layout_layer.h"
#include "timeline_layout.h"

#include "applib/ui/kino/kino_layer.h"
#include "pbl/services/timeline/attribute.h"
#include "pbl/services/timeline/timeline_resources.h"

//! Banner height of notification and reminder layouts (excluding status bar)
//! Rectangular banner is the same size on both the top and bottom
#define LAYOUT_BANNER_HEIGHT_RECT                                  \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ 36,                                              \
      /* medium */ 36,                                             \
      /* large */ 29,                                              \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* extralarge */ 29                                          \
    )
//! Round banner is different between the top and bottom
#define LAYOUT_TOP_BANNER_HEIGHT_ROUND 60
#define LAYOUT_BOTTOM_BANNER_HEIGHT_ROUND 40
#define LAYOUT_TOP_BANNER_HEIGHT \
    PBL_IF_RECT_ELSE(LAYOUT_BANNER_HEIGHT_RECT, \
                     LAYOUT_TOP_BANNER_HEIGHT_ROUND - STATUS_BAR_LAYER_HEIGHT)
//! Height of the white arrow layer
#define LAYOUT_ARROW_HEIGHT PBL_IF_RECT_ELSE(19, 16)
//! Display height of the layout, which removes the arrow_height from the text layout region
//! PBL-23103 Remove hard-coded layout_height used for S4 paging notification and swap_layer
#define LAYOUT_HEIGHT (DISP_ROWS - STATUS_BAR_LAYER_HEIGHT - LAYOUT_ARROW_HEIGHT)
//! The y-offset before the text begins
#define LAYOUT_BEFORE_TEXT_SPACING_Y 40
//! Radius of the (round) banner
#define BANNER_CIRCLE_RADIUS 140
//! The starting y-position of the top banner
#define LAYOUT_TOP_BANNER_ORIGIN_Y (LAYOUT_TOP_BANNER_HEIGHT_ROUND - \
                                    (BANNER_CIRCLE_RADIUS + STATUS_BAR_LAYER_HEIGHT) - 1)

#define TEXT_VISIBLE_UPPER_THRESHOLD (STATUS_BAR_LAYER_HEIGHT - INTERPOLATE_MOOOK_BOUNCE_BACK - 1)
#define TEXT_VISIBLE_LOWER_THRESHOLD(h) \
    (-(h) + DISP_ROWS - STATUS_BAR_LAYER_HEIGHT - INTERPOLATE_MOOOK_BOUNCE_BACK - 1)

//! Offset and margin refer to the GTextNode definition where offset is the internal position delta
//! to be applied not affecting neighboring elements and margin is the size delta to be applied not
//! affecting internal positioning of the owning element. Padding is the amount of additional
//! spacing needed to be added between the owning element and the lower subsequent element. Upper
//! padding is the same except between the owning element and the upper previous element. Newer
//! positional or sizing fields should generally be either offset or margin instead of padding.
//! @see \ref GTextNode
typedef struct {
  const char *header_font_key;
  const char *title_font_key;
  const char *subtitle_font_key;
  const char *body_font_key;
  const char *footer_font_key;
  int8_t header_padding;
  int8_t title_offset_if_body_icon; //!< Conditional title delta offset if body icon exists
  int8_t title_padding;
  int8_t title_line_delta;
  int8_t subtitle_upper_padding;
  int8_t subtitle_lower_padding;
  int8_t subtitle_line_delta;
  int8_t location_offset;
  int8_t location_margin;
  int8_t body_padding;
  int8_t body_line_delta;
  int8_t body_icon_offset; //!< Body icon refers to a large body icon, currently used by Jumboji
  int8_t body_icon_margin;
  int8_t timestamp_upper_padding;
  int8_t timestamp_lower_padding;
} NotificationStyle;

typedef struct {
  TimelineItem *item;
  bool show_notification_timestamp;
} NotificationLayoutInfo;

typedef struct {
  LayoutLayer layout;
  KinoLayer icon_layer;
  AppResourceInfo icon_res_info;
  LayoutColors colors;
  NotificationLayoutInfo info;
  KinoLayer *detail_icon_layer; //!< Not common, so not inline with the layout
  const NotificationStyle *style;
  GTextNode *view_node;
  GSize view_size;
} NotificationLayout;

//! Default notification color
#define DEFAULT_NOTIFICATION_COLOR (GColorFolly)
#define DEFAULT_REMINDER_COLOR (GColorRed)
//! Generic notification icon
static const TimelineResourceId NOTIF_FALLBACK_ICON = TIMELINE_RESOURCE_NOTIFICATION_GENERIC;
static const TimelineResourceId REMINDER_FALLBACK_ICON = TIMELINE_RESOURCE_NOTIFICATION_REMINDER;
//! Height of tiny resource icons used in the top banner of notifications.
#define NOTIFICATION_TINY_RESOURCE_HEIGHT (ATTRIBUTE_ICON_TINY_SIZE_PX)
//! Used because some notification icons are 30 px wide.
#define NOTIFICATION_TINY_RESOURCE_SIZE (GSize(30, NOTIFICATION_TINY_RESOURCE_HEIGHT))
//! Adjusts the vertical position of the tiny resource icon on notifications to account for the
//! whitespace inside the status bar but below the status bar text. Note that this depends on the
//! font used in the status bar.
#define NOTIFICATION_TINY_RESOURCE_VERTICAL_OFFSET                 \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ -1,                                              \
      /* medium */ -1,                                             \
      /* large */ -2,                                              \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* extralarge */ -2                                          \
    )
//! Used to know where the icon is within the layout.
#define CARD_ICON_UPPER_PADDING                                            \
    ((LAYOUT_TOP_BANNER_HEIGHT - NOTIFICATION_TINY_RESOURCE_HEIGHT) / 2) + \
      NOTIFICATION_TINY_RESOURCE_VERTICAL_OFFSET

//! Bounds on the aspect a notification image band can reserve; the phone clamps to the same range.
#define NOTIFICATION_IMAGE_MIN_ASPECT (4)   //!< 4:1 landscape
#define NOTIFICATION_IMAGE_MAX_ASPECT (24)  //!< 2:3 portrait

LayoutLayer *notification_layout_create(const LayoutLayerConfig *config);

bool notification_layout_verify(bool existing_attributes[]);

//! Size of the image band this layout reserves, or false if the item has no image. The requester
//! and the renderer must agree on it, so it has one home.
bool notification_layout_get_image_size(const LayoutLayer *layout, GSize *size_out);

TimelineResourceId notification_layout_get_fallback_icon_id(TimelineItemType item_type);
