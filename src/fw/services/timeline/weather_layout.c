/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/weather_layout.h"
#include "pbl/services/timeline/timeline_layout.h"

#include "applib/graphics/gtypes.h"
#include "applib/preferred_content_size.h"
#include "applib/ui/ui.h"
#include "apps/system/timeline/text_node.h"
#include "font_resource_keys.auto.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/clock.h"
#include <pbl/logging/logging.h>
#include "pbl/util/size.h"
#include "pbl/util/string.h"

#define WEATHER_CARD_TITLE_LENGTH 30 // We're limited to one line for this layout

//////////////////////////////////////////
//  Card Mode
//////////////////////////////////////////

#define CARD_MARGIN_TOP                                            \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ PBL_IF_RECT_ELSE(3, 8),                          \
      /* medium */ PBL_IF_RECT_ELSE(3, 8),                         \
      /* large */ 12,                                              \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* extralarge */ 12                                          \
    )

#define IF_ICON_AT_TOP_ELSE(at_top, if_not)                        \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ PBL_IF_RECT_ELSE(if_not, at_top),                \
      /* medium */ PBL_IF_RECT_ELSE(if_not, at_top),               \
      /* large */ at_top,                                          \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* extralarge */ at_top                                      \
    )

#define CARD_MARGIN_BOTTOM PBL_IF_RECT_ELSE(7, 0)

static bool prv_should_display_time(const LayoutLayer *layout) {
  const int display_time = attribute_get_uint8(layout->attributes, AttributeIdDisplayTime,
                                               WeatherTimeType_Pin);
  return (display_time == WeatherTimeType_Pin);
}

// Append the pin time to the pin title to generate the card title
static void prv_title_update(const LayoutLayer *layout_ref,
                             const LayoutNodeTextDynamicConfig *config, char *buffer, bool render) {
  TimelineLayout *layout = (TimelineLayout *)layout_ref;
  const char *attr_text = attribute_get_string(layout_ref->attributes, AttributeIdTitle, "");
  strncpy(buffer, attr_text, config->buffer_size);
  buffer[config->buffer_size - 1] = '\0';
  if (!prv_should_display_time(layout_ref)) {
    return;
  }
  size_t pos = strnlen(attr_text, ATTRIBUTE_TITLE_MAX_LEN);
  if (pos + 1 /* one space */ < config->buffer_size) {
    buffer[pos++] = ' ';
    char time_buffer[TIME_STRING_REQUIRED_LENGTH];
    clock_copy_time_string_timestamp(time_buffer, sizeof(time_buffer), layout->info->timestamp);
    strncpy(buffer + pos, time_buffer, config->buffer_size - pos);
    buffer[config->buffer_size - 1] = '\0';
  }
}

static void prv_body_header_update(const LayoutLayer *layout_ref,
                                   const LayoutNodeTextDynamicConfig *config, char *buffer,
                                   bool render) {
  TimelineLayout *layout = (TimelineLayout *)layout_ref;
  if (prv_should_display_time(layout_ref)) {
    clock_get_friendly_date(buffer, config->buffer_size, layout->info->timestamp);
  }
}

static GTextNode *prv_card_view_constructor(TimelineLayout *timeline_layout) {
  static const LayoutNodeTextDynamicConfig s_title_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_title_update,
    .buffer_size = WEATHER_CARD_TITLE_LENGTH,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
    .text.fixed_lines = 1, // title fixed lines
    .text.alignment = LayoutTextAlignment_Center,
    .text.extent.margin.h = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      //! @note this is the same as Medium until Small is designed
      /* small */ PBL_IF_RECT_ELSE(2, 0),
      /* medium */ PBL_IF_RECT_ELSE(2, 0),
      /* large */ 1,
      //! @note this is the same as Large until ExtraLarge is designed
      /* extralarge */ 1), // title margin height
  };
  static const LayoutNodeTextAttributeConfig s_subtitle_config = {
    .attr_id = AttributeIdSubtitle,
    .text.font_key = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      //! @note this is the same as Medium until Small is designed
      /* small */ FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM,
      /* medium */ FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM,
      /* large */ FONT_KEY_LECO_36_BOLD_NUMBERS,
      //! @note this is the same as Large until ExtraLarge is designed
      /* extralarge */ FONT_KEY_LECO_36_BOLD_NUMBERS),
    .text.fixed_lines = 1, // subtitle fixed lines
    .text.alignment = LayoutTextAlignment_Center,
    .text.extent.margin.h = IF_ICON_AT_TOP_ELSE(1, 9), // subtitle margin height
  };
  static const LayoutNodeExtentConfig s_icon_config = {
    .node.type = LayoutNodeType_TimelineIcon,
    .margin.h = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      //! @note this is the same as Medium until Small is designed
      /* small */ 3,
      /* medium */ 3,
      /* large */ 0,
      //! @note this is the same as Large until ExtraLarge is designed
      /* extralarge */ 0), // icon margin height
  };
  static const LayoutNodeTextAttributeConfig s_glance_location_config = {
    .attr_id = AttributeIdLocationName,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
    .text.fixed_lines = 1, // glance location fixed lines
    .text.alignment = LayoutTextAlignment_Center,
  };
  static const LayoutNodeConfig s_page_break_config = {
    .type = LayoutNodeType_TimelinePageBreak,
  };
  static const LayoutNodeTextAttributeConfig s_location_config = {
    .attr_id = AttributeIdLocationName,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
    .text.line_spacing_delta = 2, // location line spacing delta
    .text.extent.margin.h = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      //! @note this is the same as Medium until Small is designed
      /* small */ 13,
      /* medium */ 13,
      /* large */ 11,
      //! @note this is the same as Large until ExtraLarge is designed
      /* extralarge */ 11), // location margin height
  };
  static const LayoutNodeTextDynamicConfig s_body_header_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_body_header_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_ParagraphHeader,
    .text.extent.margin.h = TIMELINE_CARD_BODY_HEADER_MARGIN_HEIGHT, // body header margin height
  };
  static const LayoutNodeTextAttributeConfig s_body_config = {
    .attr_id = AttributeIdBody,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Body,
    .text.line_spacing_delta = -2, // body line spacing delta
    .text.extent.margin.h = TIMELINE_CARD_BODY_MARGIN_HEIGHT, // body margin height
  };
  static const LayoutNodeConfig * const s_vertical_config_nodes[] = {
    IF_ICON_AT_TOP_ELSE(&s_icon_config.node, &s_title_config.text.extent.node),
    IF_ICON_AT_TOP_ELSE(&s_title_config.text.extent.node, &s_subtitle_config.text.extent.node),
    IF_ICON_AT_TOP_ELSE(&s_subtitle_config.text.extent.node, &s_icon_config.node),
    &s_glance_location_config.text.extent.node,
    &s_page_break_config,
    &s_location_config.text.extent.node,
    &s_body_header_config.text.extent.node,
    &s_body_config.text.extent.node,
  };
  static const LayoutNodeVerticalConfig s_vertical_config = {
    .container.extent.node.type = LayoutNodeType_Vertical,
    .container.num_nodes = ARRAY_LENGTH(s_vertical_config_nodes),
    .container.nodes = (LayoutNodeConfig **)&s_vertical_config_nodes,
    .container.extent.offset.y = CARD_MARGIN_TOP,
    .container.extent.margin.h = CARD_MARGIN_TOP + CARD_MARGIN_BOTTOM,
  };

  return timeline_layout_create_card_view_from_config(timeline_layout,
                                                      &s_vertical_config.container.extent.node);
}

//////////////////////////////////////////
// LayoutLayer API
//////////////////////////////////////////

bool weather_layout_verify(bool existing_attributes[]) {
  return (existing_attributes[AttributeIdTitle] && existing_attributes[AttributeIdLocationName]);
}

LayoutLayer *weather_layout_create(const LayoutLayerConfig *config) {
  WeatherLayout *layout = task_zalloc_check(sizeof(WeatherLayout));

  static const TimelineLayoutImpl s_timeline_layout_impl = {
    .attributes = { AttributeIdTitle, AttributeIdSubtitle },
    .default_colors = { { .argb = GColorBlackARGB8 },
                        { .argb = GColorClearARGB8 },
                        { .argb = GColorLightGrayARGB8 } },
    .default_icon = TIMELINE_RESOURCE_TIMELINE_WEATHER,
    .card_icon_align = GAlignCenter,
    .card_icon_size = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      //! @note this is the same as Medium until Small is designed
      /* small */ TimelineResourceSizeSmall,
      /* medium */ TimelineResourceSizeSmall,
      /* large */ TimelineResourceSizeLarge,
      //! @note this is the same as Large until ExtraLarge is designed
      /* extralarge */ TimelineResourceSizeLarge),
    .card_view_constructor = prv_card_view_constructor,
  };

  timeline_layout_init((TimelineLayout *)layout, config, &s_timeline_layout_impl);

  return (LayoutLayer *)layout;
}
