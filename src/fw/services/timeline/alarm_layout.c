/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/alarm_layout.h"
#include "pbl/services/timeline/timeline_layout.h"

#include "applib/graphics/gtypes.h"
#include "applib/graphics/text.h"
#include "applib/preferred_content_size.h"
#include "font_resource_keys.auto.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/kernel_ui.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/alarms/alarm.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"

//////////////////////////////////////////
//  Card Mode
//////////////////////////////////////////

#define CARD_MARGIN_TOP PBL_IF_RECT_ELSE(3, 10)

static void prv_until_time_update(const LayoutLayer *layout_ref,
                                  const LayoutNodeTextDynamicConfig *config, char *buffer,
                                  bool render) {
  const TimelineLayout *layout = (TimelineLayout *)layout_ref;
  const int max_relative_hours = 24; // show up to "in 24 hours"
  clock_get_until_time_without_fulltime(buffer, config->buffer_size, layout->info->timestamp,
                                        max_relative_hours);
}

T_STATIC void prv_get_subtitle_from_attributes(AttributeList *attributes, char *buffer,
                                               size_t buffer_size, const void *i18n_owner) {
  const char *subtitle_string = NULL;
  // We only all-caps the subtitle in the card view on rectangular displays
  bool all_caps_desired = PBL_IF_RECT_ELSE(true, false);
  bool need_to_all_caps_string = false;

  // First, try to extract an AlarmKind from the pin to request a string with the desired
  // capitalization
  Attribute *alarm_kind_attribute = attribute_find(attributes, AttributeIdAlarmKind);
  if (alarm_kind_attribute) {
    const AlarmKind alarm_kind = (AlarmKind)alarm_kind_attribute->uint8;
    subtitle_string = i18n_get(alarm_get_string_for_kind(alarm_kind, all_caps_desired), i18n_owner);
  } else {
    // Otherwise, fallback to just using the subtitle in the pin
    subtitle_string = attribute_get_string(attributes, AttributeIdSubtitle, "");
    need_to_all_caps_string = all_caps_desired;
  }

  strncpy(buffer, subtitle_string, buffer_size);
  buffer[buffer_size - 1] = '\0';

  if (need_to_all_caps_string) {
    toupper_str(buffer);
  }
}

static void prv_subtitle_update(const LayoutLayer *layout,
                                const LayoutNodeTextDynamicConfig *config, char *buffer,
                                bool render) {
  prv_get_subtitle_from_attributes(layout->attributes, buffer, config->buffer_size, layout);
}

static void prv_time_digits_update(const LayoutLayer *layout_ref,
                                   const LayoutNodeTextDynamicConfig *config, char *buffer,
                                   bool render) {
  const TimelineLayout *layout = (TimelineLayout *)layout_ref;
  clock_get_time_number(buffer, config->buffer_size, layout->info->timestamp);
}

static void prv_time_am_pm_update(const LayoutLayer *layout_ref,
                                  const LayoutNodeTextDynamicConfig *config, char *buffer,
                                  bool render) {
  const TimelineLayout *layout = (TimelineLayout *)layout_ref;
  clock_get_time_word(buffer, config->buffer_size, layout->info->timestamp);
}

static GTextNode *prv_time_node_constructor(
    const LayoutLayer *layout_ref, const LayoutNodeConstructorConfig *config) {
  static const LayoutNodeTextDynamicConfig s_time_default_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = timeline_layout_time_text_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.font_key = FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM,
    .text.alignment = LayoutTextAlignment_Center,
  };
  static const LayoutNodeTextDynamicConfig s_time_digits_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_time_digits_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.font_key = FONT_KEY_LECO_42_NUMBERS,
    .text.alignment = LayoutTextAlignment_Left,
    .text.extent.margin.w = 4,
  };
  static const LayoutNodeTextDynamicConfig s_time_am_pm_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_time_am_pm_update,
    .buffer_size = 4,
    .text.font_key = FONT_KEY_GOTHIC_28_BOLD,
    .text.alignment = LayoutTextAlignment_Left,
    .text.extent.offset.y = 14,
  };

  const bool use_large_time = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* small */ false, /* medium */ false, /* large */ true, /* extralarge */ true);

  if (!use_large_time) {
    return layout_create_text_node_from_config(layout_ref, &s_time_default_config.text.extent.node);
  }

  GTextNodeHorizontal *horizontal_node = graphics_text_node_create_horizontal(2);
  horizontal_node->horizontal_alignment = GTextAlignmentCenter;
  GTextNode *digits_node = layout_create_text_node_from_config(
      layout_ref, &s_time_digits_config.text.extent.node);
  GTextNode *am_pm_node = layout_create_text_node_from_config(
      layout_ref, &s_time_am_pm_config.text.extent.node);
  graphics_text_node_container_add_child(&horizontal_node->container, digits_node);
  graphics_text_node_container_add_child(&horizontal_node->container, am_pm_node);
  return &horizontal_node->container.node;
}

static GTextNode *prv_card_view_constructor(TimelineLayout *timeline_layout) {
  static const LayoutNodeTextDynamicConfig s_title_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_until_time_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
    .text.alignment = LayoutTextAlignment_Center,
    .text.extent.margin.h = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* small */ PBL_IF_RECT_ELSE(2, 0),
      /* medium */ PBL_IF_RECT_ELSE(2, 0),
      /* large */ 2,
      /* extralarge */ 2), // title margin height
  };
  static const LayoutNodeConstructorConfig s_time_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_time_node_constructor,
    .extent.margin.h = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* small */ PBL_IF_RECT_ELSE(9, 1),
      /* medium */ PBL_IF_RECT_ELSE(9, 1),
      /* large */ 5,
      /* extralarge */ 5), // time margin height
  };
  static const LayoutNodeExtentConfig s_icon_config = {
    .node.type = LayoutNodeType_TimelineIcon,
    .margin.h = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* small */ PBL_IF_RECT_ELSE(3, 1),
      /* medium */ PBL_IF_RECT_ELSE(3, 1),
      /* large */ 0,
      /* extralarge */ 0), // icon margin height
  };
  static const LayoutNodeTextDynamicConfig s_subtitle_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_subtitle_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
    .text.alignment = LayoutTextAlignment_Center,
  };
  static const LayoutNodeConfig * const s_vertical_config_nodes[] = {
    PBL_IF_RECT_ELSE(&s_title_config.text.extent.node, &s_icon_config.node),
    PBL_IF_RECT_ELSE(&s_time_config.extent.node, &s_title_config.text.extent.node),
    PBL_IF_RECT_ELSE(&s_icon_config.node, &s_time_config.extent.node),
    &s_subtitle_config.text.extent.node,
  };
  static const LayoutNodeVerticalConfig s_vertical_config = {
    .container.extent.node.type = LayoutNodeType_Vertical,
    .container.num_nodes = ARRAY_LENGTH(s_vertical_config_nodes),
    .container.nodes = (LayoutNodeConfig **)&s_vertical_config_nodes,
    .container.extent.offset.y = CARD_MARGIN_TOP,
    .container.extent.margin.h = CARD_MARGIN_TOP,
  };

  return layout_create_text_node_from_config(&timeline_layout->layout_layer,
                                             &s_vertical_config.container.extent.node);
}

//////////////////////////////////////////
// LayoutLayer API
//////////////////////////////////////////

bool alarm_layout_verify(bool existing_attributes[]) {
  return (existing_attributes[AttributeIdTitle] && existing_attributes[AttributeIdSubtitle]);
}

LayoutLayer *alarm_layout_create(const LayoutLayerConfig *config) {
  AlarmLayout *layout = task_zalloc_check(sizeof(AlarmLayout));

  static const TimelineLayoutImpl s_timeline_layout_impl = {
    .attributes = { AttributeIdTitle, AttributeIdSubtitle },
    .default_colors = { { .argb = GColorBlackARGB8 },
                        { .argb = GColorClearARGB8 },
                        { .argb = GColorJaegerGreenARGB8 } },
    .default_icon = TIMELINE_RESOURCE_ALARM_CLOCK,
    .card_icon_align = GAlignCenter,
    .card_icon_size = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* small */ TimelineResourceSizeSmall,
      /* medium */ TimelineResourceSizeSmall,
      /* large */ TimelineResourceSizeLarge,
      /* extralarge */ TimelineResourceSizeLarge),
    .card_view_constructor = prv_card_view_constructor,
  };

  timeline_layout_init((TimelineLayout *)layout, config, &s_timeline_layout_impl);

  return (LayoutLayer *)layout;
}