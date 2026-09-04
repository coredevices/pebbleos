/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/calendar_layout.h"
#include "pbl/services/timeline/calendar_layout_resources.h"
#include "pbl/services/timeline/timeline_layout.h"

#include "applib/fonts/fonts.h"
#include "applib/graphics/gtypes.h"
#include "applib/graphics/text.h"
#include "applib/preferred_content_size.h"
#include "applib/ui/ui.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"

//////////////////////////////////////////
//  Card Mode
//////////////////////////////////////////

#if PBL_RECT
#define CARD_ICON_OFFSET_Y                                           \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,       \
      /* small */ 6, /* medium */ 6, /* large */ -3, /* extralarge */ -3)
#define CARD_ICON_MARGIN_W                                           \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,       \
      /* small */ 3, /* medium */ 3, /* large */ 5, /* extralarge */ 5)
#define CARD_ICON_OFFSET { 0, CARD_ICON_OFFSET_Y }
#define CARD_ICON_MARGIN_H                                           \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,       \
      /* small */ 2, /* medium */ 2, /* large */ 0, /* extralarge */ 0)
#define CARD_ICON_MARGIN { CARD_ICON_MARGIN_W, CARD_ICON_MARGIN_H }
#else
#define CARD_ICON_OFFSET { 0, 9 }
#define CARD_ICON_MARGIN { 0, 6 }
#endif

//! This offset only applies for TIMELINE_RESOURCE_TIMELINE_CALENDAR and variants
#define CARD_ICON_CALENDAR_OFFSET_X PBL_IF_RECT_ELSE(-5, 0)

#define CARD_MARGIN_TOP                                              \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,       \
      /* small */ -1,                                                \
      /* medium */ -1,                                               \
      /* large */ 6,                                                 \
      /* extralarge */ 6)
#define CARD_MARGIN_BOTTOM PBL_IF_RECT_ELSE(7, 0)
#define CARD_NUM_TIME_DATE_SPACES 2
#define CARD_LINE_DELTA -2

#define CALENDAR_TIME_LINE_LENGTH (TIME_STRING_TIME_LENGTH + CARD_NUM_TIME_DATE_SPACES + \
                                   TIME_STRING_DATE_LENGTH)

#define DEFAULT_ICON_RESOURCE TIMELINE_RESOURCE_TIMELINE_CALENDAR

typedef void (*CalendarLayoutBufferCallback)(const TimelineLayout *layout, char *buffer,
                                             size_t buffer_size);

TimelineResourceId prv_get_icon_resource(
    LayoutLayerMode mode, const AttributeList *attributes, TimelineResourceSize icon_size) {
  const TimelineResourceId resource = timeline_layout_get_icon_resource_id(
      mode, attributes, icon_size, DEFAULT_ICON_RESOURCE);
  if (resource == DEFAULT_ICON_RESOURCE) {
    // Since calendar layout is using the default icon, it can be replaced with
    // the empty calendar icon for the date to be displayed within the icon
    return TIMELINE_RESOURCE_TIMELINE_EMPTY_CALENDAR;
  } else {
    return resource;
  }
}

TimelineResourceId prv_get_icon_resource_with_layout(const TimelineLayout *timeline_layout) {
  const LayoutLayer *layout = &timeline_layout->layout_layer;
  return prv_get_icon_resource(layout->mode, layout->attributes,
                               timeline_layout->impl->card_icon_size);
}

static GTextNode *prv_icon_node_constructor(
    const LayoutLayer *layout_ref, const LayoutNodeConstructorConfig *config) {
  CalendarLayout *layout = (CalendarLayout *)layout_ref;
  static const LayoutNodeExtentConfig s_icon_config = {
    .node.type = LayoutNodeType_TimelineIcon,
    .offset = CARD_ICON_OFFSET,
    .margin = CARD_ICON_MARGIN,
  };
  GTextNode *text_node = layout_create_text_node_from_config(layout_ref, &s_icon_config.node);
  const TimelineResourceId icon_resource =
      prv_get_icon_resource_with_layout(&layout->timeline_layout);
  if (icon_resource == TIMELINE_RESOURCE_TIMELINE_CALENDAR ||
      icon_resource == TIMELINE_RESOURCE_TIMELINE_EMPTY_CALENDAR) {
    text_node->offset.x += CARD_ICON_CALENDAR_OFFSET_X;
  }
  return text_node;
}

static void prv_day_node_callback(GContext *ctx, const GRect *box,
                                  const GTextNodeDrawConfig *config, bool render,
                                  GSize *size_out, void *user_data) {
  CalendarLayout *layout = user_data;
  const GRect *icon_frame = &layout->timeline_layout.icon_layer.layer.frame;
  const GPoint date_offset = {
    1, PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* small */ 16, /* medium */ 16, /* large */ 16, /* extralarge */ 16)
  };
  layer_set_frame((Layer *)&layout->date_layer,
                  &(GRect) { gpoint_add(icon_frame->origin, date_offset), icon_frame->size });
  clock_get_day_date(layout->day_date_buffer, sizeof(layout->day_date_buffer),
                     layout->timeline_layout.info->timestamp);
  *size_out = GSizeZero;
}

static GTextNode *prv_day_node_constructor(
    const LayoutLayer *layout_ref, const LayoutNodeConstructorConfig *config) {
  CalendarLayout *layout = (CalendarLayout *)layout_ref;
  if (prv_get_icon_resource_with_layout(&layout->timeline_layout) !=
      TIMELINE_RESOURCE_TIMELINE_EMPTY_CALENDAR) {
    return NULL;
  }
  const LayoutColors *colors = &layout->timeline_layout.impl->default_colors;
  text_layer_init_with_parameters(&layout->date_layer, &GRectZero, layout->day_date_buffer,
                                  fonts_get_system_font(PREFERRED_CONTENT_SIZE_SWITCH(
                                      PreferredContentSizeDefault,
                                      /* small */ FONT_KEY_LECO_20_BOLD_NUMBERS,
                                      /* medium */ FONT_KEY_LECO_20_BOLD_NUMBERS,
                                      /* large */ FONT_KEY_LECO_32_BOLD_NUMBERS,
                                      /* extralarge */ FONT_KEY_LECO_32_BOLD_NUMBERS)),
                                  colors->primary_color, GColorClear, GTextAlignmentCenter,
                                  GTextOverflowModeWordWrap);
  layer_add_child((Layer *)layout, (Layer *)&layout->date_layer);
  return &graphics_text_node_create_custom(prv_day_node_callback, layout)->node;
}

static void prv_format_glance_start_time(const TimelineLayout *layout, char *buffer,
                                         size_t buffer_size) {
  const time_t start_time = layout->info->timestamp;
  if (start_time >= layout->info->current_day) {
    // Not end of multi-day
    clock_copy_time_string_timestamp(buffer, buffer_size, start_time);
  }
}

static void prv_format_glance_end_time(const TimelineLayout *layout, char *buffer,
                                       size_t buffer_size) {
  const TimelineLayoutInfo *info = layout->info;
  if (info->all_day) {
    return;
  }
  if (info->timestamp < info->current_day) {
    // End of multi-day
    clock_copy_time_string_timestamp(buffer, buffer_size, info->end_time);
  } else if (info->end_time > layout->info->current_day + SECONDS_PER_DAY) {
    // Start of multi-day
    clock_get_date(buffer, buffer_size, info->end_time);
  } else {
    // Within a day
    clock_copy_time_string_timestamp(buffer, buffer_size, info->end_time);
  }
}

#if PBL_ROUND
static void prv_set_glance_time_line_round(const TimelineLayout *layout, char *buffer,
                                           size_t buffer_size) {
  prv_format_glance_start_time(layout, buffer, buffer_size);
  size_t pos = strnlen(buffer, CALENDAR_TIME_LINE_LENGTH);
  if (pos) {
    const char *delimiter = i18n_get(" - ", layout); // Freed in `timeline_layout_deinit`
    const int max_delimiter_i18n_size = 16;
    const size_t delimiter_size = strnlen(delimiter, max_delimiter_i18n_size);
    strncpy(buffer + pos, delimiter, buffer_size - pos);
    (buffer + pos)[MIN(delimiter_size, buffer_size - pos - 1)] = '\0';
    pos += delimiter_size;
  }
  prv_format_glance_end_time(layout, buffer + pos, buffer_size - pos);
}
#endif

static void prv_image_node_callback(GContext *ctx, const GRect *box,
                                    const GTextNodeDrawConfig *config, bool render,
                                    GSize *size_out, void *user_data) {
  GDrawCommandImage *image = user_data;
  if (render) {
    gdraw_command_image_draw(ctx, image, box->origin);
  }
  if (size_out) {
    *size_out = gdraw_command_image_get_bounds_size(image);
  }
}

static GTextNodeCustom *prv_create_image_node(GDrawCommandImage *image) {
  return graphics_text_node_create_custom(prv_image_node_callback, image);
}

static void prv_format_time_date(char *buffer, size_t buffer_length, time_t timestamp) {
  const size_t time_len = clock_copy_time_string_timestamp(buffer, buffer_length, timestamp);
  const size_t spaces_date_len = time_len + CARD_NUM_TIME_DATE_SPACES;
  if (buffer_length < spaces_date_len) {
    return;
  }
  char *cursor = &buffer[time_len];
  for (int i = 0; i < CARD_NUM_TIME_DATE_SPACES; i++) {
    *cursor++ = ' ';
  }
  const size_t buffer_length_left = buffer_length - spaces_date_len;
  clock_get_date(cursor, buffer_length_left, timestamp);
}

static bool prv_should_show_start_and_stop(const TimelineLayout *layout) {
  const TimelineLayoutInfo *info = layout->info;
  // Draw if this is a day in a multi-day event
  const bool is_multi_day = (layout->info->all_day || info->timestamp < info->current_day ||
                             info->end_time > info->current_day + SECONDS_PER_DAY);

  // But not if it spans one day
  const bool is_single_day = (info->duration_s <= SECONDS_PER_DAY);

  return (is_multi_day && !is_single_day);
}

static void prv_format_start_time(const TimelineLayout *layout, char *buffer, size_t buffer_size) {
  if (prv_should_show_start_and_stop(layout)) {
    const time_t start_time = layout->info->timestamp;
    prv_format_time_date(buffer, buffer_size, start_time);
  }
}

static void prv_format_end_time(const TimelineLayout *layout, char *buffer, size_t buffer_size) {
  if (prv_should_show_start_and_stop(layout)) {
    prv_format_time_date(buffer, buffer_size, layout->info->end_time);
  }
}

static bool prv_should_draw_recurring(const TimelineLayout *layout) {
  const AttributeList *attributes = layout->layout_layer.attributes;
  const uint8_t recurring = attribute_get_uint8(attributes, AttributeIdDisplayRecurring,
                                                CalendarRecurringTypeNone);
  return (recurring != CalendarRecurringTypeNone);
}

static GTextNode *prv_construct_if_recurring(
    const LayoutLayer *layout, const LayoutNodeConstructorConfig *config) {
  if (prv_should_draw_recurring((const TimelineLayout *)layout)) {
    return layout_create_text_node_from_config(layout, config->context);
  }
  return NULL;
}

static void prv_not_recurring_spacer_callback(GContext *ctx, const GRect *box,
                                              const GTextNodeDrawConfig *config, bool render,
                                              GSize *size_out, void *user_data) {
  if (size_out) {
    *size_out = (GSize) { 0, PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
        /* small */ 0, /* medium */ 0, /* large */ 3, /* extralarge */ 3) };
  }
}

static GTextNode *prv_construct_if_not_recurring(
    const LayoutLayer *layout, const LayoutNodeConstructorConfig *config) {
  if (prv_should_draw_recurring((const TimelineLayout *)layout)) {
    return NULL;
  }
  return &graphics_text_node_create_custom(prv_not_recurring_spacer_callback, NULL)->node;
}

typedef struct {
  GDrawCommandImage *image;
  CalendarLayoutBufferCallback callback;
  size_t buffer_size;
} IconLabelContext;

static GTextNode *prv_create_icon_label_node_rect(
    const LayoutLayer *layout, const LayoutNodeConstructorConfig *config) {
  const IconLabelContext *ctx = config->context;
  char buffer[ctx->buffer_size];
  memset(buffer, 0, ctx->buffer_size);
  ctx->callback((const TimelineLayout *)layout, buffer, ctx->buffer_size);
  const int time_margin_h = PBL_IF_RECT_ELSE(-1, 0);
  const LayoutNodeTextBufferConfig time_config = {
    .text.extent.node.type = LayoutNodeType_TextBuffer,
    .str = buffer,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
    .text.extent.margin.h = time_margin_h,
  };
  GTextNode *node =
      layout_create_text_node_from_config(layout, &time_config.text.extent.node);
  if (PBL_IF_RECT_ELSE(!node, true)) {
    // Don't append the icon if there is no node or if on round
    return node;
  }
  GTextNodeHorizontal *horizontal_node = graphics_text_node_create_horizontal(2);
  GTextNodeCustom *image_node = prv_create_image_node(ctx->image);
  image_node->node.offset.y = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* small */ 8, /* medium */ 8, /* large */ 11, /* extralarge */ 11);
  image_node->node.margin.w = 6;
  graphics_text_node_container_add_child(&horizontal_node->container, &image_node->node);
  graphics_text_node_container_add_child(&horizontal_node->container, node);
  return &horizontal_node->container.node;
}

static GTextNode *prv_construct_all_day_or_node(
    const LayoutLayer *layout_ref, const LayoutNodeConstructorConfig *config) {
  const TimelineLayout *layout = (TimelineLayout *)layout_ref;
  static const LayoutNodeTextBufferConfig s_all_day_config = {
    .text.extent.node.type = LayoutNodeType_TextBuffer,
    .str = i18n_noop("All Day"),
    .use_i18n = true,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
  };
  return layout_create_text_node_from_config(
      layout_ref, layout->info->all_day ? &s_all_day_config.text.extent.node : config->context);
}

static GTextNode *prv_card_view_constructor(TimelineLayout *timeline_layout) {
  static const LayoutNodeConstructorConfig s_icon_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_icon_node_constructor,
  };
  static const IconLabelContext s_glance_start_icon_label_context = {
    .image = &g_calendar_start_icon.image,
    .callback = PBL_IF_RECT_ELSE(prv_format_glance_start_time, prv_set_glance_time_line_round),
    .buffer_size = PBL_IF_RECT_ELSE(TIME_STRING_TIME_LENGTH, CALENDAR_TIME_LINE_LENGTH),
  };
  static const LayoutNodeConstructorConfig s_glance_start_time_with_icon_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_create_icon_label_node_rect,
    .context = (void *)&s_glance_start_icon_label_context,
  };
  static const LayoutNodeConstructorConfig s_glance_start_time_or_all_day_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_construct_all_day_or_node,
    .context = (void *)&s_glance_start_time_with_icon_config,
    .extent.margin.h = PBL_IF_ROUND_ELSE(-2, 0), // glance start time margin height
  };
  static const IconLabelContext s_glance_end_icon_label_context = {
    .image = &g_calendar_end_icon.image,
    .callback = prv_format_glance_end_time,
    .buffer_size = MAX(TIME_STRING_TIME_LENGTH, TIME_STRING_DATE_LENGTH),
  };
  PBL_UNUSED static const LayoutNodeConstructorConfig s_glance_end_time_with_icon_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_create_icon_label_node_rect,
    .context = (void *)&s_glance_end_icon_label_context,
  };
  static const LayoutNodeTextBufferConfig s_recurring_config = {
    .text.extent.node.type = LayoutNodeType_TextBuffer,
    .str = i18n_noop("Recurring"),
    .use_i18n = true,
    .text.font_key = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* small */ PBL_IF_RECT_ELSE(FONT_KEY_GOTHIC_14, FONT_KEY_GOTHIC_14_BOLD),
      /* medium */ PBL_IF_RECT_ELSE(FONT_KEY_GOTHIC_14, FONT_KEY_GOTHIC_14_BOLD),
      /* large */ FONT_KEY_GOTHIC_18_BOLD,
      /* extralarge */ FONT_KEY_GOTHIC_18_BOLD),
    .text.extent.offset.y = PBL_IF_RECT_ELSE(4, 1), // recurring offset y
    .text.extent.margin.h = PBL_IF_RECT_ELSE(4, 1), // recurring margin height
  };
  PBL_UNUSED static const LayoutNodeConstructorConfig s_if_recurring_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_construct_if_recurring,
    .context = (void *)&s_recurring_config,
  };
  PBL_UNUSED static const LayoutNodeConstructorConfig s_if_not_recurring_spacer_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_construct_if_not_recurring,
  };
  static const LayoutNodeTextAttributeConfig s_glance_title_config = {
    .attr_id = AttributeIdTitle,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Title,
    .text.fixed_lines = PBL_IF_RECT_ELSE(2, 1), // glance title fixed lines
    .text.line_spacing_delta = CARD_LINE_DELTA,
    .text.extent.margin.h = PBL_IF_RECT_ELSE(6, 4), // glance title margin height
  };
  static const LayoutNodeTextAttributeConfig s_glance_location_config = {
    .attr_id = AttributeIdLocationName,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
    .text.fixed_lines = 1, // glance location fixed lines
  };
  static const LayoutNodeConstructorConfig s_digit_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_day_node_constructor,
  };
  static const LayoutNodeConfig s_page_break_config = {
    .type = LayoutNodeType_TimelinePageBreak,
  };
  static const LayoutNodeTextAttributeConfig s_title_config = {
    .attr_id = AttributeIdTitle,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Title,
    .text.line_spacing_delta = CARD_LINE_DELTA,
    .text.extent.margin.h = 7, // title margin height
  };
  static const LayoutNodeTextAttributeConfig s_location_config = {
    .attr_id = AttributeIdLocationName,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
    .text.extent.margin.h = 15, // location margin height
  };
  static const IconLabelContext s_start_icon_label_context = {
    .image = &g_calendar_start_icon.image,
    .callback = prv_format_start_time,
    .buffer_size = CALENDAR_TIME_LINE_LENGTH,
  };
  PBL_UNUSED static const LayoutNodeConstructorConfig s_start_time_with_icon_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_create_icon_label_node_rect,
    .context = (void *)&s_start_icon_label_context,
  };
  static const IconLabelContext s_end_icon_label_context = {
    .image = &g_calendar_end_icon.image,
    .callback = prv_format_end_time,
    .buffer_size = CALENDAR_TIME_LINE_LENGTH,
  };
  PBL_UNUSED static const LayoutNodeConstructorConfig s_end_time_with_icon_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_create_icon_label_node_rect,
    .context = (void *)&s_end_icon_label_context,
    .extent.margin.h = 13, // end time margin height
  };
  static const LayoutNodeTextAttributeConfig s_body_config = {
    .attr_id = AttributeIdBody,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Body,
    .text.line_spacing_delta = CARD_LINE_DELTA,
    .text.extent.margin.h = 17, // body margin height
  };
  static const LayoutNodeTextAttributeConfig s_sender_config = {
    .attr_id = AttributeIdSender,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_PinSubtitle,
    .text.line_spacing_delta = CARD_LINE_DELTA,
    .text.extent.margin.h = 17, // sender margin height
  };

#if PBL_RECT
  static const LayoutNodeConfig * const s_metadata_config_nodes[] = {
    &s_if_not_recurring_spacer_config.extent.node,
    &s_glance_start_time_or_all_day_config.extent.node,
    &s_glance_end_time_with_icon_config.extent.node,
    &s_if_recurring_config.extent.node,
  };
  static const LayoutNodeVerticalConfig s_metadata_config = {
    .container.extent.node.type = LayoutNodeType_Vertical,
    .vertical_alignment = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* small */ LayoutVerticalAlignment_Center,
      /* medium */ LayoutVerticalAlignment_Center,
      /* large */ LayoutVerticalAlignment_Left,
      /* extralarge */ LayoutVerticalAlignment_Left),
    .container.num_nodes = ARRAY_LENGTH(s_metadata_config_nodes),
    .container.nodes = (LayoutNodeConfig **)&s_metadata_config_nodes,
  };
  static const LayoutNodeConfig * const s_horizontal_config_nodes[] = {
    &s_icon_config.extent.node,
    &s_metadata_config.container.extent.node,
  };
  static const LayoutNodeHorizontalConfig s_horizontal_config = {
    .container.extent.node.type = LayoutNodeType_Horizontal,
    .container.num_nodes = ARRAY_LENGTH(s_horizontal_config_nodes),
    .container.nodes = (LayoutNodeConfig **)&s_horizontal_config_nodes,
  };
#endif

  static const LayoutNodeConfig * const s_vertical_config_nodes[] = {
#if PBL_RECT
    &s_horizontal_config.container.extent.node,
    &s_glance_title_config.text.extent.node,
    &s_glance_location_config.text.extent.node,
    &s_digit_config.extent.node,
    &s_page_break_config,
    &s_title_config.text.extent.node,
    &s_location_config.text.extent.node,
    &s_start_time_with_icon_config.extent.node,
    &s_end_time_with_icon_config.extent.node,
    &s_body_config.text.extent.node,
    &s_sender_config.text.extent.node,
#else
    &s_icon_config.extent.node,
    &s_glance_start_time_or_all_day_config.extent.node,
    &s_glance_title_config.text.extent.node,
    &s_glance_location_config.text.extent.node,
    &s_if_recurring_config.extent.node,
    &s_digit_config.extent.node,
    &s_page_break_config,
    &s_title_config.text.extent.node,
    &s_location_config.text.extent.node,
    &s_body_config.text.extent.node,
    &s_sender_config.text.extent.node,
#endif
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

bool calendar_layout_verify(bool existing_attributes[]) {
  return existing_attributes[AttributeIdTitle];
}

LayoutLayer *calendar_layout_create(const LayoutLayerConfig *config) {
  CalendarLayout *layout = task_zalloc_check(sizeof(CalendarLayout));

  static const TimelineLayoutImpl s_timeline_layout_impl = {
    .attributes = { AttributeIdTitle, AttributeIdLocationName },
    .default_colors = { { .argb = GColorBlackARGB8 },
                        { .argb = GColorWhiteARGB8 },
                        { .argb = GColorSunsetOrangeARGB8 } },
    .default_icon = TIMELINE_RESOURCE_TIMELINE_CALENDAR,
    .card_icon_align = PBL_IF_RECT_ELSE(GAlignLeft, GAlignCenter),
    .card_icon_size = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* small */ TimelineResourceSizeSmall,
      /* medium */ TimelineResourceSizeSmall,
      /* large */ TimelineResourceSizeLarge,
      /* extralarge */ TimelineResourceSizeLarge),
    .card_view_constructor = prv_card_view_constructor,
  };

  const TimelineResourceId icon_resource =
      prv_get_icon_resource(config->mode, config->attributes,
                            s_timeline_layout_impl.card_icon_size);
  timeline_layout_init_with_icon_id((TimelineLayout *)layout, config, &s_timeline_layout_impl,
                                    icon_resource);

  return (LayoutLayer *)layout;
}
