/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/timeline_layout.h"

#include "applib/preferred_content_size.h"
#include "applib/ui/ui.h"
#include "apps/system/timeline/text_node.h"
#include "apps/system/timeline/layer.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/kernel_ui.h"
#include "popups/timeline/peek.h"
#include "process_state/app_state/app_state.h"
#include "resource/timeline_resource_ids.auto.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/timeline/timeline_resources.h"
#include "shell/system_theme.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"

#define ARROW_SIZE_PX \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ PBL_IF_RECT_ELSE(8, 6),                          \
      /* medium */ PBL_IF_RECT_ELSE(8, 6),                         \
      /* large */ 6,                                               \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* x-large */ 6                                              \
    )

#define TIME_NUMBERS_MARGIN_W                                      \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ 0,                                               \
      /* medium */ 0,                                              \
      /* large */ 2,                                               \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* x-large */ 2                                              \
    )

#define TIME_WORDS_OFFSET_Y                                        \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ 0,                                               \
      /* medium */ 0,                                              \
      /* large */ 2,                                               \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* x-large */ 2                                              \
    )

typedef struct TimelineLayoutStyle {
  int16_t fat_time_margin_h;
  int16_t thin_time_margin_h;
  int16_t primary_line_spacing_delta;
  int16_t primary_list_margin_h;
  int16_t primary_secondary_peek_margin_h;
  bool thin_can_have_secondary;
  int16_t fat_future_title_offset_y;
  int16_t fat_past_title_offset_y;
  int16_t thin_future_title_offset_y;
  int16_t thin_past_title_offset_y;
} TimelineLayoutStyle;

static const TimelineLayoutStyle s_style_medium = {
  .fat_time_margin_h = -8,
  .thin_time_margin_h = -8,
  .primary_list_margin_h = 6,
  .primary_line_spacing_delta = -2,
  .fat_future_title_offset_y = 0,
  .fat_past_title_offset_y = 0,
  .thin_future_title_offset_y = 0,
  .thin_past_title_offset_y = 0,
};

static const TimelineLayoutStyle s_style_large = {
  .fat_time_margin_h = -3,
  .thin_time_margin_h = -6,
  .primary_list_margin_h = 2,
  // PBL-42540: This property is dependent on the screen size. Whether there can be a secondary
  // depends on whether the remaining screen space after fat permits.
  .primary_secondary_peek_margin_h = -5,
  .thin_can_have_secondary = true,
  .fat_future_title_offset_y = 25,
  .fat_past_title_offset_y = -30,
  .thin_future_title_offset_y = 45,
  .thin_past_title_offset_y = 10,
};

static const TimelineLayoutStyle * const s_styles[NumPreferredContentSizes] = {
  [PreferredContentSizeSmall] = &s_style_medium,
  [PreferredContentSizeMedium] = &s_style_medium,
  [PreferredContentSizeLarge] = &s_style_large,
  [PreferredContentSizeExtraLarge] = &s_style_large,
};

static GPath s_page_break_arrow_path = {
  .num_points = 3,
  .points = (GPoint[]) {{-ARROW_SIZE_PX, 0}, {ARROW_SIZE_PX, 0}, {0, ARROW_SIZE_PX}}
};

static void prv_init_icon(TimelineLayout *layout, const GRect *icon_frame,
                          TimelineResourceSize icon_res_size, TimelineResourceId resource,
                          TimelineResourceId fallback_resource, const Uuid *app_id);
static void prv_deinit_icon(TimelineLayout *timeline_layout);
static void prv_init_colors(TimelineLayout *timeline_layout);
static void prv_update_proc(Layer *layer, GContext *ctx);
static GTextNode *prv_create_pin_view_node(TimelineLayout *layout);

static const TimelineLayoutStyle *prv_get_style(void) {
  return s_styles[PreferredContentSizeDefault];
}

TimelineResourceId timeline_layout_get_icon_resource_id(
    LayoutLayerMode mode, const AttributeList *attributes, TimelineResourceSize card_icon_size,
    TimelineResourceId fallback_resource) {
  AttributeId card_attr_id;
  switch (card_icon_size) {
    default:
    case TimelineResourceSizeTiny:
      card_attr_id = AttributeIdIconTiny;
      break;
    case TimelineResourceSizeSmall:
      card_attr_id = AttributeIdIconSmall;
      break;
    case TimelineResourceSizeLarge:
      card_attr_id = AttributeIdIconLarge;
      break;
  }
  const bool is_card = (mode == LayoutLayerModeCard);
  const AttributeId primary_id = is_card ? card_attr_id : AttributeIdIconPin;
  const AttributeId secondary_id = AttributeIdIconTiny;
  const AttributeId tertiary_id = is_card ? AttributeIdIconPin : card_attr_id;
  return (attribute_get_uint32(attributes, primary_id, 0) ?:
          attribute_get_uint32(attributes, secondary_id, 0) ?:
          attribute_get_uint32(attributes, tertiary_id, fallback_resource));
}

void timeline_layout_init_with_icon_id(TimelineLayout *layout, const LayoutLayerConfig *config,
                                       const TimelineLayoutImpl *timeline_layout_impl,
                                       TimelineResourceId icon_resource) {
  TimelineLayoutInfo *info = config->context;

  static const LayoutLayerImpl s_layout_layer_impl = {
    .size_getter = timeline_layout_get_content_size,
    .destructor = timeline_layout_destroy,
    .mode_setter = timeline_layout_change_mode,
#if PBL_COLOR
    .color_getter = timeline_layout_get_colors,
#endif
  };

  *layout = (TimelineLayout) {
    .layout_layer = {
      .mode = config->mode,
      .attributes = config->attributes,
      .impl = &s_layout_layer_impl,
    },
    .impl = timeline_layout_impl,
    .info = info,
    .has_page_break = false,
  };

  prv_init_colors(layout);

  layer_init(&layout->layout_layer.layer, config->frame);
  layer_set_clips(&layout->layout_layer.layer, false);
  layer_set_update_proc((Layer *)layout, prv_update_proc);

  GRect icon_frame;
  timeline_layout_get_icon_frame(&(GRect) { GPointZero , config->frame->size },
                                 info->scroll_direction, &icon_frame);
  const TimelineResourceSize icon_size = (config->mode == LayoutLayerModeCard)
      ? layout->impl->card_icon_size : TimelineResourceSizeTiny;
  prv_init_icon(layout, &icon_frame, icon_size, icon_resource, layout->impl->default_icon,
                config->app_id);
  timeline_layout_init_view(layout, layout->layout_layer.mode);
}

void timeline_layout_init(TimelineLayout *layout, const LayoutLayerConfig *config,
                          const TimelineLayoutImpl *timeline_layout_impl) {
  const TimelineResourceId icon_resource = timeline_layout_get_icon_resource_id(
      config->mode, config->attributes, timeline_layout_impl->card_icon_size,
      timeline_layout_impl->default_icon);
  timeline_layout_init_with_icon_id(layout, config, timeline_layout_impl, icon_resource);
}

void timeline_layout_deinit(TimelineLayout *timeline_layout) {
  if (timeline_layout->metric_icon_layers) {
    for (unsigned int i = 0; i < timeline_layout->num_metric_icon_layers; i++) {
      kino_layer_destroy(timeline_layout->metric_icon_layers[i]);
    }
    task_free(timeline_layout->metric_icon_layers);
    timeline_layout->num_metric_icon_layers = 0;
    timeline_layout->metric_icon_layers = NULL;
  }
  animation_unschedule(timeline_layout->transition_animation);
  timeline_layout_deinit_view(timeline_layout);
  prv_deinit_icon(timeline_layout);
  layer_deinit(&timeline_layout->layout_layer.layer);
  i18n_free_all(timeline_layout);
}

void timeline_layout_init_info(TimelineLayoutInfo *info, TimelineItem *item, time_t current_day) {
  *info = (TimelineLayoutInfo) {
    .timestamp = item->header.timestamp,
    .duration_s = item->header.duration * SECONDS_PER_MINUTE,
    .current_day = current_day,
    .all_day = item->header.all_day,
  };

  info->end_time = info->timestamp + info->duration_s;

  // mark pin as all day if it spans the current day
  if (time_util_range_spans_day(info->timestamp, info->end_time, info->current_day)) {
    info->all_day = true;
    item->header.all_day = true;
  }

  // Pins representing the last day of a multiday event use the end time
  info->pin_time = (!info->all_day && time_util_get_midnight_of(info->timestamp) != current_day &&
                    time_util_get_midnight_of(info->end_time) == current_day)
      ? info->end_time : info->timestamp;
}

void timeline_layout_get_icon_frame(const GRect *bounds, TimelineScrollDirection scroll_direction,
                                    GRect *frame) {
  const GSize size = timeline_resources_get_gsize(TimelineResourceSizeTiny);
  const bool is_future = (scroll_direction == TimelineScrollDirectionDown);
  PBL_UNUSED const int offset_y_rect = -5;
  // Center the icon vertically at screen center (offsets differ by content size/style)
  const bool use_large_style = (PreferredContentSizeDefault >= PreferredContentSizeLarge);
  // s_style_large: future_top_margin=39, past layout origin=117, icon_offset_y=3
  // s_style_medium: future_top_margin=39, past layout origin=61, icon_offset_y=0
  PBL_UNUSED const int offset_y_round = use_large_style ? (is_future ? 76 : -2)
                                                        : (is_future ? 40 : 17);
  const GPoint origin = {
    .x = bounds->size.w - size.w + 2,
    .y = PBL_IF_RECT_ELSE(offset_y_rect, offset_y_round),
  };
  *frame = (GRect) { gpoint_add(bounds->origin, origin), size };
}

static KinoReel *prv_create_kino_reel_with_timeline_resource(
    TimelineLayout *timeline_layout, TimelineResourceSize icon_res_size,
    TimelineResourceId resource, TimelineResourceId fallback_resource, const Uuid *app_id) {
  timeline_layout->icon_info = (TimelineResourceInfo) {
    .res_id = resource,
    .app_id = app_id,
    .fallback_id = fallback_resource
  };
  AppResourceInfo *res_info = &timeline_layout->icon_res_info;
  timeline_resources_get_id(&timeline_layout->icon_info, icon_res_size, res_info);
  return kino_reel_create_with_resource_system(res_info->res_app_num, res_info->res_id);
}

static void prv_init_icon(TimelineLayout *timeline_layout, const GRect *icon_frame,
                          TimelineResourceSize icon_res_size, TimelineResourceId resource,
                          TimelineResourceId fallback_resource, const Uuid *app_id) {
  KinoReel *icon_reel = prv_create_kino_reel_with_timeline_resource(
      timeline_layout, icon_res_size, resource, fallback_resource, app_id);
  if (!icon_reel) {
    return;
  }

  GSize icon_size = kino_reel_get_size(icon_reel);
  const GSize max_icon_size = timeline_resources_get_gsize(icon_res_size);
  if ((icon_size.w > max_icon_size.w) || (icon_size.h > max_icon_size.h)) {
    // The icon is too large, use the fallback instead
    kino_reel_destroy(icon_reel);
    icon_reel = prv_create_kino_reel_with_timeline_resource(
        timeline_layout, icon_res_size, fallback_resource, TIMELINE_RESOURCE_NOTIFICATION_FLAG,
        NULL /* app_id */);
    if (!icon_reel) {
      return;
    }
    icon_size = kino_reel_get_size(icon_reel);
  }

  if (timeline_layout->layout_layer.mode == LayoutLayerModePeek) {
    icon_size = GSize(TIMELINE_PEEK_ICON_BOX_WIDTH,
                      timeline_layout->layout_layer.layer.frame.size.h);
  }
  const GRect frame = { icon_frame->origin, icon_size };

  // create the static reel
  timeline_layout->icon_size = frame.size;

  // init the kino layer
  KinoLayer *icon_layer = &timeline_layout->icon_layer;
  kino_layer_init(icon_layer, &frame);
  kino_layer_set_reel(icon_layer, icon_reel, true);
  if (timeline_layout->layout_layer.mode == LayoutLayerModeCard) {
    kino_layer_set_alignment(icon_layer, timeline_layout->impl->card_icon_align);
  } else if (PBL_IF_ROUND_ELSE(timeline_layout->layout_layer.mode == LayoutLayerModePeek, false)) {
    kino_layer_set_alignment(icon_layer, GAlignLeft);
  }
  layer_add_child(&timeline_layout->layout_layer.layer, &icon_layer->layer);
  kino_layer_play(icon_layer);
}

static void prv_deinit_icon(TimelineLayout *layout) {
  kino_layer_deinit(&layout->icon_layer);
}

////////////////////////
// Layout Impl
////////////////////////

GSize timeline_layout_get_content_size(GContext *ctx, LayoutLayer *layout) {
  return ((TimelineLayout *)layout)->view_size;
}

static void prv_update_proc(Layer *layer, GContext *ctx) {
  timeline_layout_render_view((TimelineLayout *)layer, ctx);
}

void timeline_layout_destroy(LayoutLayer *layout) {
  timeline_layout_deinit((TimelineLayout *)layout);
  task_free(layout);
}

void timeline_layout_change_mode(LayoutLayer *layout, LayoutLayerMode final_mode) {
  timeline_layout_deinit_view((TimelineLayout *)layout);
  layout->mode = final_mode;
  timeline_layout_init_view((TimelineLayout *)layout, final_mode);
}

void prv_init_colors(TimelineLayout *timeline_layout) {
  LayoutLayer *layout = &timeline_layout->layout_layer;
  const LayoutColors *default_colors = &timeline_layout->impl->default_colors;
  LayoutColors *colors = &timeline_layout->colors;
  colors->bg_color =
      (GColor) attribute_get_uint8(layout->attributes, AttributeIdBgColor,
                                   default_colors->bg_color.argb);
  colors->primary_color =
      (GColor) attribute_get_uint8(layout->attributes, AttributeIdPrimaryColor,
                                   default_colors->primary_color.argb);
  colors->secondary_color =
      (GColor) attribute_get_uint8(layout->attributes, AttributeIdSecondaryColor,
                                   default_colors->secondary_color.argb);
}

const LayoutColors *timeline_layout_get_colors(const LayoutLayer *layout_ref) {
  return &((TimelineLayout *)layout_ref)->colors;
}

////////////////////////
// View
////////////////////////

void timeline_layout_init_view(TimelineLayout *layout, LayoutLayerMode mode) {
  GTextNode *view_node = NULL;
  switch (mode) {
    case LayoutLayerModeCard:
      view_node = layout->impl->card_view_constructor(layout);
      break;
    case LayoutLayerModePeek:
    case LayoutLayerModePinnedThin:
    case LayoutLayerModePinnedFat:
      view_node = prv_create_pin_view_node(layout);
      break;
    case LayoutLayerModeNone:
    case NumLayoutLayerModes:
      break;
  }
  layout->view_node = view_node;
  timeline_layout_get_size(layout, graphics_context_get_current_context(), &layout->view_size);
}

void timeline_layout_deinit_view(TimelineLayout *layout) {
  if (layout->layout_layer.mode == LayoutLayerModeCard && layout->impl->card_view_deinitializer) {
    layout->impl->card_view_deinitializer(layout);
  }
  graphics_text_node_destroy(layout->view_node);
  layout->view_node = NULL;
}

static GTextNode *prv_create_all_day_text_node(const TimelineLayout *layout) {
  static const LayoutNodeTextBufferConfig s_all_day_config = {
    .text.extent.node.type = LayoutNodeType_TextBuffer,
    .str = i18n_noop("All day"),
    .use_i18n = true,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Title,
    .text.fixed_lines = 1,
    .text.alignment = PBL_IF_RECT_ELSE(LayoutTextAlignment_Left, LayoutTextAlignment_Right),
    // The font leaves blank space above the text; this pulls the header back up. Large rect
    // displays need less of a pull than the others.
    .text.extent.offset.y = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      /* This is the same as Medium until Small is designed */
      /* small */ -13,
      /* medium */ -13,
      /* large */ PBL_IF_RECT_ELSE(-8, -13),
      /* This is the same as Large until ExtraLarge is designed */
      /* extralarge */ PBL_IF_RECT_ELSE(-8, -13)),
    .text.extent.margin.h = -7,
  };
  GTextNodeText *text_node =
      (GTextNodeText *)layout_create_text_node_from_config(
          &layout->layout_layer, &s_all_day_config.text.extent.node);
  // TODO: PBL-30522 Enable timeline list view text flow
  // Remove when text flow is enabled
  if (PBL_IF_ROUND_ELSE(layout->layout_layer.mode == LayoutLayerModePinnedThin, false)) {
    const bool is_future = (layout->info->scroll_direction == TimelineScrollDirectionDown);
    const int padding_left = is_future ? 25 : 29;
    text_node->node.offset.x += padding_left;
    text_node->node.margin.w += padding_left;
  }
  return &text_node->node;
}

static void prv_time_number_update(const LayoutLayer *layout_ref,
                                   const LayoutNodeTextDynamicConfig *config, char *buffer,
                                   bool render) {
  const TimelineLayout *layout = (TimelineLayout *)layout_ref;
  clock_get_time_number(buffer, config->buffer_size, layout->info->pin_time);
}

static void prv_time_word_update(const LayoutLayer *layout_ref,
                                 const LayoutNodeTextDynamicConfig *config, char *buffer,
                                 bool render) {
  const TimelineLayout *layout = (TimelineLayout *)layout_ref;
  clock_get_time_word(buffer, config->buffer_size, layout->info->pin_time);
}

static GTextNode *prv_create_hour_text_node(const TimelineLayout *layout) {
  static const LayoutNodeTextDynamicConfig s_number_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_time_number_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_TimeHeaderNumbers,
    .text.fixed_lines = 1,
    .text.alignment = LayoutTextAlignment_Left,
    .text.extent.offset.y = -6,
    .text.extent.margin.w = TIME_NUMBERS_MARGIN_W,
  };
  static const LayoutNodeTextDynamicConfig s_word_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_time_word_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_TimeHeaderWords,
    .text.fixed_lines = 1,
    .text.alignment = LayoutTextAlignment_Left,
    .text.extent.offset.y = TIME_WORDS_OFFSET_Y,
  };
  static const LayoutNodeConfig * const s_horizontal_config_nodes[] = {
    &s_number_config.text.extent.node,
    &s_word_config.text.extent.node,
  };
  static const LayoutNodeHorizontalConfig s_horizontal_config = {
    .container.extent.node.type = LayoutNodeType_Horizontal,
    .container.num_nodes = ARRAY_LENGTH(s_horizontal_config_nodes),
    .container.nodes = (LayoutNodeConfig **)&s_horizontal_config_nodes,
  };

  GTextNodeHorizontal *horizontal_node =
      (GTextNodeHorizontal *)layout_create_text_node_from_config(
          &layout->layout_layer, &s_horizontal_config.container.extent.node);
  horizontal_node->horizontal_alignment = TIMELINE_LAYER_TEXT_ALIGNMENT;
  return &horizontal_node->container.node;
}

static GTextNode *prv_create_time_text_node(const TimelineLayout *layout) {
  if (layout->info->all_day) {
    return prv_create_all_day_text_node(layout);
  } else {
    return prv_create_hour_text_node(layout);
  }
}

static const char *prv_get_secondary_text(const TimelineLayout *layout) {
  const AttributeList *attributes = layout->layout_layer.attributes;
  return attribute_get_string(attributes, AttributeIdShortSubtitle, NULL) ?:
      attribute_get_string(attributes, layout->impl->attributes.secondary_id, "");
}

static void prv_peek_time_text_update(const LayoutLayer *layout_ref,
                                      const LayoutNodeTextDynamicConfig *config,
                                      char *buffer, bool render) {
  const TimelineLayout *layout = (TimelineLayout *)layout_ref;
  if (rtc_get_time() < layout->info->timestamp) {
    clock_get_until_time(buffer, config->buffer_size, layout->info->timestamp,
                         24 /* max_relative_hrs */);
  } else {
    strncpy(buffer, prv_get_secondary_text(layout), config->buffer_size);
    buffer[config->buffer_size - 1] = '\0';
  }
}

static GTextNode *prv_create_pin_view_node(TimelineLayout *layout) {
  const AttributeList *attributes = layout->layout_layer.attributes;

  const size_t num_vertical_nodes = 3;
  GTextNodeVertical *vertical_node = graphics_text_node_create_vertical(num_vertical_nodes);
  const bool is_future = (layout->info->scroll_direction == TimelineScrollDirectionDown);
  const bool is_peek = (layout->layout_layer.mode == LayoutLayerModePeek);
  vertical_node->vertical_alignment = is_peek ? GVerticalAlignmentCenter :
      PBL_IF_ROUND_ELSE((is_future ? GVerticalAlignmentBottom : GVerticalAlignmentTop),
                        GVerticalAlignmentTop);
  GTextNode *time_text_node = !is_peek ? prv_create_time_text_node(layout) : NULL;
  if (time_text_node) {
    graphics_text_node_container_add_child(&vertical_node->container, time_text_node);
  }

  const char *secondary_text = prv_get_secondary_text(layout);
  const bool is_fat = (layout->layout_layer.mode == LayoutLayerModePinnedFat);
  PBL_UNUSED const bool is_thin = (layout->layout_layer.mode == LayoutLayerModePinnedThin);
  const TimelineLayoutStyle *style = prv_get_style();
  const bool thin_can_have_secondary = style->thin_can_have_secondary;
  const bool has_secondary =
      (((is_peek || is_fat || thin_can_have_secondary) && !IS_EMPTY_STRING(secondary_text)) ||
       (is_peek && (rtc_get_time() < layout->info->timestamp)));
  const int peek_text_width =
      DISP_COLS - TIMELINE_PEEK_ICON_BOX_WIDTH - (2 * TIMELINE_PEEK_MARGIN);
  const GPoint peek_text_offset = GPoint(TIMELINE_PEEK_MARGIN, PBL_IF_RECT_ELSE(-5, -6));
  const GTextOverflowMode overflow =
      (has_secondary && !is_peek) ? GTextOverflowModeTrailingEllipsis : GTextOverflowModeFill;
  if (PBL_IF_ROUND_ELSE(!is_thin, true)) {
    // Move the hour and title closer together
    const int hour_title_margin = is_fat ? style->fat_time_margin_h : style->thin_time_margin_h;
    if (time_text_node) {
      time_text_node->margin.h += hour_title_margin;
    }

    static const LayoutNodeTextConfig s_primary_config = {
      .extent.node.type = LayoutNodeType_Text,
      .style = LayoutContentSizeDefault,
      .style_font = TextStyleFont_Title,
      .alignment = ToLayoutTextAlignment(TIMELINE_LAYER_TEXT_ALIGNMENT),
    };

    GTextNodeText *primary_node =
        (GTextNodeText *)layout_create_text_node_from_config(
            &layout->layout_layer, &s_primary_config.extent.node);
    primary_node->text =
        attribute_get_string(attributes, AttributeIdShortTitle, NULL) ?:
        attribute_get_string(attributes, layout->impl->attributes.primary_id, "");
    primary_node->line_spacing_delta = style->primary_line_spacing_delta;
    int num_primary_lines = is_fat ? 2 : 1;
    if (is_peek) {
      if (!has_secondary) {
        num_primary_lines = 2;
        const int primary_only_offset_y = -2;
        primary_node->node.offset.y += primary_only_offset_y;
        primary_node->line_spacing_delta = -5;
      }
      gpoint_add_eq(&primary_node->node.offset, peek_text_offset);
      primary_node->max_size.w = peek_text_width;
    }
    primary_node->max_size.h = num_primary_lines * fonts_get_font_height(primary_node->font);
    primary_node->overflow = overflow;
    if (!is_peek) {
      primary_node->node.margin.h = style->primary_list_margin_h;
    } else if (has_secondary) {
      primary_node->node.margin.h = style->primary_secondary_peek_margin_h;
    }
    graphics_text_node_container_add_child(&vertical_node->container, &primary_node->node);
  }

  if (has_secondary) {
    LayoutNodeTextDynamicConfig secondary_config = {
      .text.extent.node.type = is_peek ? LayoutNodeType_TextDynamic : LayoutNodeType_Text,
      .update = prv_peek_time_text_update,
      .buffer_size = ATTRIBUTE_SUBTITLE_MAX_LEN,
      .text.style = LayoutContentSizeDefault,
      .text.style_font = TextStyleFont_PinSubtitle,
      .text.alignment = ToLayoutTextAlignment(TIMELINE_LAYER_TEXT_ALIGNMENT),
    };

    GTextNodeText *secondary_node =
        (GTextNodeText *)layout_create_text_node_from_config(
            &layout->layout_layer, &secondary_config.text.extent.node);
    if (is_peek) {
      secondary_node->node.offset = peek_text_offset;
      secondary_node->max_size.w = peek_text_width;
    } else {
      secondary_node->text = secondary_text;
    }
    secondary_node->overflow = overflow;
    graphics_text_node_container_add_child(&vertical_node->container, &secondary_node->node);

    // TODO: PBL-30522 Enable timeline list view text flow
    // Remove when text flow is enabled
    if (PBL_IF_ROUND_ELSE(!is_future && !is_peek, false)) {
      const int padding_left = 8;
      secondary_node->node.offset.x += padding_left;
      secondary_node->node.margin.w += padding_left;
    }
  }

  if (PBL_IF_ROUND_ELSE(!is_peek, false)) {
    if (is_fat) {
      vertical_node->container.node.offset.y =
          is_future ? style->fat_future_title_offset_y : style->fat_past_title_offset_y;
    } else {
      vertical_node->container.node.offset.y =
          is_future ? style->thin_future_title_offset_y : style->thin_past_title_offset_y;
    }
  }

  if (is_peek) {
    vertical_node->container.size.w = DISP_COLS - TIMELINE_PEEK_ICON_BOX_WIDTH;
    const size_t num_horizontal_nodes = 2;
    GTextNodeHorizontal *horizontal_node =
        graphics_text_node_create_horizontal(num_horizontal_nodes);
    graphics_text_node_container_add_child(&horizontal_node->container,
                                           &vertical_node->container.node);
    const size_t num_horizontal_icon_nodes = 1;
    const size_t num_vertical_icon_nodes = 1;
    GTextNodeHorizontal *horizontal_icon_node =
        graphics_text_node_create_horizontal(num_horizontal_icon_nodes);
    GTextNodeVertical *vertical_icon_node =
        graphics_text_node_create_vertical(num_vertical_icon_nodes);
    graphics_text_node_container_add_child(&horizontal_icon_node->container,
                                           &vertical_icon_node->container.node);
    horizontal_icon_node->horizontal_alignment = GTextAlignmentCenter;
    vertical_icon_node->vertical_alignment = GVerticalAlignmentCenter;
    GTextNodeCustom *icon_node = timeline_layout_create_icon_node(layout);
    const unsigned int num_concurrent = layout->info->num_concurrent;
    const unsigned int concurrent_height = timeline_peek_get_concurrent_height(num_concurrent);
    gpoint_add_eq(&icon_node->node.offset,
                  GPoint(PBL_IF_RECT_ELSE(1, 2),
                         PBL_IF_RECT_ELSE(0, -1) - (concurrent_height / 2)));
    graphics_text_node_container_add_child(&vertical_icon_node->container, &icon_node->node);
    graphics_text_node_container_add_child(&horizontal_node->container,
                                           &horizontal_icon_node->container.node);
    return &horizontal_node->container.node;
  }

  return &vertical_node->container.node;
}

static void prv_get_pin_view_bounds(TimelineLayout *layout, GRect *box_out) {
  *box_out = (GRect) { GPointZero, layout->layout_layer.layer.frame.size };
  if (layout->layout_layer.mode == LayoutLayerModePeek) {
    const unsigned int num_concurrent = layout->info->num_concurrent;
    const unsigned int concurrent_height = timeline_peek_get_concurrent_height(num_concurrent);
    // Reduce frame to content size
    box_out->origin.y += concurrent_height;
    box_out->size.h -= concurrent_height;
    return;
  }
  box_out->size.w -= timeline_layer_get_ideal_sidebar_width();
  if (PBL_IF_ROUND_ELSE(layout->layout_layer.mode == LayoutLayerModePinnedThin, false)) {
    const int thin_height = 20;
    box_out->size.h = thin_height;
  } else if (layout->layout_layer.mode == LayoutLayerModePinnedFat) {
    box_out->size.h -= PBL_IF_ROUND_ELSE(30, 20);
  }
}

static void prv_get_card_view_bounds(TimelineLayout *layout, GRect *box_out) {
  const GRect *frame = &((Layer *)layout)->frame;
  *box_out = GRect(TIMELINE_CARD_MARGIN, 0, frame->size.w - 2 * TIMELINE_CARD_MARGIN,
                   TIMELINE_MAX_BOX_HEIGHT);
}

static void prv_render_view(TimelineLayout *layout, GContext *ctx, bool render, GSize *size_out) {
  const bool is_card = (layout->layout_layer.mode == LayoutLayerModeCard);
  const bool is_peek = (layout->layout_layer.mode == LayoutLayerModePeek);
  const bool PBL_UNUSED paging = PBL_IF_ROUND_ELSE((is_card || is_peek), false);
  GRect box;
  (is_card ? prv_get_card_view_bounds : prv_get_pin_view_bounds)(layout, &box);
  graphics_context_set_text_color(
      ctx, (is_card ? layout_get_colors((LayoutLayer *)layout)->primary_color : GColorBlack));
  static const GRect page_frame_on_screen =
     { { 0, STATUS_BAR_LAYER_HEIGHT }, { DISP_COLS, DISP_ROWS - STATUS_BAR_LAYER_HEIGHT } };
  const GTextNodeDrawConfig config = {
    .page_frame = is_peek ? &GRectZero : &page_frame_on_screen,
    .origin_on_screen = is_peek ? &GPointZero : &page_frame_on_screen.origin,
    .content_inset = 8,
    .text_flow = PBL_IF_ROUND_ELSE(paging, false),
    .paging = PBL_IF_ROUND_ELSE(paging, false),
  };
  (render ? graphics_text_node_draw :
            graphics_text_node_get_size)(layout->view_node, ctx, &box, &config, size_out);
}

void timeline_layout_render_view(TimelineLayout *layout, GContext *ctx) {
  const bool render = true;
  prv_render_view(layout, ctx, render, NULL);
}

void timeline_layout_get_size(TimelineLayout *layout, GContext *ctx, GSize *size_out) {
  const bool render = false;
  prv_render_view(layout, ctx, render, size_out);
}

/////////////////////////
// Card View
/////////////////////////

GTextNodeCustom *timeline_layout_create_icon_node(const TimelineLayout *layout) {
  return layout_node_create_kino_layer_wrapper((void *)&layout->icon_layer);
}

static void prv_last_updated_update(const LayoutLayer *layout,
                                    const LayoutNodeTextDynamicConfig *config, char *buffer,
                                    bool render) {
  AttributeList *attributes = layout->attributes;
  const time_t last_updated_time = attribute_get_uint32(attributes, AttributeIdLastUpdated, 0);
  clock_get_since_time(buffer, config->buffer_size, last_updated_time);
}

GTextNode *timeline_layout_create_card_view_from_config(const TimelineLayout *layout,
                                                        const LayoutNodeConfig *config) {
  if (config->type != LayoutNodeType_Vertical) {
    return timeline_layout_create_card_view_from_config(layout, config);
  }
  LayoutNodeVerticalConfig vertical_config = *(LayoutNodeVerticalConfig *)config;
  const bool has_last_updated =
      (attribute_get_uint32(layout->layout_layer.attributes, AttributeIdLastUpdated, 0) != 0);
  // One node for paragraphs and headings, and conditionally two more for the last updated time.
  const int num_default_nodes = has_last_updated ? 3 : 1;
  vertical_config.container.extra_capacity = num_default_nodes;
  GTextNode *vertical_node = layout_create_text_node_from_config(
      &layout->layout_layer, &vertical_config.container.extent.node);

  const LayoutNodeHeadingsParagraphsConfig headings_paragraphs_config = {
    .size = LayoutContentSizeDefault,
    .heading_style_font = TextStyleFont_ParagraphHeader,
    .paragraph_style_font = TextStyleFont_Body,
  };
  graphics_text_node_container_add_child((GTextNodeContainer *)vertical_node,
      &layout_create_headings_paragraphs_node(
          &layout->layout_layer, &headings_paragraphs_config)->container.node);

  if (!has_last_updated) {
    return vertical_node;
  }

  static const LayoutNodeTextBufferConfig s_header_config = {
    .text.extent.node.type = LayoutNodeType_TextBuffer,
    .str = i18n_noop("Last updated"),
    .use_i18n = true,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_ParagraphHeader,
    .text.extent.margin.h = TIMELINE_CARD_BODY_HEADER_MARGIN_HEIGHT,
  };
  graphics_text_node_container_add_child(
      (GTextNodeContainer *)vertical_node,
      layout_create_text_node_from_config(&layout->layout_layer,
                                          &s_header_config.text.extent.node));

  static const LayoutNodeTextDynamicConfig s_body_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_last_updated_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Body,
    .text.extent.margin.h = TIMELINE_CARD_BODY_MARGIN_HEIGHT,
  };
  graphics_text_node_container_add_child(
      (GTextNodeContainer *)vertical_node,
      layout_create_text_node_from_config(&layout->layout_layer,
                                          &s_body_config.text.extent.node));
  return vertical_node;
}

static void prv_page_break_node_callback(GContext *ctx, const GRect *box,
                                         const GTextNodeDrawConfig *config, bool render,
                                         GSize *size_out, void *user_data) {
  TimelineLayout *layout = user_data;
  const GRect *bounds = &layout->layout_layer.layer.bounds;
  const int16_t height = bounds->size.h - box->origin.y;
  if (render) {
    graphics_context_set_fill_color(ctx, layout_get_colors((LayoutLayer *)layout)->primary_color);
    const int arrow_offset = ((int[NumPreferredContentSizes]) {
      //! @note this is the same as Medium until Small is designed
      [PreferredContentSizeSmall] = PBL_IF_ROUND_ELSE(-1, 0),
      [PreferredContentSizeMedium] = PBL_IF_ROUND_ELSE(-1, 0),
      [PreferredContentSizeLarge] = -1,
      //! @note this is the same as Large until ExtraLarge is designed
      [PreferredContentSizeExtraLarge] = -1,
    })[PreferredContentSizeDefault];
    const GPoint origin = GPoint(bounds->size.w / 2,
                                 bounds->size.h - TIMELINE_CARD_ARROW_HEIGHT + arrow_offset);
    gpath_move_to(&s_page_break_arrow_path, origin);
    gpath_draw_filled(ctx, &s_page_break_arrow_path);
  } else {
    layout->page_break_height = height;
    layout->has_page_break = true;
  }
  if (size_out) {
    *size_out = (GSize) { bounds->size.w, height };
  }
}

GTextNodeCustom *timeline_layout_create_page_break_node(const TimelineLayout *layout) {
  GTextNodeCustom *page_break_node =
      graphics_text_node_create_custom(prv_page_break_node_callback, (void *)layout);
  return page_break_node;
}

void timeline_layout_time_text_update(const LayoutLayer *layout_ref,
                                      const LayoutNodeTextDynamicConfig *config,
                                      char *buffer, bool render) {
  const TimelineLayout *layout = (TimelineLayout *)layout_ref;
  clock_copy_time_string_timestamp(buffer, config->buffer_size, layout->info->timestamp);
}
