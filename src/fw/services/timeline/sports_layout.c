/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/sports_layout.h"
#include "pbl/services/timeline/timeline_layout.h"

#include "applib/graphics/gtypes.h"
#include "applib/ui/ui.h"
#include <pbl/drivers/rtc.h>
#include "font_resource_keys.auto.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/clock.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"
#include "util/time/time.h"

_Static_assert(AttributeIdRankAway + 1 == AttributeIdRankHome,
               "Sports layout requires that all Home attributes are directly after Away");
_Static_assert(AttributeIdNameAway + 1 == AttributeIdNameHome,
               "Sports layout requires that all Home attributes are directly after Away");
_Static_assert(AttributeIdRecordAway + 1 == AttributeIdRecordHome,
               "Sports layout requires that all Home attributes are directly after Away");
_Static_assert(AttributeIdScoreAway + 1 == AttributeIdScoreHome,
               "Sports layout requires that all Home attributes are directly after Away");

//////////////////////////////////////////
//  Card Mode
//////////////////////////////////////////

#define CARD_MARGIN_TOP 3
#define CARD_MARGIN_BOTTOM PBL_IF_RECT_ELSE(7, 0)
#define CARD_LINE_DELTA -2

static GTextNode *prv_create_team_node(const LayoutLayer *layout, int team_offset);

static void prv_get_until_time(const LayoutLayer *layout, char *buffer, int buffer_size,
                               time_t timestamp) {
  const int max_relative_hrs = 24;
  const time_t now = rtc_get_time();
  const time_t difference = timestamp - now;
  size_t starts_len = 0;
  if (difference <= SECONDS_PER_HOUR * max_relative_hrs &&
      time_util_get_midnight_of(now) == time_util_get_midnight_of(timestamp)) {
    const char *starts = i18n_get("STARTS ", layout); // Freed by `timeline_layout_deinit`
    starts_len = strlen(starts);
    strncpy(buffer, starts, buffer_size);
  }
  clock_get_until_time_capitalized(buffer + starts_len, buffer_size - starts_len, timestamp,
                                   max_relative_hrs);
}

static void prv_time_until_update(const LayoutLayer *layout_ref,
                                  const LayoutNodeTextDynamicConfig *config, char *buffer,
                                  bool render) {
  const TimelineLayout *layout = (TimelineLayout *)layout_ref;
  prv_get_until_time(layout_ref, buffer, config->buffer_size, layout->info->timestamp);
}

static GTextNode *prv_subtitle_constructor(
    const LayoutLayer *layout_ref, const LayoutNodeConstructorConfig *config) {
  const SportsLayout *layout = (const SportsLayout *)layout_ref;
  static const LayoutNodeTextDynamicConfig s_time_until_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_time_until_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.font_key = FONT_KEY_GOTHIC_18_BOLD,
    .text.fixed_lines = 1,
    .text.alignment = LayoutTextAlignment_Center,
    .text.extent.margin.h = -1,
  };
  static const LayoutNodeTextAttributeConfig s_term_config = {
    .attr_id = AttributeIdSubtitle,
    .text.font_key = FONT_KEY_GOTHIC_18_BOLD,
    .text.fixed_lines = 1,
    .text.alignment = LayoutTextAlignment_Center,
    .text.extent.margin.h = 1,
  };
  return layout_create_text_node_from_config(
      layout_ref, (layout->state == GameStatePreGame ? &s_time_until_config.text.extent.node :
                                                       &s_term_config.text.extent.node));
}

static void prv_game_line_node_callback(GContext *ctx, const GRect *box,
                                        const GTextNodeDrawConfig *config, bool render,
                                        GSize *size_out, void *user_data) {
  const SportsLayout *layout = user_data;
  const int16_t offset_top = (layout->state == GameStatePreGame) ? 11 : 9;
  const int16_t offset_bottom = PBL_IF_RECT_ELSE(-2, 0);
  const int16_t min_y = box->origin.y + offset_top;
  const int16_t max_y = box->origin.y + box->size.h + offset_bottom;
  const int16_t offset_x = box->origin.x + box->size.w / 2;
  graphics_context_set_stroke_color(ctx, GColorBlack);
  graphics_draw_line(ctx, GPoint(offset_x, min_y), GPoint(offset_x, max_y));
  *size_out = GSizeZero;
}

static GTextNode *prv_game_constructor(
    const LayoutLayer *layout_ref, const LayoutNodeConstructorConfig *config) {
  const SportsLayout *layout = (SportsLayout *)layout_ref;
  const int num_teams = 2;
  const int num_nodes = num_teams + 1; // two team nodes and one line node
  GTextNodeHorizontal *game_node = graphics_text_node_create_horizontal(num_nodes);
  graphics_text_node_container_add_child(
      &game_node->container,
      &graphics_text_node_create_custom(prv_game_line_node_callback, (void *)layout)->node);
  for (int i = 0; i < num_teams; i++) {
    GTextNode *team_node = prv_create_team_node(layout_ref, i);
    graphics_text_node_container_add_child(&game_node->container, team_node);
  }
  return &game_node->container.node;
}

static GTextNode *prv_broadcaster_header_constructor(
    const LayoutLayer *layout_ref, const LayoutNodeConstructorConfig *config) {
  const char *broadcaster =
      attribute_get_string(layout_ref->attributes, AttributeIdBroadcaster, "");
  if (IS_EMPTY_STRING(broadcaster)) {
    return NULL;
  }
  static const LayoutNodeTextBufferConfig s_broadcaster_header_config = {
    .text.extent.node.type = LayoutNodeType_TextBuffer,
    .str = i18n_noop("Broadcaster"),
    .use_i18n = true,
    .text.font_key = FONT_KEY_GOTHIC_14,
    .text.line_spacing_delta = CARD_LINE_DELTA,
  };
  return layout_create_text_node_from_config(
      layout_ref, &s_broadcaster_header_config.text.extent.node);
}

static GTextNode *prv_card_view_constructor(TimelineLayout *timeline_layout) {
  SportsLayout *layout = (SportsLayout *)timeline_layout;
  AttributeList *attributes = layout->timeline_layout.layout_layer.attributes;
  layout->state = attribute_get_uint8(attributes, AttributeIdSportsGameState, GameStatePreGame);

  static const LayoutNodeConstructorConfig s_subtitle_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_subtitle_constructor,
  };
  static const LayoutNodeConstructorConfig s_game_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_game_constructor,
  };
  static const LayoutNodeExtentConfig s_icon_config = {
    .node.type = LayoutNodeType_TimelineIcon,
    .offset.y = PBL_IF_RECT_ELSE(5, 11), // icon offset y
    .margin.h = PBL_IF_RECT_ELSE(5, 11), // icon margin height
  };
  static const LayoutNodeConfig s_page_break_config = {
    .type = LayoutNodeType_TimelinePageBreak,
  };
  static const LayoutNodeTextAttributeConfig s_body_config = {
    .attr_id = AttributeIdBody,
    .text.font_key = FONT_KEY_GOTHIC_24_BOLD,
    .text.line_spacing_delta = CARD_LINE_DELTA,
    .text.extent.margin.h = 14, // body margin height
  };
  static const LayoutNodeConstructorConfig  s_broadcaster_header_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_broadcaster_header_constructor,
  };
  static const LayoutNodeTextAttributeConfig s_broadcaster_config = {
    .attr_id = AttributeIdBroadcaster,
    .text.font_key = FONT_KEY_GOTHIC_24_BOLD,
    .text.line_spacing_delta = CARD_LINE_DELTA,
    .text.extent.margin.h = 8, // broadcaster margin height
  };
  static const LayoutNodeConfig * const s_vertical_config_nodes[] = {
    &s_subtitle_config.extent.node,
    &s_game_config.extent.node,
    &s_icon_config.node,
    &s_page_break_config,
    &s_body_config.text.extent.node,
    &s_broadcaster_header_config.extent.node,
    &s_broadcaster_config.text.extent.node,
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

static GTextNode *prv_create_team_node(const LayoutLayer *layout_ref, int team_offset) {
  const SportsLayout *layout = (const SportsLayout *)layout_ref;
  AttributeList *attributes = layout->timeline_layout.layout_layer.attributes;

  const bool is_pregame = (layout->state == GameStatePreGame);
  const bool has_record = (attribute_find(attributes, AttributeIdRecordAway + team_offset) != NULL);

  const AttributeId large_attr = !is_pregame ? AttributeIdScoreAway :
                                               AttributeIdNameAway;
  const AttributeId small_attr = !is_pregame ? AttributeIdNameAway :
                                 has_record  ? AttributeIdRecordAway :
                                               AttributeIdRankAway;
  const char *large_font = is_pregame ? FONT_KEY_GOTHIC_28_BOLD :
                                        FONT_KEY_LECO_26_BOLD_NUMBERS_AM_PM;
  const char *small_font = FONT_KEY_GOTHIC_18_BOLD;

  const LayoutNodeTextAttributeConfig large_config = {
    .attr_id = large_attr + team_offset,
    .text.font_key = large_font,
#if PBL_COLOR
    .text.color = LayoutColor_Secondary,
#endif
    .text.fixed_lines = 1, // large fixed lines
    .text.alignment = LayoutTextAlignment_Center,
    .text.extent.margin.h = -2, // large margin height
  };
  const LayoutNodeTextAttributeConfig small_config = {
    .attr_id = small_attr + team_offset,
    .text.font_key = small_font,
#if PBL_COLOR
    .text.color = LayoutColor_Secondary,
#endif
    .text.fixed_lines = 1, // small fixed lines
    .text.alignment = LayoutTextAlignment_Center,
  };
  const LayoutNodeConfig * const vertical_config_nodes[] = {
    &large_config.text.extent.node,
    &small_config.text.extent.node,
  };
  const LayoutNodeVerticalConfig vertical_config = {
    .container.extent.node.type = LayoutNodeType_Vertical,
    .container.num_nodes = ARRAY_LENGTH(vertical_config_nodes),
    .container.nodes = (LayoutNodeConfig **)&vertical_config_nodes,
    .container.extent.offset.y = CARD_MARGIN_TOP,
    .container.extent.margin.h = CARD_MARGIN_TOP + CARD_MARGIN_BOTTOM,
  };

  GTextNodeVertical *vertical_node =
      (GTextNodeVertical *)layout_create_text_node_from_config(
          layout_ref, &vertical_config.container.extent.node);
  const GRect *bounds = &((Layer *)layout)->bounds;
  const int16_t midwidth = bounds->size.w / 2 - TIMELINE_CARD_MARGIN;
  vertical_node->container.size.w = midwidth;
  return &vertical_node->container.node;
}

//////////////////////////////////////////
// LayoutLayer API
//////////////////////////////////////////

bool sports_layout_verify(bool existing_attributes[]) {
  return existing_attributes[AttributeIdTitle];
}

LayoutLayer *sports_layout_create(const LayoutLayerConfig *config) {
  SportsLayout *layout = task_zalloc_check(sizeof(SportsLayout));

  static const TimelineLayoutImpl s_timeline_layout_impl = {
    .attributes = { AttributeIdTitle, AttributeIdSubtitle },
    .default_colors = { { .argb = GColorBlackARGB8 },
                        { .argb = GColorWhiteARGB8 },
                        { .argb = GColorVividCeruleanARGB8 } },
    .default_icon = TIMELINE_RESOURCE_TIMELINE_SPORTS,
    .card_icon_align = GAlignCenter,
    .card_icon_size = TimelineResourceSizeSmall,
    .card_view_constructor = prv_card_view_constructor,
  };

  timeline_layout_init((TimelineLayout *)layout, config, &s_timeline_layout_impl);

  return (LayoutLayer *)layout;
}
