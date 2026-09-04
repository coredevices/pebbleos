/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/generic_layout.h"
#include "pbl/services/timeline/timeline_layout.h"

#include "applib/graphics/gtypes.h"
#include "applib/ui/ui.h"
#include "font_resource_keys.auto.h"
#include "kernel/pbl_malloc.h"
#include "pbl/services/clock.h"
#include <pbl/logging/logging.h>
#include "pbl/util/size.h"

//////////////////////////////////////////
//  Card Mode
//////////////////////////////////////////

#define CARD_MARGIN_TOP                                            \
    PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,     \
      /* This is the same as Medium until Small is designed */     \
      /* small */ PBL_IF_RECT_ELSE(8, 13),                         \
      /* medium */ PBL_IF_RECT_ELSE(8, 13),                        \
      /* large */ 2,                                               \
      /* This is the same as Large until ExtraLarge is designed */ \
      /* extralarge */ 2                                           \
    )
#define CARD_MARGIN_BOTTOM PBL_IF_RECT_ELSE(7, 0)
#define CARD_LINE_DELTA -2

#if PBL_RECT
static void prv_horizontal_rule_node_callback(GContext *ctx, const GRect *box,
                                              const GTextNodeDrawConfig *config,
                                              bool render, GSize *size_out, void *user_data) {
  const LayoutLayer *layout = user_data;

  const int16_t horizontal_margin = 1;
  const int16_t hr_height = 2;
  GRect hr_box = GRectZero;
  if (box && render) {
    hr_box = grect_inset_internal(*box, horizontal_margin, 0);
    hr_box.size.h = hr_height;

    const LayoutColors *colors = layout_get_colors(layout);
    if (colors) {
      graphics_context_set_fill_color(ctx, colors->primary_color);
    }

    graphics_fill_rect(ctx, &hr_box);
  }

  if (size_out) {
    *size_out = hr_box.size;
  }
}

static GTextNode *prv_horizontal_rule_constructor(const LayoutLayer *layout_ref,
                                                  const LayoutNodeConstructorConfig *config) {
  GTextNodeCustom *custom_node = graphics_text_node_create_custom(prv_horizontal_rule_node_callback,
                                                                  (void *)layout_ref);
  if (custom_node) {
    custom_node->node.offset.y = 11;
    custom_node->node.margin.h = 12;
    return &custom_node->node;
  } else {
    return NULL;
  }
}
#endif // PBL_RECT

static GTextNode *prv_card_view_constructor(TimelineLayout *timeline_layout) {
  static const LayoutNodeExtentConfig s_icon_config = {
    .node.type = LayoutNodeType_TimelineIcon,
#if PBL_RECT
    .offset.x = -1, // icon offset x
    .margin.w = 9, // icon margin width
#endif
    .margin.h = PBL_IF_RECT_ELSE(-2, 2), // icon margin height
  };
  static const LayoutNodeTextDynamicConfig s_time_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = timeline_layout_time_text_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
    .text.alignment = PBL_IF_RECT_ELSE(LayoutTextAlignment_Right, LayoutTextAlignment_Center),
    .text.extent.margin.h = PBL_IF_RECT_ELSE(0, -2), // time margin height
  };
  static const LayoutNodeTextAttributeConfig s_title_config = {
    .attr_id = AttributeIdTitle,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Title,
    .text.line_spacing_delta = CARD_LINE_DELTA,
    .text.extent.margin.h = 4, // title margin height
  };
  static const LayoutNodeTextAttributeConfig s_subtitle_config = {
    .attr_id = AttributeIdSubtitle,
    // This is spec'd to always be Gothic 24 Bold regardless of content size
    .text.font_key = FONT_KEY_GOTHIC_24_BOLD,
    .text.line_spacing_delta = CARD_LINE_DELTA,
    .text.extent.margin.h = 10, // subtitle margin height
  };
  static const LayoutNodeTextAttributeConfig s_location_config = {
    .attr_id = AttributeIdLocationName,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Header,
    .text.extent.margin.h = 10, // location margin height
  };
  static const LayoutNodeTextAttributeConfig s_body_config = {
    .attr_id = AttributeIdBody,
    .text.style = LayoutContentSizeDefault,
    .text.style_font = TextStyleFont_Body,
    .text.extent.margin.h = 12, // body margin height
  };

#if PBL_RECT
  static const LayoutNodeConstructorConfig s_horizontal_rule_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_horizontal_rule_constructor,
  };
  static const LayoutNodeConfig * const s_icon_vertical_config_nodes[] = {
    &s_icon_config.node,
  };
  static const LayoutNodeVerticalConfig s_icon_vertical_container_config = {
    .vertical_alignment = LayoutVerticalAlignment_Center,
    .container.extent.node.type = LayoutNodeType_Vertical,
    .container.num_nodes = ARRAY_LENGTH(s_icon_vertical_config_nodes),
    .container.nodes = (LayoutNodeConfig **)&s_icon_vertical_config_nodes,
  };
  static const LayoutNodeConfig * const s_time_vertical_config_nodes[] = {
    &s_time_config.text.extent.node,
  };
  static const LayoutNodeVerticalConfig s_time_vertical_container_config = {
    .vertical_alignment = LayoutVerticalAlignment_Center,
    .container.extent.node.type = LayoutNodeType_Vertical,
    .container.num_nodes = ARRAY_LENGTH(s_time_vertical_config_nodes),
    .container.nodes = (LayoutNodeConfig **)&s_time_vertical_config_nodes,
  };
  static const LayoutNodeConfig * const s_horizontal_config_nodes[] = {
    &s_icon_vertical_container_config.container.extent.node,
    &s_time_vertical_container_config.container.extent.node,
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
    &s_horizontal_rule_config.extent.node,
#else
    &s_icon_config.node,
    &s_time_config.text.extent.node,
#endif
    &s_title_config.text.extent.node,
    &s_subtitle_config.text.extent.node,
    &s_location_config.text.extent.node,
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

bool generic_layout_verify(bool existing_attributes[]) {
  return existing_attributes[AttributeIdTitle];
}

LayoutLayer *generic_layout_create(const LayoutLayerConfig *config) {
  GenericLayout *layout = task_zalloc_check(sizeof(GenericLayout));

  static const TimelineLayoutImpl s_timeline_layout_impl = {
    .attributes = { AttributeIdTitle, AttributeIdSubtitle },
    .default_colors = { { .argb = GColorBlackARGB8 },
                        { .argb = GColorWhiteARGB8 },
                        { .argb = GColorSunsetOrangeARGB8 } },
    .default_icon = TIMELINE_RESOURCE_NOTIFICATION_FLAG,
    .card_icon_align = PBL_IF_RECT_ELSE(GAlignLeft, GAlignCenter),
    .card_icon_size = PREFERRED_CONTENT_SIZE_SWITCH(PreferredContentSizeDefault,
      //! @note this is the same as Medium until Small is designed
      /* small */ TimelineResourceSizeTiny,
      /* medium */ TimelineResourceSizeTiny,
      /* large */ TimelineResourceSizeSmall,
      //! @note this is the same as Large until ExtraLarge is designed
      /* extralarge */ TimelineResourceSizeSmall),
    .card_view_constructor = prv_card_view_constructor,
  };

  timeline_layout_init((TimelineLayout *)layout, config, &s_timeline_layout_impl);

  return (LayoutLayer *)layout;
}
