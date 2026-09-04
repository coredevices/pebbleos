/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/services/timeline/notification_jumboji_table.h"
#include "pbl/services/timeline/notification_layout.h"

#include "applib/graphics/gtypes.h"
#include "kernel/pbl_malloc.h"
#include "kernel/ui/kernel_ui.h"
#include "resource/resource_ids.auto.h"
#include "resource/timeline_resource_ids.auto.h"
#include "pbl/services/clock.h"
#include "pbl/services/clock.h"
#include "pbl/services/blob_db/pin_db.h"
#include "pbl/services/notifications/alerts_preferences_private.h"
#include "pbl/services/notifications/notification_image.h"
#include "pbl/services/timeline/timeline_resources.h"
#include "shell/system_theme.h"
#include "pbl/util/math.h"
#include "pbl/util/size.h"
#include "pbl/util/string.h"

// NOTIFICATION
// Title -> Sender/App
// Subtitle -> Subject (Emails)
// Body -> Body
// Footer -> Friendly Timestamp

// REMINDER
// Title -> Friendly Timestamp
// Subtitle -> NA
// Body -> Title
// Footer -> Location

#define LAYOUT_MAX_HEIGHT 2500
#define CARD_MARGIN PBL_IF_ROUND_ELSE(12, 10)
// All paddings relate to padding above the object unless othersize noted
#define CARD_BOTTOM_PADDING 18
#define BOTTOM_BANNER_CIRCLE_RADIUS 8

static void prv_card_render(NotificationLayout *layout, GContext *ctx, bool render);
static const LayoutColors *prv_layout_get_colors(const LayoutLayer *layout);

static time_t prv_get_parent_timestamp(TimelineItem *reminder) {
  TimelineItem pin;
  if (S_SUCCESS != pin_db_get(&reminder->header.parent_id, &pin)) {
    return reminder->header.timestamp;
  }
  timeline_item_free_allocated_buffer(&pin);
  return pin.header.timestamp;
}

//////////////////////////////////////////
//  Card Mode
//////////////////////////////////////////

static const NotificationStyle s_notification_styles[NumPreferredContentSizes] = {
  [PreferredContentSizeSmall] = {
    .header_padding = 3,
    .title_padding = 3,
    .subtitle_upper_padding = PBL_IF_RECT_ELSE(1, 4),
    .subtitle_lower_padding = PBL_IF_RECT_ELSE(2, 1),
    .location_offset = PBL_IF_RECT_ELSE(3, 7),
    .location_margin = PBL_IF_RECT_ELSE(5, 9),
    .body_icon_offset = 3,
    .body_icon_margin = -5,
    .body_padding = PBL_IF_RECT_ELSE(1, 1),
#if PBL_ROUND
    .timestamp_upper_padding = 6,
    .timestamp_lower_padding = -3,
#else
    .timestamp_upper_padding = 3,
#endif
  },
  [PreferredContentSizeMedium] = {
#if PBL_ROUND
    .body_padding = 3,
    .subtitle_upper_padding = 3,
#endif
    .header_padding = 3,
    .title_padding = 3,
    .title_line_delta = -1,
    .subtitle_lower_padding = PBL_IF_RECT_ELSE(6, 2),
    .subtitle_line_delta = -1,
    .location_offset = PBL_IF_RECT_ELSE(-2, 6),
    .location_margin = PBL_IF_RECT_ELSE(3, 10),
    .body_icon_offset = 3,
    .body_icon_margin = -5,
    .body_line_delta = -1,
#if PBL_ROUND
    .timestamp_upper_padding = 6,
    .timestamp_lower_padding = -3,
#else
    .timestamp_upper_padding = 3
#endif
  },
  [PreferredContentSizeLarge] = {
    .title_offset_if_body_icon = -2,
    .subtitle_upper_padding = 2,
    .subtitle_lower_padding = PBL_IF_RECT_ELSE(4, 2),
    .subtitle_line_delta = -2,
    .location_offset = 6,
    .location_margin = 10,
    .body_icon_margin = -10,
    .body_padding = 2,
    // Round: -3 makes the Gothic-24 body pitch 21px, fitting 10 lines per 224px page
    // (9*21 + the 29px last-line reserve = 218 <= 224) instead of 9, shrinking the
    // page-seam gap from more than a line pitch (26px) to a 14px modulo.
    .body_line_delta = PBL_IF_RECT_ELSE(-2, -3),
#if PBL_ROUND
    .timestamp_upper_padding = 6,
#else
    .timestamp_upper_padding = 3,
#endif
  },
  [PreferredContentSizeExtraLarge] = {
    .subtitle_upper_padding = 2,
    .subtitle_lower_padding = 4,
    .subtitle_line_delta = -2,
    .location_offset = 6,
    .location_margin = 10,
    .body_icon_offset = 6,
    .body_icon_margin = -10,
    .body_line_delta = -2,
    .timestamp_upper_padding = 6,
  },
};

static bool prv_is_reminder(const NotificationLayout *layout) {
  return (layout->info.item->header.type == TimelineItemTypeReminder);
}

static void prv_reminder_timestamp_update(const LayoutLayer *layout_ref,
                                          const LayoutNodeTextDynamicConfig *config, char *buffer,
                                          bool render) {
  const NotificationLayout *layout = (NotificationLayout *)layout_ref;
  const int max_relative_hrs = 1;
  clock_get_until_time(buffer, config->buffer_size, prv_get_parent_timestamp(layout->info.item),
                       max_relative_hrs);
  const char *buffer_ptr = string_strip_leading_whitespace(buffer);
  memmove(buffer, buffer_ptr, config->buffer_size - (buffer_ptr - buffer));
}

static void prv_notification_timestamp_update(const LayoutLayer *layout_ref,
                                              const LayoutNodeTextDynamicConfig *config,
                                              char *buffer,
                                              bool render) {
  const NotificationLayout *layout = (NotificationLayout *)layout_ref;
  clock_get_since_time(buffer, config->buffer_size, layout->info.item->header.timestamp);
}

static const EmojiEntry s_emoji_table[] = JUMBOJI_TABLE(EMOJI_ENTRY);

static bool prv_each_emoji_codepoint(int index, Codepoint codepoint, void *context) {
  Codepoint *emoji_codepoint = context;
  if (codepoint_is_end_of_word(codepoint) ||
      codepoint_is_formatting_indicator(codepoint) ||
      codepoint_is_skin_tone_modifier(codepoint) ||
      codepoint_is_special(codepoint) ||
      codepoint_is_zero_width(codepoint) ||
      codepoint_should_skip(codepoint)) {
    // Skip this codepoint
    return true;
  } else if (codepoint_is_emoji(codepoint)) {
    if (*emoji_codepoint) {
      // This has more than one emoji
      goto fail;
    }
    // Found an emoji
    *emoji_codepoint = codepoint;
    return true;
  }
  // This is not an emoji-only string
fail:
  *emoji_codepoint = NULL_CODEPOINT;
  return false;
}

T_STATIC ResourceId prv_get_emoji_icon_by_string(const EmojiEntry *table, const char *str) {
  if (!str) {
    return INVALID_RESOURCE;
  }
  Codepoint emoji_codepoint = NULL_CODEPOINT;
  utf8_each_codepoint(str, prv_each_emoji_codepoint, &emoji_codepoint);
  for (unsigned int i = 0; i < ARRAY_LENGTH(s_emoji_table); i++) {
    const EmojiEntry *emoji = &s_emoji_table[i];
    if (emoji->codepoint == emoji_codepoint) {
      return emoji->resource_id;
    }
  }
  return INVALID_RESOURCE;
}

static ResourceId prv_get_emoji_icon(NotificationLayout *layout) {
  const char *body = attribute_get_string(layout->layout.attributes, AttributeIdBody, NULL);
  return prv_get_emoji_icon_by_string(s_emoji_table, body);
}

static bool prv_should_enlarge_emoji(NotificationLayout *layout) {
  return (!layout->info.show_notification_timestamp &&
          prv_get_emoji_icon(layout) != INVALID_RESOURCE);
}

#if NOTIFICATION_IMAGE_SUPPORTED
//! Space above and below the image band. Its own constant because body_padding is 0 for rect at
//! the default content size, which leaves the image touching the body text.
#define NOTIFICATION_IMAGE_PADDING (8)
#define NOTIFICATION_IMAGE_CORNER_RADIUS (4)

#if PBL_ROUND
//! Round cards page in fixed steps and the usable width narrows towards the top and bottom of the
//! circle, so the image takes a page of its own and sits centred in it.
#define NOTIFICATION_IMAGE_CIRCLE_INSET (8)
#define NOTIFICATION_IMAGE_RADIUS (DISP_ROWS / 2 - NOTIFICATION_IMAGE_CIRCLE_INSET)
//! Centre of the page the image gets, which sits between the status bar and the paging arrow and so
//! is a couple of pixels below the circle's own centre.
#define NOTIFICATION_IMAGE_PAGE_CENTRE_Y (STATUS_BAR_LAYER_HEIGHT + LAYOUT_HEIGHT / 2)

//! Whether a `width`-wide image of this aspect, centred in its page, clears the bezel.
//!
//! Checking the rectangle's own corners against the circle rather than bounding it by the inscribed
//! square: a landscape photo is far wider than the square that fits the same circle, and photos are
//! mostly landscape.
static bool prv_image_fits_circle(int16_t width, uint8_t aspect) {
  const int32_t half_w = width / 2;
  const int32_t half_h = ((int32_t)width * aspect) / 32;
  const int32_t dy = ABS(NOTIFICATION_IMAGE_PAGE_CENTRE_Y - DISP_ROWS / 2) + half_h;
  const int32_t r = NOTIFICATION_IMAGE_RADIUS;
  return (half_w * half_w) + (dy * dy) <= (r * r);
}
#endif

//! Clamped height/width in sixteenths of the image the phone holds, or 0 for none. Clamping bounds
//! the band a malformed attribute can reserve.
static uint8_t prv_image_aspect(const NotificationLayout *layout) {
  const uint8_t aspect =
      attribute_get_uint8(layout->layout.attributes, AttributeIdImageAspectRatio, 0);
  return aspect ? CLIP(aspect, NOTIFICATION_IMAGE_MIN_ASPECT, NOTIFICATION_IMAGE_MAX_ASPECT) : 0;
}

//! Size of the image itself for a content box `width` wide. Derived from the attribute rather than
//! the bitmap, so the card's height is final before the pixels arrive and nothing reflows when they
//! do — and so the requester and the renderer ask for the same thing.
static GSize prv_image_size(const NotificationLayout *layout, int16_t width) {
  const uint8_t aspect = prv_image_aspect(layout);
  GSize size = { width, (width * aspect) / 16 };
#if PBL_ROUND
  // Shrink to the widest that still clears the bezel. Bounded by the content width, so this is a
  // few dozen integer comparisons at most, and the result is cached with the node's size.
  while (size.w > 0 && !prv_image_fits_circle(size.w, aspect)) {
    size.w--;
    size.h = (size.w * aspect) / 16;
  }
#endif
  return size;
}

static void prv_image_node_callback(GContext *ctx, const GRect *box,
                                    const GTextNodeDrawConfig *config, bool render,
                                    GSize *size_out, void *user_data) {
  NotificationLayout *layout = user_data;
  const GSize image = prv_image_size(layout, box->size.w);
  GSize band = { box->size.w, image.h };
  GPoint origin = box->origin;

#if PBL_ROUND
  // Give the image a page to itself: a fixed-height band that straddled a page seam would be
  // sliced in half, since paging only knows how to break text.
  if (config && config->paging && config->page_frame->size.h > 0) {
    const int16_t page_h = config->page_frame->size.h;
    const int16_t page_y = config->origin_on_screen->y + box->origin.y -
                           config->page_frame->origin.y;
    const int16_t into_page = (page_y > 0) ? (page_y % page_h) : 0;
    const int16_t skip = into_page ? (page_h - into_page) : 0;
    band.h = skip + page_h;
    origin.y += skip + (page_h - image.h) / 2;
  }
#endif
  origin.x += (band.w - image.w) / 2;

  if (render) {
    const Uuid *item_id = &layout->info.item->header.id;
    const GBitmap *bitmap = notification_image_lock(item_id);
    if (bitmap) {
      const GSize bitmap_size = bitmap->bounds.size;
      const GRect dest = {
        { origin.x + (image.w - bitmap_size.w) / 2,
          origin.y + (image.h - bitmap_size.h) / 2 },
        bitmap_size,
      };
      graphics_context_set_compositing_mode(ctx, GCompOpAssign);
      graphics_draw_bitmap_in_rect(ctx, bitmap, &dest);
    } else if (notification_image_is_pending(item_id)) {
      // Still transferring: fill the reserved area so the card doesn't look broken. Once the phone
      // answers NoImage this stops and the space is simply blank.
      const GRect placeholder = { origin, image };
      graphics_context_set_fill_color(ctx, GColorLightGray);
      graphics_fill_round_rect(ctx, &placeholder, NOTIFICATION_IMAGE_CORNER_RADIUS, GCornersAll);
    }
    notification_image_unlock();
  }
  if (size_out) {
    *size_out = band;
  }
}

static GTextNode *prv_create_image_node(const LayoutLayer *layout_ref,
                                        const LayoutNodeConstructorConfig *config) {
  NotificationLayout *layout = (NotificationLayout *)layout_ref;
  if (!prv_image_aspect(layout)) {
    return NULL;
  }
  return &graphics_text_node_create_custom(prv_image_node_callback, layout)->node;
}

bool notification_layout_get_image_size(const LayoutLayer *layout_ref, GSize *size_out) {
  const NotificationLayout *layout = (const NotificationLayout *)layout_ref;
  if (!prv_image_aspect(layout)) {
    return false;
  }
  *size_out = prv_image_size(layout, DISP_COLS - 2 * CARD_MARGIN);
  return true;
}
#endif

//! Creates a GTextNode view node representing the inner content of the notification
//! @param layout NotificationLayout of the notification
//! @param use_body_icon Whether to display a body icon. Currently used by Jumboji
//! @return the GTextNode view node of the notification
static NOINLINE GTextNode *prv_create_view(NotificationLayout *layout, bool use_body_icon) {
  const NotificationStyle *style = &s_notification_styles[system_theme_get_content_size()];

  const bool is_reminder = prv_is_reminder(layout);
  const LayoutNodeTextAttributeConfig header_config = {
    .attr_id = AttributeIdAppName,
    .text.style_font = TextStyleFont_Header,
    .text.extent.offset.y = style->header_padding,
    .text.extent.margin.h = style->header_padding,
  };
  const LayoutNodeTextDynamicConfig notification_timestamp_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_notification_timestamp_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.style_font = PBL_IF_RECT_ELSE(TextStyleFont_Footer, TextStyleFont_Caption),
    .text.extent.offset.y = style->timestamp_upper_padding,
    .text.extent.margin.h = style->timestamp_upper_padding + style->timestamp_lower_padding,
  };
  const LayoutNodeTextDynamicConfig reminder_timestamp_config = {
    .text.extent.node.type = LayoutNodeType_TextDynamic,
    .update = prv_reminder_timestamp_update,
    .buffer_size = TIME_STRING_REQUIRED_LENGTH,
    .text.style_font = TextStyleFont_Header,
    .text.extent.offset.y = style->header_padding,
    .text.extent.margin.h = style->header_padding,
  };
  const LayoutNodeTextAttributeConfig title_config = {
    .attr_id = is_reminder ? AttributeIdUnused : AttributeIdTitle,
    .text.style_font = TextStyleFont_Header,
    .text.line_spacing_delta = style->title_line_delta,
    .text.alignment = use_body_icon ? LayoutTextAlignment_Center : LayoutTextAlignment_Auto,
    .text.extent.offset.y =
        style->title_padding + (use_body_icon ? style->title_offset_if_body_icon : 0),
    .text.extent.margin.h = style->title_padding,
  };
  const LayoutNodeTextAttributeConfig subtitle_config = {
    .attr_id = is_reminder ? AttributeIdTitle : AttributeIdSubtitle,
    .text.style_font = TextStyleFont_Title,
    .text.line_spacing_delta = style->subtitle_line_delta,
    .text.alignment = use_body_icon ? LayoutTextAlignment_Center : LayoutTextAlignment_Auto,
    .text.extent.offset.y = style->subtitle_upper_padding,
    .text.extent.margin.h = style->subtitle_upper_padding + style->subtitle_lower_padding,
  };
  const LayoutNodeIconConfig body_icon_config = {
    .extent.node.type = LayoutNodeType_Icon,
    .res_info = &(AppResourceInfo) {
      .res_app_num = SYSTEM_APP,
      .res_id = prv_get_emoji_icon(layout),
    },
    .align = GAlignCenter,
    .icon_layer = &layout->detail_icon_layer,
    .extent.offset.y = style->body_icon_offset,
    .extent.margin.h = style->body_icon_margin,
  };
  const LayoutNodeTextAttributeConfig location_config = {
    .attr_id = AttributeIdLocationName,
    .text.style_font = TextStyleFont_Footer,
    .text.extent.offset.y = style->location_offset,
    .text.extent.margin.h = style->location_margin,
  };
  const int reminder_body_line_delta = 0;
  const LayoutNodeTextAttributeConfig body_config = {
    .attr_id = AttributeIdBody,
    .text.style_font = is_reminder ? TextStyleFont_Caption : TextStyleFont_Body,
    .text.line_spacing_delta = is_reminder ? reminder_body_line_delta :
                                             style->body_line_delta,
    .text.extent.offset.y = style->body_padding,
    .text.extent.margin.h = style->body_padding,
  };
  const LayoutNodeHeadingsParagraphsConfig headings_paragraphs_node = {
    .extent.node.type = LayoutNodeType_HeadingsParagraphs,
    .extent.offset.y = 12,
    .extent.margin.h = 5,
    .heading_style_font = TextStyleFont_Header,
    .paragraph_style_font = TextStyleFont_Body,
  };
#if NOTIFICATION_IMAGE_SUPPORTED
  const LayoutNodeConstructorConfig image_config = {
    .extent.node.type = LayoutNodeType_Constructor,
    .constructor = prv_create_image_node,
    .extent.offset.y = NOTIFICATION_IMAGE_PADDING,
    .extent.margin.h = 2 * NOTIFICATION_IMAGE_PADDING,
  };
#endif
  const LayoutNodeConfig *reminder_timestamp_node_config = NULL;
  const LayoutNodeConfig *notification_timestamp_node_config = NULL;
  const LayoutNodeConfig *header_node_config = NULL;

  if (is_reminder) {
    reminder_timestamp_node_config = &reminder_timestamp_config.text.extent.node;
  } else {
    notification_timestamp_node_config = &notification_timestamp_config.text.extent.node;
    header_node_config = &header_config.text.extent.node;
  }
  // Only jumboji drops the timestamp: an enlarged emoji replaces it. Incoming (modal)
  // notifications keep it on round too, matching rect.
  if (!layout->info.show_notification_timestamp && use_body_icon) {
    notification_timestamp_node_config = NULL;
  }
  const LayoutNodeConfig * const vertical_config_nodes[] = {
    reminder_timestamp_node_config,
    header_node_config,
    &title_config.text.extent.node,
    &subtitle_config.text.extent.node,
    &location_config.text.extent.node,
    use_body_icon ? &body_icon_config.extent.node :
                    &body_config.text.extent.node,
    &headings_paragraphs_node.extent.node,
#if NOTIFICATION_IMAGE_SUPPORTED
    &image_config.extent.node,
#endif
    notification_timestamp_node_config,
  };
  const LayoutNodeVerticalConfig vertical_config = {
    .container.extent.node.type = LayoutNodeType_Vertical,
    .container.num_nodes = ARRAY_LENGTH(vertical_config_nodes),
    .container.nodes = (LayoutNodeConfig **)&vertical_config_nodes,
  };
  return layout_create_text_node_from_config(&layout->layout,
                                             &vertical_config.container.extent.node);
}

static void prv_destroy_view(NotificationLayout *layout) {
  graphics_text_node_destroy(layout->view_node);
  layout->view_node = NULL;
  kino_layer_destroy(layout->detail_icon_layer);
  layout->detail_icon_layer = NULL;
}

//! Do common init related tasks
static void prv_card_init(NotificationLayout *layout, AttributeList *attributes,
                          const Uuid *app_id) {
  // init the icon
  const TimelineResourceId fallback_icon_id =
      notification_layout_get_fallback_icon_id(layout->info.item->header.type);
  const uint32_t timeline_res_id = attribute_get_uint32(attributes, AttributeIdIconTiny,
                                                        fallback_icon_id);
  const TimelineResourceInfo timeline_res = {
    .res_id = timeline_res_id,
    .app_id = app_id,
    .fallback_id = fallback_icon_id
  };
  timeline_resources_get_id(&timeline_res, TimelineResourceSizeTiny, &layout->icon_res_info);

  const GRect *frame = &layout->layout.layer.frame;
  const GSize icon_size = NOTIFICATION_TINY_RESOURCE_SIZE;
  const int16_t origin_x = frame->origin.x + (frame->size.w / 2) - (icon_size.w / 2);
  const int16_t origin_y = frame->origin.y + CARD_ICON_UPPER_PADDING;
  kino_layer_init(&layout->icon_layer, &GRect(origin_x, origin_y, icon_size.w, icon_size.h));
#if PBL_BW
  const bool use_alternative_design = alerts_preferences_get_notification_alternative_design();
  kino_layer_set_reel_with_resource_system(&layout->icon_layer, layout->icon_res_info.res_app_num,
                                           layout->icon_res_info.res_id, use_alternative_design);
#else
  kino_layer_set_reel_with_resource_system(&layout->icon_layer, layout->icon_res_info.res_app_num,
                                           layout->icon_res_info.res_id, false);
#endif
  layer_add_child(&layout->layout.layer, kino_layer_get_layer(&layout->icon_layer));
}

static void NOINLINE prv_init_view(NotificationLayout *layout) {
  const bool use_body_icon = prv_should_enlarge_emoji(layout);
  layout->view_node = prv_create_view(layout, use_body_icon);

  if (use_body_icon) {
    // Only calculate size if using a body icon, calculating size is stack expensive
    prv_card_render(layout, graphics_context_get_current_context(), false /* render */);

    if ((layout->view_size.h > (LAYOUT_HEIGHT + LAYOUT_ARROW_HEIGHT))) {
      // The large emoji won't fit in a single screen, so don't use the large emoji
      prv_destroy_view(layout);
      layout->view_node = prv_create_view(layout, false /* use_body_icon */);
    } else {
    }
  }
}

#if PBL_ROUND
static void prv_hide_or_show_banner_icon(KinoLayer *icon_layer,
                                         const GRect *notification_layout_frame) {
  const int32_t frame_too_high_for_icon_threshold = -2;
  const int32_t top_banner_not_visible_threshold = 18;
  const bool icon_hidden =
      (notification_layout_frame->origin.y < frame_too_high_for_icon_threshold) ||
      (notification_layout_frame->origin.y > top_banner_not_visible_threshold);
  layer_set_hidden(&icon_layer->layer, icon_hidden);
}
#endif

#if PBL_ROUND
static CONST_FUNC int32_t prv_interpolate_linear(int32_t out_min, int32_t out_max, int32_t in_min,
                                                 int32_t in_max, int32_t progress) {
  return out_min + (out_max - out_min) * (progress - in_min) / (in_max - in_min);
}

// Effective notification status bar height: taller when the "Big & Bold" clock
// style is selected, otherwise the default. The content top is pushed down by
// this amount, so the banner clip math must match it.
static int16_t prv_notif_status_bar_height(void) {
  return (alerts_preferences_get_notification_status_bar_style() ==
          NotificationStatusBarStyle_LargeBold)
             ? STATUS_BAR_LAYER_LARGE_BOLD_HEIGHT
             : STATUS_BAR_LAYER_HEIGHT;
}

static void prv_draw_banner_round(NotificationLayout *notification_layout, GContext *ctx,
                                  const GRect *const notification_layout_frame,
                                  LayoutColors colors) {
  // We use DISP_ROWS and DISP_COLS instead of the layer's frame or bounds because the
  // notification layout's frame is not the same size as the display
  const int32_t half_screen_width = DISP_COLS / 2;
  const int16_t status_bar_height = prv_notif_status_bar_height();
  // The y-position of a layout frame when its banner is peeking
  const int32_t banner_peek_static_y = DISP_ROWS - status_bar_height;
  graphics_context_set_fill_color(ctx, colors.bg_color);
  const int32_t saved_clip_box_size_h = ctx->draw_state.clip_box.size.h;
  const int32_t saved_clip_box_origin_y = ctx->draw_state.clip_box.origin.y;
  ctx->draw_state.clip_box.origin.y =
      MAX(ctx->draw_state.clip_box.origin.y - status_bar_height, 0);
  ctx->draw_state.clip_box.size.h = DISP_ROWS;
  grect_clip(&ctx->draw_state.clip_box, &DISP_FRAME);

  const int32_t banner_movement_raw_offset =
      CLIP(banner_peek_static_y - notification_layout_frame->origin.y,
           0, banner_peek_static_y);
  const int32_t banner_radius = prv_interpolate_linear(BOTTOM_BANNER_CIRCLE_RADIUS,
                                                       BANNER_CIRCLE_RADIUS,
                                                       0, banner_peek_static_y,
                                                       banner_movement_raw_offset);
  const int32_t banner_diameter = banner_radius * 2;
  const int32_t banner_center_y = prv_interpolate_linear(0, LAYOUT_TOP_BANNER_ORIGIN_Y,
                                                         0, banner_peek_static_y,
                                                         banner_movement_raw_offset);
  const GRect banner_frame = GRect(half_screen_width - banner_radius,
                                   banner_center_y - banner_radius,
                                   banner_diameter, banner_diameter);
  graphics_fill_oval(ctx, banner_frame, GOvalScaleModeFitCircle);
  ctx->draw_state.clip_box.origin.y = saved_clip_box_origin_y;
  ctx->draw_state.clip_box.size.h = saved_clip_box_size_h;
}
#endif

static NOINLINE void prv_card_render_internal(NotificationLayout *layout, GContext *ctx,
                                              bool render) {
#if PBL_ROUND
  const int orig_clip_height = ctx->draw_state.clip_box.size.h;
  const GRect *notification_layout_frame = &layout->layout.layer.frame;
#endif

  // get layout colors and fill in the banner at the top
  if (render) {
    const LayoutColors *colors = prv_layout_get_colors((LayoutLayer *)layout);
    graphics_context_set_fill_color(ctx, colors->bg_color);

#if PBL_ROUND
    prv_hide_or_show_banner_icon(&layout->icon_layer, notification_layout_frame);
    prv_draw_banner_round(layout, ctx, notification_layout_frame, *colors);
    // work around the clip box and smaller layout_height for circular text paging
    ctx->draw_state.clip_box.size.h = MIN(ctx->draw_state.clip_box.size.h, LAYOUT_HEIGHT);
#else
    static const GRect banner_box = { .size = { DISP_COLS, LAYOUT_BANNER_HEIGHT_RECT } };
    graphics_fill_rect(ctx, &banner_box);
#endif
  }

#if PBL_ROUND
  const bool text_visible =
      (render && WITHIN(notification_layout_frame->origin.y,
                        TEXT_VISIBLE_LOWER_THRESHOLD(notification_layout_frame->size.h),
                        TEXT_VISIBLE_UPPER_THRESHOLD));
#else
  const bool text_visible = render;
#endif
  static const GRect box = {
    .origin = { CARD_MARGIN, LAYOUT_TOP_BANNER_HEIGHT },
    .size = { DISP_COLS - 2 * CARD_MARGIN, LAYOUT_MAX_HEIGHT },
  };
  static const GRect page_frame_on_screen = {
    .origin = { 0, STATUS_BAR_LAYER_HEIGHT },
    .size = { DISP_COLS, DISP_ROWS - STATUS_BAR_LAYER_HEIGHT - LAYOUT_ARROW_HEIGHT }
  };
  static const GTextNodeDrawConfig config = {
    .page_frame = &page_frame_on_screen,
    .origin_on_screen = &page_frame_on_screen.origin,
    .content_inset = 8, // text flow inset
    .text_flow = PBL_IF_ROUND_ELSE(true, false),
    .paging = PBL_IF_ROUND_ELSE(true, false),
  };
  graphics_context_set_text_color(ctx, GColorBlack);
  (text_visible ? graphics_text_node_draw :
                  graphics_text_node_get_size)(layout->view_node, ctx, &box, &config,
                                               &layout->view_size);

#if PBL_ROUND
  if (render) {
    // restore original clip box
    ctx->draw_state.clip_box.size.h = orig_clip_height;
  }
#endif

  layout->view_size.h += LAYOUT_TOP_BANNER_HEIGHT;

#if PBL_ROUND
  // Notification text is paged by LAYOUT_HEIGHT, so make full page height
  layout->view_size.h = ROUND_TO_MOD_CEIL(layout->view_size.h, LAYOUT_HEIGHT);
  // Notifications are swapped using frame height, so last page includes additional arrow height
  layout->view_size.h += LAYOUT_ARROW_HEIGHT;
#else
  layout->view_size.h += CARD_BOTTOM_PADDING;
#endif
}

static void prv_card_render(NotificationLayout *layout, GContext *ctx, bool render) {
  if (!layout->view_node) {
    prv_init_view(layout);
  }
  prv_card_render_internal(layout, ctx, render);
}

//////////////////////////////////////////
// LayoutLayer API
//////////////////////////////////////////

static void prv_layout_update_proc(Layer *layer, GContext *ctx) {
  NotificationLayout *layout = (NotificationLayout *)layer;
  switch (layout->layout.mode) {
    case LayoutLayerModeCard:
      prv_card_render(layout, ctx, true);
      break;
    default:
      break;
  }
}

static void prv_layout_init(NotificationLayout *layout, const LayoutLayerConfig *config);

LayoutLayer *notification_layout_create(const LayoutLayerConfig *config) {
  NotificationLayout *layout = task_zalloc_check(sizeof(NotificationLayout));
  if (!layout) {
    return NULL;
  }
  prv_layout_init(layout, config);
  return (LayoutLayer *)layout;
}

bool notification_layout_verify(bool existing_attributes[]) {
  return existing_attributes[AttributeIdTitle];
}

static void prv_layout_init_colors(NotificationLayout *notification_layout) {
  LayoutColors *colors = &notification_layout->colors;
  
#if PBL_BW
  const bool use_alternative_design = alerts_preferences_get_notification_alternative_design();
  if (use_alternative_design) {
    *colors = (LayoutColors){
        .primary_color = GColorWhite,
        .secondary_color = GColorBlack,
        .bg_color = GColorBlack,
    };
  } else {
    *colors = (LayoutColors){
        .primary_color = GColorBlack,
        .secondary_color = GColorBlack,
        .bg_color = GColorLightGray,
    };
  }
#else
  *colors = (LayoutColors){
      .primary_color = GColorBlack,
      .secondary_color = GColorBlack,
      .bg_color = GColorLightGray,
  };
#endif

#if PBL_COLOR
  const bool is_notification =
      (notification_layout->info.item->header.type == TimelineItemTypeNotification);
  const GColor default_bg_color = is_notification ? DEFAULT_NOTIFICATION_COLOR :
                                                    DEFAULT_REMINDER_COLOR;

  LayoutLayer *layout = &notification_layout->layout;
  colors->bg_color = (GColor) attribute_get_uint8(layout->attributes, AttributeIdBgColor,
                                                  default_bg_color.argb);
  colors->primary_color = (GColor) attribute_get_uint8(layout->attributes, AttributeIdPrimaryColor,
                                                       GColorBlack.argb);
#endif
}

static const LayoutColors *prv_layout_get_colors(const LayoutLayer *layout_ref) {
  return &((NotificationLayout *)layout_ref)->colors;
}

static void *prv_layout_get_context(LayoutLayer *layout) {
  NotificationLayout *notification_layout = (NotificationLayout *)layout;
  return (void *)notification_layout->info.item;
}

static GSize prv_layout_get_content_size(GContext *ctx, LayoutLayer *layout_ref) {
  NotificationLayout *layout = (NotificationLayout *)layout_ref;
  if (layout->view_size.h == 0) {
    prv_card_render(layout, graphics_context_get_current_context(), false);
  }
  return layout->view_size;
}

static void prv_layout_destroy(LayoutLayer *layout) {
  NotificationLayout *notification_layout = (NotificationLayout *)layout;
  prv_destroy_view(notification_layout);
  kino_layer_deinit(&notification_layout->icon_layer);
  // Every other layout deinits its base layer before freeing (see timeline_layout_deinit);
  // skipping it leaks attached recognizers and frees a layer the touch system may still
  // reference.
  layer_deinit(&notification_layout->layout.layer);
  task_free(notification_layout);
}

static void prv_layout_init(NotificationLayout *layout, const LayoutLayerConfig *config) {
  NotificationLayoutInfo *layout_info = config->context;
  static const LayoutLayerImpl s_layout_layer_impl = {
    .size_getter = prv_layout_get_content_size,
    .destructor = prv_layout_destroy,
#if PBL_COLOR
    .color_getter = prv_layout_get_colors,
#endif
    .context_getter = prv_layout_get_context,
  };
  // init the layout struct
  layout->layout = (LayoutLayer) {
    .mode = config->mode,
    .attributes = config->attributes,
    .impl = &s_layout_layer_impl,
  };
  layout->info = *layout_info;

  // init the layer in the layout
  layer_init(&layout->layout.layer, config->frame);
  layer_set_update_proc(&layout->layout.layer, prv_layout_update_proc);
#if PBL_ROUND
  layer_set_clips(&layout->layout.layer, false);
#endif

  prv_layout_init_colors(layout);

  switch (layout->layout.mode) {
    case LayoutLayerModeCard:
      prv_card_init(layout, config->attributes, config->app_id);
      break;
    default:
      break;
  }

  layer_mark_dirty(&(layout->layout.layer));
}

TimelineResourceId notification_layout_get_fallback_icon_id(TimelineItemType item_type) {
  return (item_type == TimelineItemTypeNotification) ? NOTIF_FALLBACK_ICON :
                                                       REMINDER_FALLBACK_ICON;
}
