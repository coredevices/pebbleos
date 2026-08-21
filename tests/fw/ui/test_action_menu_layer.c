/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"
#include "pebble_asserts.h"

#include "applib/ui/action_menu_layer.h"
#include "applib/ui/menu_layer.h"
#include "applib/ui/menu_layer_private.h"
#include "applib/ui/recognizer/recognizer.h"
#include "applib/ui/recognizer/recognizer_list.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/touch_nav.h"
#include "shell/system_theme.h"
#include "pbl/util/size.h"

#include "fake_rtc.h"
#include "pbl/drivers/rtc.h"

// Stubs
/////////////////////
#include "stubs_app_state.h"
#include "stubs_click.h"
#include "stubs_fonts.h"
#include "stubs_graphics.h"
#include "stubs_heap.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_tasks.h"
#include "stubs_process_manager.h"
#include "stubs_system_theme.h"
#include "stubs_ui_window.h"
#include "stubs_unobstructed_area.h"
#include "stubs_vibes.h"

// ---------------------------------------------------------------------------------------------
// Touch-navigation harness (CONFIG_TOUCH), mirroring test_menu_layer.c: menu_layer.c resolves the
// per-task touch-nav state through these accessors; the recognizer manager needs a few
// window/layer collaborators to link.

static bool s_nav_enabled = true;
bool sys_touch_nav_enabled(void) { return s_nav_enabled; }
bool sys_touch_app_nav_active(void) { return false; }

static TouchNavState s_touch_nav_state;
struct TouchNavState *app_state_get_touch_nav_state(void) { return &s_touch_nav_state; }
struct TouchNavState *modal_manager_get_touch_nav_state(void) { return &s_touch_nav_state; }

static Layer s_root_layer;
static RecognizerManager s_recognizer_manager;
static RecognizerList s_global_list;

struct Layer *window_get_root_layer(const Window *window) { return &s_root_layer; }
RecognizerList *window_get_recognizer_list(Window *window) { return NULL; }
RecognizerManager *window_get_recognizer_manager(Window *window) { return &s_recognizer_manager; }

static TouchNavOps s_bridge_ops;

static void prv_touch_nav_setup(void) {
  s_bridge_ops = (TouchNavOps){0};
  layer_init(&s_root_layer, &GRect(0, 0, 200, 400));
  recognizer_list_init(&s_global_list);
  recognizer_manager_init(&s_recognizer_manager);
  s_recognizer_manager.window = (Window *)&s_root_layer;  // non-NULL sentinel
  s_recognizer_manager.global_list = &s_global_list;
  touch_nav_state_init(&s_touch_nav_state, &s_recognizer_manager, &s_bridge_ops);
}

// Fakes / stubs for action_menu_layer.c dependencies
////////////////////////

static GContext s_gcontext;
GContext *graphics_context_get_current_context(void) { return &s_gcontext; }

GDrawState graphics_context_get_drawing_state(GContext *ctx) { return (GDrawState){}; }
void graphics_context_set_drawing_state(GContext *ctx, GDrawState draw_state) {}
void graphics_context_set_fill_color(GContext *ctx, GColor color) {}
void graphics_context_set_stroke_color(GContext *ctx, GColor color) {}
void graphics_context_set_text_color(GContext *ctx, GColor color) {}
void graphics_context_set_compositing_mode(GContext *ctx, GCompOp mode) {}
void graphics_draw_bitmap_in_rect(GContext *ctx, const GBitmap *bitmap, const GRect *rect) {}
void graphics_draw_horizontal_line_dotted(GContext *ctx, GPoint p, uint16_t length) {}

// Every item lays out as a single stub-font line.
uint16_t graphics_text_layout_get_text_height(GContext *ctx, const char *text, GFont const font,
                                              uint16_t bounds_width,
                                              const GTextOverflowMode overflow_mode,
                                              const GTextAlignment alignment) {
  return FONT_HEIGHT;
}

GSize graphics_text_layout_get_max_used_size(GContext *ctx, const char *text, GFont const font,
                                             const GRect box,
                                             const GTextOverflowMode overflow_mode,
                                             const GTextAlignment alignment,
                                             GTextLayoutCacheRef layout) {
  return GSize(10, FONT_HEIGHT);
}

GFont system_theme_get_font(TextStyleFont font) { return NULL; }

void menu_cell_basic_draw_custom(GContext *ctx, const Layer *cell_layer, GFont const title_font,
                                 const char *title, GFont const value_font, const char *value,
                                 GFont const subtitle_font, const char *subtitle, GBitmap *icon,
                                 bool icon_on_right, GTextOverflowMode overflow_mode) {}
int16_t menu_cell_basic_horizontal_inset(void) { return 8; }
int16_t menu_cell_small_cell_height(void) { return 24; }
int16_t menu_cell_basic_cell_height(void) { return 44; }

bool gbitmap_init_with_resource_system(GBitmap *bitmap, ResAppNum app_num, uint32_t resource_id) {
  return true;
}
void gbitmap_deinit(GBitmap *bitmap) {}

Layer *inverter_layer_get_layer(InverterLayer *inverter_layer) { return &inverter_layer->layer; }
void inverter_layer_init(InverterLayer *inverter, const GRect *frame) {}

void window_long_click_subscribe(ButtonId button_id, uint16_t delay_ms, ClickHandler down_handler,
                                 ClickHandler up_handler) {}
void window_single_click_subscribe(ButtonId button_id, ClickHandler handler) {}
void window_single_repeating_click_subscribe(ButtonId button_id, uint16_t repeat_interval_ms,
                                             ClickHandler handler) {}
void window_set_click_config_provider_with_context(Window *window,
                                                   ClickConfigProvider click_config_provider,
                                                   void *context) {}
void window_set_click_context(ButtonId button_id, void *context) {}

void content_indicator_destroy_for_scroll_layer(ScrollLayer *scroll_layer) {}

static ContentIndicator s_content_indicator;
ContentIndicator *content_indicator_get_for_scroll_layer(ScrollLayer *scroll_layer) {
  return &s_content_indicator;
}
ContentIndicator *content_indicator_get_or_create_for_scroll_layer(ScrollLayer *scroll_layer) {
  return &s_content_indicator;
}
void content_indicator_set_content_available(ContentIndicator *content_indicator,
                                             ContentIndicatorDirection direction, bool available) {}

// Observation
////////////////////////

static int s_select_count;
static const ActionMenuItem *s_last_selected_item;
static int s_selection_changed_count;
static const ActionMenuItem *s_last_changed_item;

static void prv_record_select(const ActionMenuItem *item, void *context) {
  s_select_count++;
  s_last_selected_item = item;
}

static void prv_record_selection_changed(const ActionMenuItem *item, void *context) {
  s_selection_changed_count++;
  s_last_changed_item = item;
}

static const ActionMenuItem s_wide_items[] = {
  { .label = "Dismiss", .is_leaf = 1 },
  { .label = "Reply", .is_leaf = 1 },
  { .label = "Mark as Read", .is_leaf = 1 },
};

static const ActionMenuItem s_short_items[] = {
  { .label = "A", .is_leaf = 1 },
  { .label = "B", .is_leaf = 1 },
  { .label = "C", .is_leaf = 1 },
  { .label = "D", .is_leaf = 1 },
  { .label = "E", .is_leaf = 1 },
};

static ActionMenuLayer s_aml;

// Return the tap point (in screen coordinates) whose hit-test resolves to \a row, independent of
// per-platform cell geometry.
static GPoint prv_tap_point_for_row(MenuLayer *ml, uint16_t row) {
  const int16_t offset_y = scroll_layer_get_content_offset(&ml->scroll_layer).y;
  for (int16_t y = 0; y < 1000; ++y) {
    MenuIndex idx;
    if (menu_layer_touch_find_row_at_content_y(ml, y, &idx) && idx.row == row) {
      // The first matching y is the row's top edge, which the half-open hit test owns.
      return GPoint(10, y + offset_y);
    }
  }
  cl_fail("row not found");
  return GPointZero;
}

static void prv_reset_counters(void) {
  s_select_count = 0;
  s_last_selected_item = NULL;
  s_selection_changed_count = 0;
  s_last_changed_item = NULL;
}

static void prv_init_aml_with_wide_items(void) {
  // action_menu_layer_init expects zeroed memory (the firmware always zallocs an AML).
  s_aml = (ActionMenuLayer){};
  action_menu_layer_init(&s_aml, &GRect(0, 0, 144, 168));
  action_menu_layer_set_callbacks(&s_aml, (ActionMenuLayerCallbacks){
    .select = prv_record_select,
    .selection_changed = prv_record_selection_changed,
  }, NULL);
  action_menu_layer_set_items(&s_aml, s_wide_items, ARRAY_LENGTH(s_wide_items), 0, 0);
  prv_reset_counters();
}

static void prv_advance_past_double_tap_window(void) {
  fake_rtc_increment_ticks((RtcTicks)400 * RTC_TICKS_HZ / 1000);
}

// Setup and Teardown
////////////////////////

void test_action_menu_layer__initialize(void) {
  fake_rtc_init(0, 0);
  prv_touch_nav_setup();
  prv_reset_counters();
}

void test_action_menu_layer__cleanup(void) {
}

// Tests
////////////////////////

// MOB-10624 regression: a tap on the selected action activates it through the AML select callback.
void test_action_menu_layer__tap_on_selected_item_activates(void) {
  prv_init_aml_with_wide_items();

  menu_layer_touch_handle_tap(&s_aml.menu_layer, prv_tap_point_for_row(&s_aml.menu_layer, 0));
  cl_assert_equal_i(s_select_count, 1);
  cl_assert(s_last_selected_item == &s_wide_items[0]);

  action_menu_layer_deinit(&s_aml);
}

// A wide-item row holds exactly one action, so a tap on a different action selects it and
// activates it in the same gesture (plain menus open on a single tap).
void test_action_menu_layer__tap_other_item_selects_and_activates(void) {
  prv_init_aml_with_wide_items();

  menu_layer_touch_handle_tap(&s_aml.menu_layer, prv_tap_point_for_row(&s_aml.menu_layer, 2));
  cl_assert_equal_i(s_aml.selected_index, 2);
  cl_assert_equal_i(s_selection_changed_count, 1);
  cl_assert(s_last_changed_item == &s_wide_items[2]);
  cl_assert_equal_i(s_select_count, 1);
  cl_assert(s_last_selected_item == &s_wide_items[2]);

  action_menu_layer_deinit(&s_aml);
}

// A tap onto a short-item (column) row adopts the row's first column, so a follow-up tap
// activates the item that is actually highlighted rather than a stale index.
void test_action_menu_layer__tap_short_row_adopts_first_column(void) {
  s_aml = (ActionMenuLayer){};
  action_menu_layer_init(&s_aml, &GRect(0, 0, 144, 168));
  action_menu_layer_set_callbacks(&s_aml, (ActionMenuLayerCallbacks){
    .select = prv_record_select,
    .selection_changed = prv_record_selection_changed,
  }, NULL);
  // 5 items in columns of 3: row 0 = items 0-2, row 1 = items 3-4.
  action_menu_layer_set_short_items(&s_aml, s_short_items, ARRAY_LENGTH(s_short_items), 0);
  prv_reset_counters();

  menu_layer_touch_handle_tap(&s_aml.menu_layer, prv_tap_point_for_row(&s_aml.menu_layer, 1));
  cl_assert_equal_i(s_select_count, 0);
  cl_assert_equal_i(s_aml.selected_index, 3);
  cl_assert_equal_i(s_selection_changed_count, 1);
  cl_assert(s_last_changed_item == &s_short_items[3]);

  prv_advance_past_double_tap_window();
  menu_layer_touch_handle_tap(&s_aml.menu_layer, prv_tap_point_for_row(&s_aml.menu_layer, 1));
  cl_assert_equal_i(s_select_count, 1);
  cl_assert(s_last_selected_item == &s_short_items[3]);

  action_menu_layer_deinit(&s_aml);
}
