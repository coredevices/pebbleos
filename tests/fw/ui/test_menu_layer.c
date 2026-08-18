/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"
#include "pebble_asserts.h"

#include "applib/ui/menu_layer.h"
#include "applib/ui/menu_layer_private.h"
#include "applib/ui/scroll_layer_private.h"
#include "applib/ui/content_indicator_private.h"
#include "applib/ui/recognizer/recognizer.h"
#include "applib/ui/recognizer/recognizer_list.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/touch_nav.h"

#include "applib/ui/animation_private.h"
#include "applib/ui/property_animation_private.h"

#include "fake_rtc.h"
#include "pbl/drivers/rtc.h"

// Stubs
/////////////////////
#include "stubs_app_state.h"
#include "stubs_click.h"
#include "stubs_graphics.h"
#include "stubs_heap.h"
#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_pebble_tasks.h"
#include "stubs_ui_window.h"
#include "stubs_process_manager.h"
#include "stubs_unobstructed_area.h"
#include "stubs_vibes.h"

// ---------------------------------------------------------------------------------------------
// Touch-navigation harness (CONFIG_TOUCH). menu_layer.c resolves the per-task touch-nav state
// through these accessors; the recognizer manager needs a few window/layer collaborators to link.

static bool s_nav_enabled = true;
bool sys_touch_nav_enabled(void) { return s_nav_enabled; }
bool sys_touch_app_nav_active(void) { return false; }

static TouchNavState s_touch_nav_state;
struct TouchNavState *app_state_get_touch_nav_state(void) { return &s_touch_nav_state; }
struct TouchNavState *modal_manager_get_touch_nav_state(void) { return &s_touch_nav_state; }

static Layer s_root_layer;               // window root, holds the menu under test while driving pans
static RecognizerManager s_recognizer_manager;
static RecognizerList s_global_list;

struct Layer *window_get_root_layer(const Window *window) { return &s_root_layer; }
RecognizerList *window_get_recognizer_list(Window *window) { return NULL; }
RecognizerManager *window_get_recognizer_manager(Window *window) { return &s_recognizer_manager; }

// Fake bridge ops so swipe-left BACK is observable.
typedef struct FakeBridgeOps {
  bool overrides_back;
  int pop_count;
  int emit_count;
  ButtonId last_emit;
} FakeBridgeOps;
static FakeBridgeOps s_bridge;
static bool prv_bridge_top_overrides_back(void *ctx) { return ((FakeBridgeOps *)ctx)->overrides_back; }
static void prv_bridge_pop_top(void *ctx) { ((FakeBridgeOps *)ctx)->pop_count++; }
static void prv_bridge_emit_button(void *ctx, ButtonId b) {
  FakeBridgeOps *o = ctx; o->emit_count++; o->last_emit = b;
}
static TouchNavOps s_bridge_ops;

// Bring up a real per-task touch-nav state so menu_layer_init() registers into a live registry.
static void prv_touch_nav_setup(void) {
  s_bridge = (FakeBridgeOps){0};
  s_bridge_ops = (TouchNavOps){
    .top_overrides_back = prv_bridge_top_overrides_back,
    .pop_top = prv_bridge_pop_top,
    .emit_button = prv_bridge_emit_button,
    .ctx = &s_bridge,
  };
  layer_init(&s_root_layer, &GRect(0, 0, 200, 400));
  recognizer_list_init(&s_global_list);
  recognizer_manager_init(&s_recognizer_manager);
  s_recognizer_manager.window = (Window *)&s_root_layer;  // non-NULL sentinel
  s_recognizer_manager.global_list = &s_global_list;
  touch_nav_state_init(&s_touch_nav_state, &s_recognizer_manager, &s_bridge_ops);
}


// Fakes
////////////////////////

//#include "fake_gbitmap_png.c"

GDrawState graphics_context_get_drawing_state(GContext* ctx) {
  return (GDrawState){};
}

void graphics_context_set_drawing_state(GContext* ctx, GDrawState draw_state) {}
void graphics_context_set_fill_color(GContext* ctx, GColor color){}

Layer* inverter_layer_get_layer(InverterLayer *inverter_layer) {
  return &inverter_layer->layer;
}

void inverter_layer_init(InverterLayer *inverter, const GRect *frame) {}

void window_long_click_subscribe(ButtonId button_id, uint16_t delay_ms,
                                 ClickHandler down_handler, ClickHandler up_handler) {}
void window_single_click_subscribe(ButtonId button_id, ClickHandler handler) {}
void window_single_repeating_click_subscribe(ButtonId button_id, uint16_t repeat_interval_ms,
                                             ClickHandler handler) {}
void window_set_click_config_provider_with_context(Window *window,
                                                   ClickConfigProvider click_config_provider,
                                                   void *context) {}
void window_set_click_context(ButtonId button_id, void *context) {}

void content_indicator_destroy_for_scroll_layer(ScrollLayer *scroll_layer) {}

ContentIndicator s_content_indicator;
ContentIndicator *content_indicator_get_for_scroll_layer(ScrollLayer *scroll_layer) {
  return &s_content_indicator;
}
ContentIndicator *content_indicator_get_or_create_for_scroll_layer(ScrollLayer *scroll_layer) {
  return &s_content_indicator;
}

static bool s_content_available[NumContentIndicatorDirections];
void content_indicator_set_content_available(ContentIndicator *content_indicator,
                                             ContentIndicatorDirection direction,
                                             bool available) {
  s_content_available[direction] = available;
}

void graphics_context_set_compositing_mode(GContext* ctx, GCompOp mode) {}
void graphics_draw_bitmap_in_rect(GContext *ctx, const GBitmap *bitmap, const GRect *rect){}

int16_t menu_cell_basic_cell_height(void) {
  return 44;
}

// Tests
//////////////////////


static uint16_t s_num_rows;

// Strong overrides of the WEAK animation stubs: capture the scroll animation's to-target and
// stopped handler so fling physics are observable and the coast end can be simulated by the test.
static GPoint s_anim_to;
static AnimationHandlers s_anim_handlers;
static void *s_anim_handlers_context;

bool property_animation_init(PropertyAnimation *animation,
                             const PropertyAnimationImplementation *implementation,
                             void *subject, void *from_value, void *to_value) {
  if (!animation) {
    return false;
  }
  *(PropertyAnimationPrivate *)animation = (PropertyAnimationPrivate) {
    .animation.implementation = (const AnimationImplementation *)implementation,
    .subject = subject,
  };
  if (to_value) {
    s_anim_to = *(GPoint *)to_value;   // every scroll animation targets a GPoint offset
  }
  return true;
}

bool animation_set_handlers(Animation *animation, AnimationHandlers callbacks, void *context) {
  if (!animation) {
    return false;
  }
  ((AnimationPrivate *)animation)->context = context;
  s_anim_handlers = callbacks;
  s_anim_handlers_context = context;
  return true;
}

void test_menu_layer__initialize(void) {
  s_num_rows = 10;
  fake_rtc_init(0, 0);
  s_anim_to = GPointZero;
  s_anim_handlers = (AnimationHandlers) { 0 };
  s_anim_handlers_context = NULL;
  s_nav_enabled = true;
  // A zeroed state has a NULL manager, so menu_layer_init() registration is inert for the tests
  // that do not opt into the touch-nav harness (prv_touch_nav_setup()).
  s_touch_nav_state = (TouchNavState){0};
  process_manager_set_compiled_with_legacy2_sdk(false);
  menu_layer_touch_nav_reset_all();
}

void test_menu_layer__cleanup(void) {
}

static void prv_draw_row(GContext* ctx,
                         const Layer *cell_layer,
                         MenuIndex *cell_index,
                         void *callback_context) {}

static uint16_t prv_get_num_rows(struct MenuLayer *menu_layer,
                                 uint16_t section_index,
                                 void *callback_context) {
  return s_num_rows;
}

void test_menu_layer__test_set_selected_classic(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(10, 10, 180, 180));
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks){
      .draw_row = prv_draw_row,
      .get_num_rows = prv_get_num_rows,
  });
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(0, l.selection.y);
  cl_assert_equal_i(0, scroll_layer_get_content_offset(&l.scroll_layer).y);

  menu_layer_set_selected_index(&l, MenuIndex(0, 1), MenuRowAlignTop, false);
  cl_assert_equal_i(1, menu_layer_get_selected_index(&l).row);
  const int16_t basic_cell_height = menu_cell_basic_cell_height();
  cl_assert_equal_i(basic_cell_height, l.selection.y);
  cl_assert_equal_i(-basic_cell_height,
                    scroll_layer_get_content_offset(&l.scroll_layer).y);
}

void test_menu_layer__test_set_selected_center_focused(void) {
  MenuLayer l;
  const int height = 180;
  menu_layer_init(&l, &GRect(10, 10, height, 180));
  menu_layer_set_center_focused(&l, true);
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks){
      .draw_row = prv_draw_row,
      .get_num_rows = prv_get_num_rows,
  });
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(0, l.selection.y);
  const int16_t basic_cell_height = menu_cell_basic_cell_height();
  const int row0_vertically_centered = (height - basic_cell_height)/2;
  cl_assert_equal_i(row0_vertically_centered, scroll_layer_get_content_offset(&l.scroll_layer).y);

  menu_layer_set_selected_index(&l, MenuIndex(0, 1), MenuRowAlignTop, false);
  cl_assert_equal_i(1, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(basic_cell_height, l.selection.y);

  const int y_center_of_row_1 = basic_cell_height + basic_cell_height / 2;
  const int row1_vertically_centered = height / 2 - y_center_of_row_1;
  cl_assert_equal_i(row1_vertically_centered, scroll_layer_get_content_offset(&l.scroll_layer).y);
}

void test_menu_layer__test_set_selection_animation(void) {
  MenuLayer l;
  const int height = 180;
  menu_layer_init(&l, &GRect(10, 10, height, 180));
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks){
    .draw_row = prv_draw_row,
    .get_num_rows = prv_get_num_rows,
  });
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(0, l.selection.y);

  // Test disabled first
  l.selection_animation_disabled = true;
  menu_layer_set_selected_index(&l, MenuIndex(0, 1), MenuRowAlignTop, true);
  cl_assert_equal_i(1, menu_layer_get_selected_index(&l).row);
  cl_assert(!l.animation.animation);

  // Test enabled
  l.selection_animation_disabled = false;
  menu_layer_set_selected_index(&l, MenuIndex(0, 0), MenuRowAlignTop, true);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
  cl_assert(l.animation.animation);
}

int16_t prv_get_row_height_depending_on_selection_state(struct MenuLayer *menu_layer,
                                                        MenuIndex *cell_index,
                                                        void *callback_context) {
  MenuIndex selected_index = menu_layer_get_selected_index(menu_layer);
  bool is_selected = menu_index_compare(&selected_index, cell_index) == 0;
  return is_selected ? MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT : menu_cell_basic_cell_height();
}

void test_menu_layer__default_ignores_row_height_for_selection(void) {
  MenuLayer l;
  const int height = 180;
  menu_layer_init(&l, &GRect(10, 10, height, 180));
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks){
      .draw_row = prv_draw_row,
      .get_num_rows = prv_get_num_rows,
      .get_cell_height = prv_get_row_height_depending_on_selection_state,
  });
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(0, l.selection.y);
  cl_assert_equal_i(0, scroll_layer_get_content_offset(&l.scroll_layer).y);
  cl_assert_equal_b(false, s_content_available[ContentIndicatorDirectionUp]);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionDown]);

  const int FOCUSED = MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT;
  const int NORMAL = menu_cell_basic_cell_height();

  cl_assert_equal_i(FOCUSED, l.selection.h);

  menu_layer_set_selected_index(&l, MenuIndex(0, 2), MenuRowAlignNone, false);

  cl_assert(menu_layer_get_center_focused(&l) == false);
  // non-center-focus behavior: don't ask adjust for changed height of row(0,0)
  cl_assert_equal_i(FOCUSED + 1 * NORMAL, l.selection.y);
  // also non-center-focus behavior: don't update selected_index before asking row (0,1) for height
  cl_assert_equal_i(NORMAL, l.selection.h);
  cl_assert_equal_b(false, s_content_available[ContentIndicatorDirectionUp]);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionDown]);

  // in general, the default behavior does not handle changes in row height correctly
  menu_layer_set_selected_next(&l, false, MenuRowAlignNone, false);
  cl_assert_equal_i(2 * FOCUSED + NORMAL, l.selection.y);
  cl_assert_equal_i(NORMAL, l.selection.h);
  cl_assert_equal_b(false, s_content_available[ContentIndicatorDirectionUp]);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionDown]);

  // totally wrong
  menu_layer_set_selected_next(&l, true, MenuRowAlignNone, false);
  cl_assert_equal_i(2 * FOCUSED, l.selection.y);
  cl_assert_equal_i(NORMAL, l.selection.h);
  cl_assert_equal_b(false, s_content_available[ContentIndicatorDirectionUp]);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionDown]);

  // WTF?!
  menu_layer_set_selected_index(&l, MenuIndex(0, 1), MenuRowAlignNone, false);
  cl_assert_equal_i(2 * FOCUSED - NORMAL, l.selection.y);
  cl_assert_equal_i(NORMAL, l.selection.h);
  cl_assert_equal_b(false, s_content_available[ContentIndicatorDirectionUp]);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionDown]);
}

void test_menu_layer__center_focused_respects_row_height_for_selection(void) {
  MenuLayer l;
  const int height = 180;
  menu_layer_init(&l, &GRect(10, 10, height, 180));
  menu_layer_set_center_focused(&l, true);
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks){
      .draw_row = prv_draw_row,
      .get_num_rows = prv_get_num_rows,
      .get_cell_height = prv_get_row_height_depending_on_selection_state,
  });

  const int FOCUSED = MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT;
  const int NORMAL = menu_cell_basic_cell_height();

  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(0, l.selection.y);
  const int row0_vertically_centered = (height - FOCUSED)/2;
  cl_assert_equal_i(row0_vertically_centered, scroll_layer_get_content_offset(&l.scroll_layer).y);
  cl_assert_equal_i(FOCUSED, l.selection.h);
  cl_assert_equal_b(false, s_content_available[ContentIndicatorDirectionUp]);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionDown]);

  menu_layer_set_selected_index(&l, MenuIndex(0, 2), MenuRowAlignNone, false);
  // new center-focus behavior: adjust for changed row sizes depending on focused row
  cl_assert(menu_layer_get_center_focused(&l) == true);
  cl_assert_equal_i(2 * NORMAL, l.selection.y);
  cl_assert_equal_i(NORMAL - FOCUSED, scroll_layer_get_content_offset(&l.scroll_layer).y);
  cl_assert_equal_i(FOCUSED, l.selection.h);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionUp]);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionDown]);

  menu_layer_set_selected_next(&l, false, MenuRowAlignNone, false);
  cl_assert_equal_i(3 * NORMAL, l.selection.y);
  cl_assert_equal_i(-FOCUSED, scroll_layer_get_content_offset(&l.scroll_layer).y);
  cl_assert_equal_i(FOCUSED, l.selection.h);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionUp]);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionDown]);

  menu_layer_set_selected_next(&l, true, MenuRowAlignNone, false);
  cl_assert_equal_i(2 * NORMAL, l.selection.y);
  cl_assert_equal_i(NORMAL - FOCUSED, scroll_layer_get_content_offset(&l.scroll_layer).y);
  cl_assert_equal_i(FOCUSED, l.selection.h);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionUp]);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionDown]);

  menu_layer_set_selected_index(&l, MenuIndex(0, 1), MenuRowAlignNone, false);
  cl_assert_equal_i(1 * NORMAL, l.selection.y);
  cl_assert_equal_i(2 * NORMAL - FOCUSED, scroll_layer_get_content_offset(&l.scroll_layer).y);
  cl_assert_equal_i(FOCUSED, l.selection.h);
  cl_assert_equal_b(false, s_content_available[ContentIndicatorDirectionUp]);
  cl_assert_equal_b(true, s_content_available[ContentIndicatorDirectionDown]);
}

static void prv_skip_odd_rows(struct MenuLayer *menu_layer,
                               MenuIndex *new_index,
                               MenuIndex old_index,
                               void *callback_context) {
  if (new_index->row == 1) {
    new_index->row = 2;
  }
  if (new_index->row == 3) {
    new_index->row = 4;
  }
}

void test_menu_layer__center_focused_handles_skipped_rows(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(10, 10, DISP_COLS, DISP_ROWS));
  menu_layer_set_center_focused(&l, true);
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks) {
    .draw_row = prv_draw_row,
    .get_num_rows = prv_get_num_rows,
    .selection_will_change = prv_skip_odd_rows,
  });
  menu_layer_reload_data(&l);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).section);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(0, l.selection.y);

  menu_layer_set_selected_next(&l, false, MenuRowAlignNone, false);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).section);
  cl_assert_equal_i(2, menu_layer_get_selected_index(&l).row);
  const int16_t basic_cell_height = menu_cell_basic_cell_height();
  cl_assert_equal_i(2 * basic_cell_height, l.selection.y);

  menu_layer_set_selected_next(&l, false, MenuRowAlignNone, false);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).section);
  cl_assert_equal_i(4, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(4 * basic_cell_height, l.selection.y);

  menu_layer_set_selected_next(&l, false, MenuRowAlignNone, false);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).section);
  cl_assert_equal_i(5, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(5 * basic_cell_height, l.selection.y);

  menu_layer_set_selected_next(&l, true, MenuRowAlignNone, false);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).section);
  cl_assert_equal_i(4, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(4 * basic_cell_height, l.selection.y);
}

// Not declared in menu_layer.h; normally reached via the window click config
extern void menu_up_click_handler(ClickRecognizerRef recognizer, MenuLayer *menu_layer);
extern void menu_down_click_handler(ClickRecognizerRef recognizer, MenuLayer *menu_layer);

static void prv_redirect_off_last_row(struct MenuLayer *menu_layer,
                                      MenuIndex *new_index,
                                      MenuIndex old_index,
                                      void *callback_context) {
  if (new_index->row == s_num_rows - 1) {
    new_index->row = s_num_rows - 2;
  }
}

void test_menu_layer__wrap_around_honors_selection_will_change(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(10, 10, DISP_COLS, DISP_ROWS));
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks) {
    .draw_row = prv_draw_row,
    .get_num_rows = prv_get_num_rows,
    .selection_will_change = prv_redirect_off_last_row,
  });
  menu_layer_set_scroll_wrap_around(&l, true);
  menu_layer_reload_data(&l);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);

  // Wrapping up from the first row must honor the redirect away from the
  // non-selectable last row
  menu_up_click_handler(NULL, &l);
  cl_assert_equal_i(s_num_rows - 2, menu_layer_get_selected_index(&l).row);

  // Scrolling down from there stays put: not at the true last index, so no
  // wrap, and normal scrolling is redirected back
  menu_down_click_handler(NULL, &l);
  cl_assert_equal_i(s_num_rows - 2, menu_layer_get_selected_index(&l).row);
}

static void prv_lock_selection(struct MenuLayer *menu_layer,
                               MenuIndex *new_index,
                               MenuIndex old_index,
                               void *callback_context) {
  *new_index = old_index;
}

void test_menu_layer__wrap_around_cancelled_when_selection_locked(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(10, 10, DISP_COLS, DISP_ROWS));
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks) {
    .draw_row = prv_draw_row,
    .get_num_rows = prv_get_num_rows,
    .selection_will_change = prv_lock_selection,
  });
  menu_layer_set_scroll_wrap_around(&l, true);
  menu_layer_reload_data(&l);

  menu_up_click_handler(NULL, &l);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
}

void test_menu_layer__center_focused_handles_skipped_rows_animated(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(10, 10, DISP_COLS, DISP_ROWS));
  menu_layer_set_center_focused(&l, true);
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks) {
    .draw_row = prv_draw_row,
    .get_num_rows = prv_get_num_rows,
    .selection_will_change = prv_skip_odd_rows,
  });
  menu_layer_reload_data(&l);
  const int16_t basic_cell_height = menu_cell_basic_cell_height();
  const int initial_scroll_offset = (DISP_ROWS - basic_cell_height) / 2;
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).section);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(0, l.selection.y);
  cl_assert_equal_i(initial_scroll_offset, l.scroll_layer.content_sublayer.bounds.origin.y);

  menu_layer_set_selected_next(&l, false, MenuRowAlignNone, true);
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).section);
  // these values are unchanged until the animation updates them
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(0 * basic_cell_height, l.selection.y);
  cl_assert_equal_i(initial_scroll_offset, l.scroll_layer.content_sublayer.bounds.origin.y);

  // in this test setup, we can directly cast an animation to AnimationPrivate
  AnimationPrivate *ap = (AnimationPrivate *) l.animation.animation;
  const AnimationImplementation *const impl = ap->implementation;
  impl->update(l.animation.animation, ANIMATION_NORMALIZED_MAX / 10);
  // still unchanged
  cl_assert_equal_i(0, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(0 * basic_cell_height, l.selection.y);
  cl_assert_equal_i(initial_scroll_offset, l.scroll_layer.content_sublayer.bounds.origin.y);

  // and updated
  impl->update(l.animation.animation, ANIMATION_NORMALIZED_MAX * 9 / 10);
  cl_assert_equal_i(2, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(2 * basic_cell_height, l.selection.y);
  cl_assert_equal_i(initial_scroll_offset - 2 * basic_cell_height,
                    l.scroll_layer.content_sublayer.bounds.origin.y);

  animation_unschedule(l.animation.animation);
  menu_layer_set_selected_next(&l, false, MenuRowAlignNone, true);
  // these values are unchanged until the animation updates them
  cl_assert_equal_i(2, menu_layer_get_selected_index(&l).row);
  cl_assert_equal_i(2 * basic_cell_height, l.selection.y);
  cl_assert_equal_i(initial_scroll_offset - 2 * basic_cell_height,
                    l.scroll_layer.content_sublayer.bounds.origin.y);
}

static MenuLayer s_menu_layer_hierarchy;

static void prv_menu_cell_is_part_of_hierarchy_draw_row(GContext* ctx,
                                                        const Layer *cell_layer,
                                                        MenuIndex *cell_index,
                                                        void *callback_context) {
  cl_assert_equal_p(cell_layer->window, s_menu_layer_hierarchy.scroll_layer.layer.window);
  cl_assert_equal_p(cell_layer->parent, &s_menu_layer_hierarchy.scroll_layer.content_sublayer);
  const GPoint actual = layer_convert_point_to_screen(cell_layer, GPointZero);
  const GPoint expected = layer_convert_point_to_screen(&s_menu_layer_hierarchy.scroll_layer.layer,
                                                        GPoint(0, cell_index->row * 44));
  cl_assert_equal_gpoint(actual, expected);
}

int prv_num_sublayers(const Layer *l) {
  int result = 0;
  Layer *child = l->first_child;
  while (l) {
    l = l->next_sibling;
    result++;
  }
  return result;
}

void test_menu_layer__menu_cell_is_part_of_hierarchy(void) {
  menu_layer_init(&s_menu_layer_hierarchy, &GRect(10, 10, 100, 180));
  Layer *layer = &s_menu_layer_hierarchy.scroll_layer.content_sublayer;
  // two layers (inverter + shadow)
  cl_assert_equal_i(2, prv_num_sublayers(layer));
  menu_layer_set_callbacks(&s_menu_layer_hierarchy, NULL, &(MenuLayerCallbacks){
    .draw_row = prv_menu_cell_is_part_of_hierarchy_draw_row,
    .get_num_rows = prv_get_num_rows,
  });
  menu_layer_reload_data(&s_menu_layer_hierarchy);
  GContext ctx = {};
  cl_assert_equal_i(2, prv_num_sublayers(layer));
  layer->update_proc(layer, &ctx);
  cl_assert_equal_i(2, prv_num_sublayers(layer));
}

void test_menu_layer__center_focused_updates_height_on_reload(void) {
  MenuLayer l;
  const int height = DISP_ROWS;
  menu_layer_init(&l, &GRect(10, 10, height, DISP_COLS));
  menu_layer_set_center_focused(&l, true);
  s_num_rows = 3;
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks) {
    .draw_row = prv_draw_row,
    .get_num_rows = prv_get_num_rows,
    .get_cell_height = prv_get_row_height_depending_on_selection_state,
  });
  menu_layer_set_center_focused(&l, true);
  menu_layer_reload_data(&l);
  const int focused_height = MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT;

  // focus last row
  menu_layer_set_selected_index(&l, MenuIndex(0, s_num_rows - 1), MenuRowAlignNone, false);
  cl_assert_equal_i(focused_height, l.selection.h);

  s_num_rows--;
  cl_assert_equal_i(2, s_num_rows);
  menu_layer_reload_data(&l);
  cl_assert_equal_i(s_num_rows - 1, l.selection.index.row);
  cl_assert_equal_i(focused_height, l.selection.h);

  s_num_rows--;
  cl_assert_equal_i(1, s_num_rows);
  menu_layer_reload_data(&l);
  cl_assert_equal_i(s_num_rows - 1, l.selection.index.row);
  cl_assert_equal_i(focused_height, l.selection.h);
}

// =============================================================================================
// Tier-1 touch navigation
// =============================================================================================

// ---- Shared callbacks / observation for the touch handlers ----

typedef enum {
  WillChange_Passthrough,
  WillChange_Veto,
  WillChange_Redirect,
} WillChangeMode;

static int s_will_change_count;
static MenuIndex s_will_change_candidate;   // value of *new_index observed on entry
static MenuIndex s_will_change_old;
static int s_selection_changed_count;
static int s_select_click_count;
static MenuIndex s_select_click_index;
static WillChangeMode s_will_change_mode;
static MenuIndex s_will_change_redirect;

static void prv_reset_touch_counters(void) {
  s_will_change_count = 0;
  s_selection_changed_count = 0;
  s_select_click_count = 0;
  s_will_change_mode = WillChange_Passthrough;
}

static void prv_touch_will_change(struct MenuLayer *m, MenuIndex *new_index, MenuIndex old_index,
                                  void *ctx) {
  s_will_change_count++;
  s_will_change_candidate = *new_index;
  s_will_change_old = old_index;
  switch (s_will_change_mode) {
    case WillChange_Veto:     *new_index = old_index; break;
    case WillChange_Redirect: *new_index = s_will_change_redirect; break;
    case WillChange_Passthrough: break;
  }
}

static void prv_touch_selection_changed(struct MenuLayer *m, MenuIndex new_index,
                                        MenuIndex old_index, void *ctx) {
  s_selection_changed_count++;
}

static void prv_touch_select_click(struct MenuLayer *m, MenuIndex *index, void *ctx) {
  s_select_click_count++;
  s_select_click_index = *index;
}

static void prv_set_touch_callbacks(MenuLayer *l) {
  menu_layer_set_callbacks(l, NULL, &(MenuLayerCallbacks){
    .draw_row = prv_draw_row,
    .get_num_rows = prv_get_num_rows,
    .selection_will_change = prv_touch_will_change,
    .selection_changed = prv_touch_selection_changed,
    .select_click = prv_touch_select_click,
  });
}

// ---- Criterion 1: hit-test with section headers ----

static uint16_t prv_two_sections(struct MenuLayer *m, void *ctx) { return 2; }
static uint16_t prv_five_rows(struct MenuLayer *m, uint16_t s, void *ctx) { return 5; }
static int16_t prv_header_20(struct MenuLayer *m, uint16_t s, void *ctx) { return 20; }
static int16_t prv_sep_0(struct MenuLayer *m, MenuIndex *i, void *ctx) { return 0; }

void test_menu_layer__touch_hit_test_with_headers(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 168));
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks){
    .draw_row = prv_draw_row,
    .get_num_rows = prv_five_rows,
    .get_num_sections = prv_two_sections,
    .get_header_height = prv_header_20,
    .draw_header = (MenuLayerDrawHeaderCallback)prv_draw_row,
    .get_separator_height = prv_sep_0,
  });
  menu_layer_reload_data(&l);

  // Use the real per-row geometry: focus a row in section 1 (past a header) and hit-test its centre.
  menu_layer_set_selected_index(&l, MenuIndex(1, 2), MenuRowAlignNone, false);
  const int16_t row_y = l.selection.y + l.selection.h / 2;
  MenuIndex hit;
  cl_assert(menu_layer_touch_find_row_at_content_y(&l, row_y, &hit));
  cl_assert_equal_i(hit.section, 1);
  cl_assert_equal_i(hit.row, 2);

  // A y inside the section-1 header (just above the first row of section 1) is not a selectable row.
  menu_layer_set_selected_index(&l, MenuIndex(1, 0), MenuRowAlignNone, false);
  const int16_t header_y = l.selection.y - 10;  // within the 20px header above row (1, 0)
  MenuIndex miss;
  cl_assert(!menu_layer_touch_find_row_at_content_y(&l, header_y, &miss));

  // The row boundary is half-open [top, bottom): a hit exactly on a row's bottom edge belongs to
  // the NEXT row, not the current one (pins the < vs <= at the boundary).
  menu_layer_set_selected_index(&l, MenuIndex(1, 2), MenuRowAlignNone, false);
  const int16_t edge_y = l.selection.y + l.selection.h;  // exact bottom of row (1, 2) == top of (1, 3)
  MenuIndex edge;
  cl_assert(menu_layer_touch_find_row_at_content_y(&l, edge_y, &edge));
  cl_assert_equal_i(edge.section, 1);
  cl_assert_equal_i(edge.row, 3);
}

// The screen->content conversion accounts for the scroll offset (as under a status bar).
void test_menu_layer__touch_tap_accounts_for_scroll_offset(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  // Scroll down two rows, then tap near the top of the frame: content_y = 22 - (-88) = 110 => row 2.
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, -88), false);
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 22));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 2);
}

// A menu inset below the status bar: the tap point is in screen space, so it must be mapped through
// the scroll layer's global frame. A tap at screen y = 56 on a menu whose frame starts at y = 16 is
// frame-relative y = 40 -> row 0 (44px rows). Without the global-frame conversion content_y would be
// 56 -> row 1, i.e. the bottom of every row would activate the next row down.
void test_menu_layer__touch_tap_below_status_bar(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 16, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 56));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 0);
}

// ---- Criterion 2: on a plain menu, a pan (Liftoff) settles the offset only and NEVER moves the
// selection (center-focused carousels track the centre instead — see the carousel section) ----

// Scrolling down then lifting off must leave the selection index exactly where it was (it may even
// scroll off-screen) and must not run the selection contract or activate anything.
void test_menu_layer__touch_snap_leaves_selection_unchanged(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 1), MenuRowAlignNone, false);
  const MenuIndex before = menu_layer_get_selected_index(&l);
  prv_reset_touch_counters();
  // A downward pan of -88 (two rows) then Liftoff: offset settles, selection stays on row 1.
  menu_layer_touch_handle_snap(&l, GPoint(0, 0), GPoint(0, -88), GPointZero);
  const MenuIndex after = menu_layer_get_selected_index(&l);
  cl_assert_equal_i(after.section, before.section);
  cl_assert_equal_i(after.row, before.row);                    // selection unchanged
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, -88);  // offset settled
  cl_assert_equal_i(s_will_change_count, 0);                   // no selection contract on a pan
  cl_assert_equal_i(s_selection_changed_count, 0);
  cl_assert_equal_i(s_select_click_count, 0);                  // no activation
}

// The same for an upward pan (scroll back toward the top): selection is still frozen.
void test_menu_layer__touch_snap_up_leaves_selection_unchanged(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 4), MenuRowAlignCenter, false);
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, -176), false);
  const MenuIndex before = menu_layer_get_selected_index(&l);
  prv_reset_touch_counters();
  menu_layer_touch_handle_snap(&l, GPoint(0, -176), GPoint(0, 88), GPointZero);  // pan up two rows, Liftoff
  const MenuIndex after = menu_layer_get_selected_index(&l);
  cl_assert_equal_i(after.row, before.row);                    // selection unchanged
  cl_assert_equal_i(s_will_change_count, 0);
  cl_assert_equal_i(s_select_click_count, 0);
}

// ---- Center-focused (carousel): the focus stays pinned at the viewport centre ----

// A pan on a center-focused menu moves the row crossing the centre into the selection, live,
// through the will_change contract. The offset keeps following the finger — no re-centring
// happens until liftoff.
void test_menu_layer__touch_pan_center_focused_tracks_center_row(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  const int16_t base = scroll_layer_get_content_offset(&l.scroll_layer).y;  // row 0 centred
  prv_reset_touch_counters();
  // Drag up two (uniform 44px) rows: row 2 crosses the centre.
  menu_layer_touch_handle_pan_update(&l, GPoint(0, base), GPoint(0, -88));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 2);   // focus tracked the centre
  cl_assert_equal_i(s_will_change_count, 1);                     // through the contract
  cl_assert_equal_i(s_selection_changed_count, 1);
  cl_assert_equal_i(s_select_click_count, 0);                    // a pan never activates
  // The finger owns the offset: no re-centring mid-pan.
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, base - 88);
}

// A veto keeps the old row focused; the content still scrolls under it.
void test_menu_layer__touch_pan_center_focused_veto_keeps_selection(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  const int16_t base = scroll_layer_get_content_offset(&l.scroll_layer).y;
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Veto;
  menu_layer_touch_handle_pan_update(&l, GPoint(0, base), GPoint(0, -88));
  cl_assert_equal_i(s_will_change_count, 1);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 0);   // vetoed: focus stays
  cl_assert_equal_i(s_selection_changed_count, 0);
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, base - 88);  // still scrolled
}

// Liftoff adopts the row under the centre and glides it to the exact centre (this replaces the
// former "#27 free scroll everywhere" behaviour for center-focused menus).
void test_menu_layer__touch_snap_center_focused_settles_center_row(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  const int16_t base = scroll_layer_get_content_offset(&l.scroll_layer).y;
  prv_reset_touch_counters();
  // -100 leaves the centre inside row 2 but off its exact centre, so a settle is required (a
  // two-row drag of -88 would land dead-centre and hide a missing settle).
  menu_layer_touch_handle_snap(&l, GPoint(0, base), GPoint(0, -100), GPointZero);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 2);   // nearest row adopted
  cl_assert_equal_i(s_will_change_count, 1);
  cl_assert_equal_i(s_select_click_count, 0);                    // snapping never activates
  // The settle glide is animated. The stubbed property animation cannot be driven to its target
  // here, so assert it was scheduled; the exact-centre target math is pinned by the cancel test,
  // which runs the same settle synchronously.
  Animation *settle = property_animation_get_animation(l.scroll_layer.animation);
  cl_assert(settle != NULL);
  cl_assert(animation_is_scheduled(settle));
}

// A cancelled pan on a carousel re-centres synchronously (no animation to outlive the handover).
void test_menu_layer__touch_cancel_center_focused_recenters(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  const int16_t base = scroll_layer_get_content_offset(&l.scroll_layer).y;
  prv_reset_touch_counters();
  menu_layer_touch_handle_pan_update(&l, GPoint(0, base), GPoint(0, -100));  // row 2, off-grid
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 2);
  menu_layer_touch_handle_cancel(&l);
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y,
                    90 - l.selection.y - l.selection.h / 2);     // immediately centred
  cl_assert_equal_i(s_select_click_count, 0);
}

// Selection-dependent cell heights (the round launcher configuration): crossing a boundary
// reflows the rows — the newly focused row inflates toward the centre — and the selection span
// must reflect the new layout while the offset keeps following the finger.
void test_menu_layer__touch_pan_center_focused_tracks_with_focus_heights(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks){
    .draw_row = prv_draw_row,
    .get_num_rows = prv_get_num_rows,
    .get_cell_height = prv_get_row_height_depending_on_selection_state,
    .selection_will_change = prv_touch_will_change,
    .selection_changed = prv_touch_selection_changed,
    .select_click = prv_touch_select_click,
  });
  menu_layer_reload_data(&l);
  const int FOCUSED = MENU_CELL_ROUND_FOCUSED_TALL_CELL_HEIGHT;
  const int NORMAL = menu_cell_basic_cell_height();
  const int16_t base = scroll_layer_get_content_offset(&l.scroll_layer).y;
  prv_reset_touch_counters();
  // Aim the centre at the exact middle of row 1 under the OLD layout (row 1 spans
  // [FOCUSED, FOCUSED + NORMAL) while row 0 is still focused), robust to the constants' parity.
  const int16_t target_offset = (int16_t)(90 - FOCUSED - NORMAL / 2);
  menu_layer_touch_handle_pan_update(&l, GPoint(0, base), GPoint(0, target_offset - base));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 1);
  cl_assert_equal_i(l.selection.y, NORMAL);        // row 0 deflated above it
  cl_assert_equal_i(l.selection.h, FOCUSED);       // the focused row inflated
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, target_offset);
}

// ---- Criterion 3: tap selects, a second tap on the selected row activates ----

// A tap on a row that is NOT the current selection only selects it (through the will_change
// contract) and centres it — it must NOT activate. Activation is the second tap.
void test_menu_layer__touch_tap_unselected_selects_no_activate(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Passthrough;
  // Default selection is row 0; tap row 2 (unselected).
  menu_layer_touch_handle_tap(&l, GPoint(72, 2 * 44 + 22));  // row 2, offset 0
  cl_assert_equal_i(s_will_change_count, 1);                 // the full contract ran
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 2);  // selection moved to the tapped row
  cl_assert_equal_i(s_select_click_count, 0);               // but it did NOT activate
}

// A tap on the row that IS the current selection activates it, without re-running the contract.
void test_menu_layer__touch_tap_selected_activates(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 2), MenuRowAlignNone, false);  // select, no scroll
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 2 * 44 + 22));  // tap the already-selected row 2
  cl_assert_equal_i(s_select_click_count, 1);               // activated
  cl_assert_equal_i(s_select_click_index.row, 2);
  cl_assert_equal_i(s_will_change_count, 0);                // no reselection contract on the selected row
}

// Animation robustness: the "tapped == selection" check keys off the committed selection INDEX, not
// the on-screen position. A selection can be committed to a row that is still off-centre (mid-flight
// toward the centre); a tap landing on it must still ACTIVATE, so a fast double tap in one spot
// launches the row even though it has not finished animating to the centre.
void test_menu_layer__touch_tap_selected_offcentre_activates(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  // Commit selection to row 3 but leave it OFF-CENTRE (screen centre is 90; row 3 sits at ~154).
  menu_layer_set_selected_index(&l, MenuIndex(0, 3), MenuRowAlignNone, false);
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 3 * 44 + 22));  // tap where row 3 currently sits
  cl_assert_equal_i(s_select_click_count, 1);               // activated despite not being centred
  cl_assert_equal_i(s_select_click_index.row, 3);
  cl_assert_equal_i(s_will_change_count, 0);
}

void test_menu_layer__touch_tap_veto_no_select_no_activate(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Veto;
  menu_layer_touch_handle_tap(&l, GPoint(72, 2 * 44 + 22));  // taps row 2, but veto keeps row 0
  cl_assert_equal_i(s_will_change_count, 1);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 0);  // no selection change
  cl_assert_equal_i(s_select_click_count, 0);                   // no activation
}

void test_menu_layer__touch_tap_redirect_selects_without_activating(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Redirect;
  s_will_change_redirect = MenuIndex(0, 5);
  menu_layer_touch_handle_tap(&l, GPoint(72, 2 * 44 + 22));  // taps row 2, redirected to row 5
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 5);  // selected the third row
  cl_assert_equal_i(s_select_click_count, 0);                   // but did NOT activate
}

// A tap-select on a center-focused menu plays the button-style jump animation instead of
// re-centring synchronously: the index commit is deferred to the jump's half-way point, while the
// double-tap window records the jump's target immediately.
void test_menu_layer__touch_tap_center_focused_schedules_jump(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 1), MenuRowAlignCenter, false);  // offset 24
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 154 + 24));  // row 3 (content_y 154), on-screen at 178
  cl_assert_equal_i(s_will_change_count, 1);
  cl_assert_equal_i(s_select_click_count, 0);                  // select, not activate
  // The jump is in flight: index not yet committed, target recorded for the double-tap window.
  cl_assert(l.animation.animation != NULL);
  cl_assert(animation_is_scheduled(l.animation.animation));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 1);
  cl_assert_equal_i(l.animation.new_selection.index.row, 3);
  cl_assert_equal_i(l.last_selected_index.row, 3);
  // Driving the jump to its end commits the selection and centres the target row.
  AnimationPrivate *jump = (AnimationPrivate *)l.animation.animation;
  jump->implementation->update(l.animation.animation, ANIMATION_NORMALIZED_MAX);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 3);
  cl_assert_equal_i(s_selection_changed_count, 1);
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y,
                    90 - l.selection.y - l.selection.h / 2);
}

// A fast second tap while the jump is still in flight must activate the jump's TARGET. The commit
// is still pending (the index is the old row), so this pins that the double-tap window records
// animation.new_selection and not the stale selection.index.
void test_menu_layer__touch_double_tap_during_center_focused_jump(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 110 + 68));  // row 2 (content_y 110), row 0 centred
  cl_assert_equal_i(s_select_click_count, 0);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 0);  // commit still pending
  fake_rtc_increment_ticks((RtcTicks)100 * RTC_TICKS_HZ / 1000);  // within the 300ms window
  menu_layer_touch_handle_tap(&l, GPoint(72, 110 + 68));  // same spot, mid-jump
  cl_assert_equal_i(s_select_click_count, 1);
  cl_assert_equal_i(s_select_click_index.row, 2);          // the jump target, not the stale row 0
}

// ---- Criterion 3a: double-tap window (fast double tap in one spot on an animated menu) ----

// The case the naive index compare CANNOT handle: on an animated (non-center_focused) menu, tap 1
// selects a row and it starts sliding to the centre; tap 2 in the SAME screen spot now hit-tests a
// DIFFERENT row, yet must ACTIVATE the row tap 1 selected (via the double-tap window), not re-select.
void test_menu_layer__touch_double_tap_same_spot_activates(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));  // non-center_focused => animated selection
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Passthrough;

  // Tap 1: selects row 3 and arms the window. No activation yet.
  menu_layer_touch_handle_tap(&l, GPoint(72, 3 * 44 + 22));  // screen y 154, offset 0 => row 3
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 3);
  cl_assert_equal_i(s_select_click_count, 0);

  // Simulate row 3 sliding toward the centre: scroll so the SAME screen point sits on another row.
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, -66), false);
  MenuIndex probe;  // content_y = 154 - (-66) = 220 => row 5, a DIFFERENT row than the selected 3
  cl_assert(menu_layer_touch_find_row_at_content_y(&l, 154 + 66, &probe));
  cl_assert(probe.row != 3);

  // Tap 2 in the same spot, well within the 300ms window: activates the RECORDED row 3, not the
  // neighbour the tap now hit-tests, and runs no new will_change contract.
  fake_rtc_increment_ticks((RtcTicks)100 * RTC_TICKS_HZ / 1000);  // +100ms < 300ms
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 3 * 44 + 22));
  cl_assert_equal_i(s_select_click_count, 1);
  cl_assert_equal_i(s_select_click_index.row, 3);   // the selected row, NOT the hit-tested neighbour
  cl_assert_equal_i(s_will_change_count, 0);
}

// A second tap AFTER the window (> 300ms) is a fresh single tap: it selects the newly tapped row and
// does not activate the earlier one.
void test_menu_layer__touch_tap_after_window_selects(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Passthrough;

  menu_layer_touch_handle_tap(&l, GPoint(72, 1 * 44 + 22));  // tap row 1: selects + arms window
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 1);
  cl_assert_equal_i(s_select_click_count, 0);

  // Let the window lapse, then tap a different visible row from a clean offset.
  fake_rtc_increment_ticks((RtcTicks)400 * RTC_TICKS_HZ / 1000);  // +400ms > 300ms
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, 0), false);
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 3 * 44 + 22));  // row 3
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 3);  // selected the new row
  cl_assert_equal_i(s_select_click_count, 0);                   // did NOT activate the old one
}

void test_menu_layer__touch_double_tap_records_clamped_selection(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();

  // Tap 1: will_change redirects to an out-of-range row; menu_layer_set_selected_index clamps it to
  // the last valid row. The recorded double-tap index must be that CLAMPED selection, not the raw
  // out-of-range will_change output.
  s_will_change_mode = WillChange_Redirect;
  s_will_change_redirect = MenuIndex(0, 999);
  menu_layer_touch_handle_tap(&l, GPoint(72, 2 * 44 + 22));  // taps row 2, redirected out of range
  const uint16_t clamped_row = menu_layer_get_selected_index(&l).row;
  cl_assert(clamped_row < 999);                 // proved clamped to a valid row
  cl_assert_equal_i(s_select_click_count, 0);   // selected, not activated

  // Tap 2 inside the window: priority 1 activates the RECORDED index, which must be the clamped row,
  // never the out-of-range 999 a client redirect asked for.
  fake_rtc_increment_ticks((RtcTicks)100 * RTC_TICKS_HZ / 1000);  // +100ms < 300ms
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 2 * 44 + 22));
  cl_assert_equal_i(s_select_click_count, 1);
  cl_assert_equal_i(s_select_click_index.row, clamped_row);  // clamped, NOT the out-of-range 999
}

// Fix B: a vetoed tap must change NOTHING — in particular it must not re-centre the (off-centre) old
// selection. A vetoed tap on a visible other row must not call set_selected_index at all, so the
// scroll offset stays exactly where it was and nothing activates.
//
// A center_focused menu is used so a (buggy) re-centre would be observable synchronously: even the
// animated jump applies the scroll position at schedule time (only the content-offset trick
// animates), so a wrong set_selected_index call would move the offset before this test's assert.
// The veto early-return branch exercised here is the same one used by ordinary menus.
void test_menu_layer__touch_tap_veto_does_not_recenter(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  // Commit selection to row 3 (centred), then a free scroll pushes it off-centre.
  menu_layer_set_selected_index(&l, MenuIndex(0, 3), MenuRowAlignCenter, false);
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, -20), false);
  const int16_t offset_before = scroll_layer_get_content_offset(&l.scroll_layer).y;
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Veto;
  // Tap at screen y 22 => content_y 42 => row 0 (a visible row that is not the selected row 3).
  menu_layer_touch_handle_tap(&l, GPoint(72, 22));
  cl_assert_equal_i(s_will_change_count, 1);                    // the contract ran
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 3);  // veto kept the old selection
  cl_assert_equal_i(s_select_click_count, 0);                   // no activation
  // Offset unchanged: a re-centre would have snapped row 3 back to the middle (a different offset).
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, offset_before);
}

// A2: a non-tap selection move (button nav) between two taps disarms the double-tap window, so the
// second in-window tap SELECTS the freshly hit-tested row instead of activating the stale recorded
// one. Without the disarm at the selection choke point, this would wrongly activate the old row.
void test_menu_layer__touch_button_nav_disarms_double_tap(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Passthrough;

  // Tap row 3: selects it and arms the window (no activation yet).
  menu_layer_touch_handle_tap(&l, GPoint(72, 3 * 44 + 22));  // content_y 154 => row 3
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 3);
  cl_assert_equal_i(s_select_click_count, 0);

  // Button DOWN moves the selection through the non-tap path -> must disarm the window.
  menu_layer_set_selected_next(&l, false, MenuRowAlignCenter, false);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 4);

  // A tap within the 300ms window at the ORIGINAL screen point: the stale arm is gone, so it
  // hit-tests row 3 and SELECTS it instead of activating the recorded row.
  fake_rtc_increment_ticks((RtcTicks)100 * RTC_TICKS_HZ / 1000);  // +100ms < 300ms
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, 0), false);
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 3 * 44 + 22));  // content_y 154 => row 3
  cl_assert_equal_i(s_select_click_count, 0);                   // SELECTED, not activated
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 3);  // the newly hit-tested row
}

// A2: menu_layer_reload_data disarms the window, so a reload that deletes the recorded row cannot let
// a follow-up in-window tap activate a now out-of-range index (client OOB on select_click).
void test_menu_layer__touch_reload_disarms_double_tap(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Passthrough;

  // Tap row 5: selects it and arms the window.
  menu_layer_touch_handle_tap(&l, GPoint(72, 5 * 44 + 22));  // content_y 242 => row 5
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 5);
  cl_assert_equal_i(s_select_click_count, 0);

  // Reload shrinking the list so the recorded row 5 no longer exists must disarm the window.
  s_num_rows = 3;
  menu_layer_reload_data(&l);

  // A tap within the window must NOT activate the stale (now out-of-range) row 5.
  fake_rtc_increment_ticks((RtcTicks)100 * RTC_TICKS_HZ / 1000);  // +100ms < 300ms
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, 0), false);
  prv_reset_touch_counters();
  menu_layer_touch_handle_tap(&l, GPoint(72, 1 * 44 + 22));  // content_y 66 => row 1
  cl_assert_equal_i(s_select_click_count, 0);                 // no stale activation
}

// ---- Criterion 4: on a plain menu the selection is frozen during a pan; no client callbacks ----

void test_menu_layer__touch_pan_freezes_selection(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 3), MenuRowAlignNone, false);
  prv_reset_touch_counters();
  menu_layer_touch_handle_pan_update(&l, GPoint(0, 0), GPoint(0, -100));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 3);            // unchanged
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, -100);  // content moved
  cl_assert_equal_i(s_will_change_count, 0);
  cl_assert_equal_i(s_selection_changed_count, 0);
  cl_assert_equal_i(s_select_click_count, 0);
}

// ---- Criterion 5: offset stays within coarse bounds (short content + center_focused widening) ----

void test_menu_layer__touch_clamp_content_shorter_than_viewport(void) {
  MenuLayer l;
  s_num_rows = 1;  // content shorter than the 180px viewport => lower bound is 0 via min()
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_touch_handle_pan_update(&l, GPoint(0, 0), GPoint(0, -100));
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, 0);
  menu_layer_touch_handle_pan_update(&l, GPoint(0, 0), GPoint(0, 100));
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, 0);
}

void test_menu_layer__touch_clamp_center_focused_widens(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  // center_focused widens the clamp by frame_h/2 (=90), so a positive offset up to +90 is allowed
  // where a normal menu would clamp at 0.
  menu_layer_touch_handle_pan_update(&l, GPoint(0, 0), GPoint(0, 300));
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, 90);

  // The widen is symmetric: it also lowers the min bound by frame_h/2. A large negative pan reaches
  // min(frame_h - content_h, 0) - frame_h/2; dropping the min_y widen would stop it at the un-widened
  // min, so this pins the lower-bound widen the +90 case alone leaves untested.
  const int16_t frame_h = l.scroll_layer.layer.frame.size.h;
  const int16_t content_h = scroll_layer_get_content_size(&l.scroll_layer).h;
  const int16_t expected_min = MIN((int16_t)(frame_h - content_h), (int16_t)0) - (int16_t)(frame_h / 2);
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, 0), false);
  menu_layer_touch_handle_pan_update(&l, GPoint(0, 0), GPoint(0, -3000));
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, expected_min);
}

// ---- Criterion 6: a cancelled pan leaves the selection alone and fires no client callback ----

void test_menu_layer__touch_cancel_leaves_selection_unchanged(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 1), MenuRowAlignNone, false);
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, -20), false);  // centre over row 2
  prv_reset_touch_counters();
  menu_layer_touch_handle_cancel(&l);
  cl_assert_equal_i(s_will_change_count, 0);                     // no contract call
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 1);  // selection untouched by the cancel
  cl_assert_equal_i(s_selection_changed_count, 0);
  cl_assert_equal_i(s_select_click_count, 0);
}

// ---- Swipes: swapped horizontal convention (right = BACK, left = no-op) ----

void test_menu_layer__touch_swipe_right_emits_back(void) {
  prv_touch_nav_setup();
  s_bridge.overrides_back = false;   // no back handler => the bridge pops the window
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  menu_layer_touch_handle_swipe(&l, SwipeDirection_Right);
  cl_assert_equal_i(s_bridge.pop_count, 1);       // right emits BACK
  cl_assert_equal_i(s_bridge.emit_count, 0);
  cl_assert_equal_i(s_select_click_count, 0);     // and does NOT activate
  menu_layer_deinit(&l);
}

void test_menu_layer__touch_swipe_left_does_nothing(void) {
  prv_touch_nav_setup();
  s_bridge.overrides_back = false;
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 3), MenuRowAlignNone, false);
  prv_reset_touch_counters();
  menu_layer_touch_handle_swipe(&l, SwipeDirection_Left);
  cl_assert_equal_i(s_select_click_count, 0);     // left no longer activates
  cl_assert_equal_i(s_bridge.emit_count, 0);      // and emits nothing
  cl_assert_equal_i(s_bridge.pop_count, 0);       // and does NOT pop / go BACK
  menu_layer_deinit(&l);
}

// ---- Criterion 8: legacy-2.x menus are not registered; normal menus are ----

void test_menu_layer__touch_legacy2_not_registered(void) {
  prv_touch_nav_setup();
  process_manager_set_compiled_with_legacy2_sdk(true);
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  cl_assert(s_touch_nav_state.menu_head == NULL);  // not a Tier-1 widget => falls back to the bridge
  menu_layer_deinit(&l);
}

void test_menu_layer__touch_registered_and_deregistered(void) {
  prv_touch_nav_setup();
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  cl_assert(s_touch_nav_state.menu_head != NULL);
  menu_layer_deinit(&l);
  cl_assert(s_touch_nav_state.menu_head == NULL);
}

// ---- Criterion 8a: init->init without deinit routes; double deinit is a no-op ----

void test_menu_layer__touch_double_init_and_double_deinit(void) {
  prv_touch_nav_setup();
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  // Re-init the same menu without deinit: the registry stays a single entry (dedup by address).
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  cl_assert(s_touch_nav_state.menu_head != NULL);
  cl_assert(s_touch_nav_state.menu_head->next == NULL);
  // The re-init zeroed the node while it was still registered; the registry must restore its layer
  // so a gesture still ROUTES to this menu (routing matches by node->layer, which the earlier
  // direct-handler assertion never exercised).
  cl_assert_equal_p(s_touch_nav_state.menu_head->layer, menu_layer_get_layer(&l));
  // And the handler still drives the scroll.
  menu_layer_touch_handle_pan_update(&l, GPoint(0, 0), GPoint(0, -44));
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, -44);
  // First deinit removes it from the registry and drains the shared recognizer set; the second
  // touch-nav deregistration is a safe no-op (no animation is pending, so the base deinit is too).
  menu_layer_deinit(&l);
  cl_assert(s_touch_nav_state.menu_head == NULL);
  menu_layer_deinit(&l);
  cl_assert(s_touch_nav_state.menu_head == NULL);
}

// ---- Criterion 7: deinit mid-gesture cancels with no callbacks; the next gesture still works ----

typedef struct { TouchEventType type; int16_t x; int16_t y; } DriveEvent;

static void prv_drive(TouchEventType type, int16_t x, int16_t y) {
  const TouchEvent e = { .type = type, .x = x, .y = y, .non_navigational = false };
  touch_nav_dispatch(&e, &s_touch_nav_state);
}

static void prv_advance_ms(uint32_t ms) {
  fake_rtc_increment_ticks((RtcTicks)ms * RTC_TICKS_HZ / 1000);
}

void test_menu_layer__touch_deinit_mid_gesture_cancels(void) {
  prv_touch_nav_setup();
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 200, 300));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  layer_add_child(&s_root_layer, menu_layer_get_layer(&l));

  // Drive a vertical pan to Started through the dispatcher so the Tier-1 route latches and the menu
  // becomes the gesture target.
  prv_drive(TouchEvent_Touchdown, 100, 150);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 110);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 70);
  cl_assert(menu_layer_touch_is_gesture_target(&l));

  // Destroy the widget under a live window: the gesture is cancelled with NO client callbacks.
  prv_reset_touch_counters();
  menu_layer_deinit(&l);
  cl_assert_equal_i(s_will_change_count, 0);
  cl_assert_equal_i(s_selection_changed_count, 0);
  cl_assert(!menu_layer_touch_is_gesture_target(&l));

  // The next menu + touch still works (routing recovered).
  MenuLayer l2;
  menu_layer_init(&l2, &GRect(0, 0, 200, 300));
  prv_set_touch_callbacks(&l2);
  menu_layer_reload_data(&l2);
  layer_add_child(&s_root_layer, menu_layer_get_layer(&l2));
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Passthrough;
  // A tap on the unselected row 2 selects it (routing recovered); activation would be a second tap.
  menu_layer_touch_handle_tap(&l2, GPoint(100, 2 * 44 + 22));
  cl_assert_equal_i(menu_layer_get_selected_index(&l2).row, 2);
  cl_assert_equal_i(s_will_change_count, 1);
  menu_layer_deinit(&l2);
}

// ---- Recognizer-set arbitration: the per-type touch filter scopes each set to its type ----
//
// A MenuLayer and an independent ScrollLayer registered on the SAME task share one global
// recognizer
// list, so their pan recognizers would otherwise compete: whichever crosses the pan threshold first
// triggers and FAILS the other, regardless of which widget the finger is on. Registering the menu
// first puts its pan ahead of the scroll pan on the list, the ordering under which the pre-fix bug
// has the menu pan win a scroll gesture (resolving no menu target, a no-op) while the scroll pan is
// failed -> the ScrollLayer scrolls by NEITHER finger nor the old bridge. The per-type touch filter
// fixes this by gating each set on the gesture's latched active layer.

// Bring up a menu (registered first) and an independent scroll layer on disjoint regions of one
// root,
// so the recognizer manager's active-layer hit-test resolves a touch to exactly one of them.
static void prv_make_menu_and_scroll(MenuLayer *menu, ScrollLayer *scroll) {
  menu_layer_init(menu, &GRect(0, 0, 200, 180));
  prv_set_touch_callbacks(menu);
  menu_layer_reload_data(menu);
  layer_add_child(&s_root_layer, menu_layer_get_layer(menu));

  scroll_layer_init(scroll, &GRect(0, 200, 200, 180));
  scroll_layer_set_content_size(scroll, GSize(200, 600));
  layer_add_child(&s_root_layer, scroll_layer_get_layer(scroll));

  cl_assert(s_touch_nav_state.menu_head != NULL);
  cl_assert(s_touch_nav_state.scroll_head != NULL);
}

// The key regression: a pan whose active layer is the ScrollLayer scrolls it 1:1 and is NOT
// eaten by
// the menu set. (Fails on the pre-filter code; passes with the filter.)
void test_menu_layer__scroll_gesture_not_eaten_by_menu(void) {
  prv_touch_nav_setup();
  MenuLayer menu;
  ScrollLayer scroll;
  prv_make_menu_and_scroll(&menu, &scroll);

  prv_drive(TouchEvent_Touchdown, 100, 300);         // active layer -> the standalone ScrollLayer
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 260);    // 40px up -> pan Started on the scroll set
  cl_assert(scroll_layer_touch_is_gesture_target(&scroll));
  cl_assert(!menu_layer_touch_is_gesture_target(&menu));
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 230);    // Updated -> live scroll
  prv_drive(TouchEvent_Liftoff, 100, 230);           // Completed -> final commit

  cl_assert(scroll_layer_get_content_offset(&scroll).y < 0);            // scrolled by finger
  cl_assert_equal_i(scroll_layer_get_content_offset(&menu.scroll_layer).y, 0);  // menu inert

  scroll_layer_deinit(&scroll);
  menu_layer_deinit(&menu);
}

// Symmetric: a pan whose active layer is the MenuLayer scrolls the menu; the scroll set bows out.
void test_menu_layer__menu_gesture_not_eaten_by_scroll(void) {
  prv_touch_nav_setup();
  MenuLayer menu;
  ScrollLayer scroll;
  prv_make_menu_and_scroll(&menu, &scroll);

  prv_drive(TouchEvent_Touchdown, 100, 90);          // active layer -> the MenuLayer
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 50);     // 40px up -> pan Started on the menu set
  cl_assert(menu_layer_touch_is_gesture_target(&menu));
  cl_assert(!scroll_layer_touch_is_gesture_target(&scroll));
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 20);     // Updated -> live scroll
  prv_drive(TouchEvent_Liftoff, 100, 20);            // Completed -> final commit

  cl_assert(scroll_layer_get_content_offset(&menu.scroll_layer).y < 0);   // menu scrolled
  cl_assert_equal_i(scroll_layer_get_content_offset(&scroll).y, 0);       // scroll set inert

  scroll_layer_deinit(&scroll);
  menu_layer_deinit(&menu);
}

// ---- П4: UP/DOWN reconciliation after a free pixel scroll ----

// After a free scroll pushes the selection OFF-screen, DOWN must step from the row nearest the
// viewport centre (what the user actually sees), not from the off-screen selection. Otherwise the
// selection teleports across the list.
void test_menu_layer__step_reconciles_offscreen_selection(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));  // 10 rows * 44 = 440 content, max scroll -260
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 0), MenuRowAlignNone, false);
  prv_reset_touch_counters();

  // Free-scroll to the bottom: selection row 0 (content 0..44) is now well above the viewport.
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, -260), false);
  // Viewport-centre content_y = 90 - (-260) = 350 => row 7 (308..352).
  MenuIndex center;
  cl_assert(menu_layer_touch_find_row_at_content_y(&l, 350, &center));
  cl_assert_equal_i(center.row, 7);

  menu_down_click_handler(NULL, &l);
  // Stepped from the CENTRE row 7 -> row 8, NOT selection+1 (row 1) of the off-screen row 0.
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 8);
  cl_assert(menu_layer_get_selected_index(&l).row != 1);
  cl_assert_equal_i(s_select_click_count, 0);  // reselect never activates
}

// The mandatory gate: when the selection is VISIBLE, UP/DOWN behaves exactly as a non-touch build
// (steps from the real selection). No reconcile, no pull-to-centre.
void test_menu_layer__step_visible_selection_unchanged(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 0), MenuRowAlignNone, false);
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, 0), false);
  prv_reset_touch_counters();

  // Selection row 0 is on screen; the viewport centre is row 2 (88..132). If the gate wrongly fired
  // it would reselect row 2 and step to row 3. With the gate, DOWN simply steps 0 -> 1.
  menu_down_click_handler(NULL, &l);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 1);
  cl_assert_equal_i(s_select_click_count, 0);
}

// Off-screen selection but the viewport centre lands on a section header/gap: reconcile must fall
// back gracefully (step from the current selection as today), with no crash.
void test_menu_layer__step_offscreen_center_on_header_falls_back(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 168));
  menu_layer_set_callbacks(&l, NULL, &(MenuLayerCallbacks){
    .draw_row = prv_draw_row,
    .get_num_rows = prv_five_rows,
    .get_num_sections = prv_two_sections,
    .get_header_height = prv_header_20,
    .draw_header = (MenuLayerDrawHeaderCallback)prv_draw_row,
    .get_separator_height = prv_sep_0,
    .select_click = prv_touch_select_click,
  });
  menu_layer_reload_data(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 0), MenuRowAlignNone, false);
  prv_reset_touch_counters();

  // Section-1 header spans content [240, 260). Offset -160 puts the viewport centre at
  // content_y = 84 - (-160) = 244 (inside that header) while selection (0,0) at content [20,64) is
  // off-screen (viewport shows content [160, 328]).
  scroll_layer_set_content_offset(&l.scroll_layer, GPoint(0, -160), false);
  MenuIndex miss;
  cl_assert(!menu_layer_touch_find_row_at_content_y(&l, 244, &miss));  // centre is a header

  menu_down_click_handler(NULL, &l);
  // No reconcile (centre was a header): step from the current selection (0,0) -> (0,1).
  cl_assert_equal_i(menu_layer_get_selected_index(&l).section, 0);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 1);
  cl_assert_equal_i(s_select_click_count, 0);
}

// =============================================================================================
// Unified widget set: gestures driven THROUGH touch_nav_dispatch (not the apply functions).
// These exercise the Ф1 vtable path end to end: Touchdown latches the menu as the migrated widget
// target, and the unified (tap, pan, swipe) set drives it via s_menu_touch_nav_ops.

// A quick, stationary tap through the dispatcher (Touchdown then Liftoff, no movement).
static void prv_menu_dispatch_tap(int16_t x, int16_t y) {
  prv_drive(TouchEvent_Touchdown, x, y);
  prv_advance_ms(30);
  prv_drive(TouchEvent_Liftoff, x, y);
}

static MenuLayer *prv_dispatch_menu_setup(MenuLayer *l) {
  menu_layer_init(l, &GRect(0, 0, 200, 300));
  prv_set_touch_callbacks(l);
  menu_layer_reload_data(l);
  layer_add_child(&s_root_layer, menu_layer_get_layer(l));
  return l;
}

// ---- Tap through the dispatcher honours the full selection_will_change contract ----

// (a) Passthrough: a tap on an unselected row runs the contract and selects it, without activating.
void test_menu_layer__dispatch_tap_passthrough_selects(void) {
  prv_touch_nav_setup();
  MenuLayer l;
  prv_dispatch_menu_setup(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Passthrough;
  prv_menu_dispatch_tap(100, 2 * 44 + 22);  // row 2 (offset 0)
  cl_assert_equal_i(s_will_change_count, 1);                     // the contract ran through dispatch
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 2);   // selection moved to the tapped row
  cl_assert_equal_i(s_select_click_count, 0);                    // but did NOT activate
  menu_layer_deinit(&l);
}

// (b) Veto: a tap the client vetoes changes nothing (selection stays, no activate).
void test_menu_layer__dispatch_tap_veto_changes_nothing(void) {
  prv_touch_nav_setup();
  MenuLayer l;
  prv_dispatch_menu_setup(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Veto;
  prv_menu_dispatch_tap(100, 2 * 44 + 22);  // taps row 2, veto keeps the default row 0
  cl_assert_equal_i(s_will_change_count, 1);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 0);   // veto kept the old selection
  cl_assert_equal_i(s_select_click_count, 0);
  menu_layer_deinit(&l);
}

// (c) Redirect: a tap the client redirects selects the redirected row, without activating.
void test_menu_layer__dispatch_tap_redirect_selects_target(void) {
  prv_touch_nav_setup();
  MenuLayer l;
  prv_dispatch_menu_setup(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Redirect;
  s_will_change_redirect = MenuIndex(0, 5);
  prv_menu_dispatch_tap(100, 2 * 44 + 22);  // taps row 2, redirected to row 5
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 5);   // selected the redirect target
  cl_assert_equal_i(s_select_click_count, 0);
  menu_layer_deinit(&l);
}

// ---- Pan through the dispatcher scrolls 1:1 and snaps on liftoff, selection frozen ----

void test_menu_layer__dispatch_pan_scrolls_1to1_and_snaps(void) {
  prv_touch_nav_setup();
  MenuLayer l;
  prv_dispatch_menu_setup(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 2), MenuRowAlignNone, false);
  prv_reset_touch_counters();

  // Touchdown, then a first update 40px up crosses the pan threshold: the pan Starts and only latches
  // the base (offset unchanged). delta_since_start is re-anchored to (0,0) at Start.
  prv_drive(TouchEvent_Touchdown, 100, 200);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 160);   // pan Started at y=160, base latched
  cl_assert(menu_layer_touch_is_gesture_target(&l));
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, 0);

  // A second update 30px further up live-scrolls the content 1:1 (base 0 + delta -30).
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 130);
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, -30);

  // Liftoff snaps to the final (unthrottled) offset, still -30, and the selection never moved.
  prv_drive(TouchEvent_Liftoff, 100, 130);
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, -30);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 2);   // selection frozen for the pan
  cl_assert_equal_i(s_will_change_count, 0);
  cl_assert_equal_i(s_select_click_count, 0);
  cl_assert(!menu_layer_touch_is_gesture_target(&l));            // latch cleared on completion
  menu_layer_deinit(&l);
}

// A pan pre-empted mid-gesture (a fresh navigational Touchdown) is dropped cleanly: it never moves
// the selection or fires a client callback. (The pan recognizer delivers no Cancelled event on a
// reset -- see pan.c prv_cancel -- so the widget's cancel handler does not run; the point here is
// that the pre-emption is side-effect free, exactly as before the refactor.)
void test_menu_layer__dispatch_pan_cancel_is_clean(void) {
  prv_touch_nav_setup();
  MenuLayer l;
  prv_dispatch_menu_setup(&l);
  menu_layer_set_selected_index(&l, MenuIndex(0, 2), MenuRowAlignNone, false);

  prv_drive(TouchEvent_Touchdown, 100, 200);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 160);   // pan Started
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 130);   // live scroll
  cl_assert(menu_layer_touch_is_gesture_target(&l));
  cl_assert(scroll_layer_get_content_offset(&l.scroll_layer).y < 0);
  prv_reset_touch_counters();

  // A new navigational Touchdown pre-empts the in-flight pan: the manager resets and unwinds the pan
  // with no snap. The selection is untouched and no client callback fires.
  prv_drive(TouchEvent_Touchdown, 100, 200);
  cl_assert_equal_i(s_will_change_count, 0);
  cl_assert_equal_i(s_selection_changed_count, 0);
  cl_assert_equal_i(s_select_click_count, 0);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 2);   // selection survived the cancel
  menu_layer_deinit(&l);
}

// A fast vertical flick scrolls/flings the menu: the unified swipe mask is Left|Right, so a vertical
// gesture is a pan, never a swipe, and emits no navigation button.
void test_menu_layer__dispatch_vertical_flick_scrolls_not_swipe(void) {
  prv_touch_nav_setup();
  s_bridge.overrides_back = false;
  MenuLayer l;
  prv_dispatch_menu_setup(&l);
  prv_reset_touch_counters();

  prv_drive(TouchEvent_Touchdown, 100, 220);
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 140);   // fast upward move -> pan Started
  prv_advance_ms(20);
  prv_drive(TouchEvent_PositionUpdate, 100, 80);    // live scroll
  prv_drive(TouchEvent_Liftoff, 100, 80);
  cl_assert(scroll_layer_get_content_offset(&l.scroll_layer).y < 0);   // scrolled by the flick
  cl_assert_equal_i(s_bridge.emit_count, 0);                           // NOT emitted as a swipe
  cl_assert_equal_i(s_bridge.pop_count, 0);
  cl_assert_equal_i(s_select_click_count, 0);
  menu_layer_deinit(&l);
}

// A no-trigger gesture (a dead-zone Touchdown with no widget under the finger) leaves the manager
// idle: the unified set is failed at the latch, so no filtered-out recognizer lingers Possible.
void test_menu_layer__dispatch_no_trigger_leaves_manager_idle(void) {
  prv_touch_nav_setup();
  // Status-bar dead zone, no registered widget -> route Dropped: the bridge set is failed AND the
  // unified set is failed at latch (the resolver found no migrated widget).
  prv_drive(TouchEvent_Touchdown, 100, 4);
  cl_assert_equal_i(s_touch_nav_state.route, TouchNavRoute_Dropped);
  cl_assert_equal_i(recognizer_get_state(s_touch_nav_state.widget_tap), RecognizerState_Failed);
  cl_assert_equal_i(recognizer_get_state(s_touch_nav_state.widget_pan), RecognizerState_Failed);
  cl_assert_equal_i(recognizer_get_state(s_touch_nav_state.widget_swipe), RecognizerState_Failed);

  // Liftoff with nothing triggered returns the manager to idle: no stuck Possible recognizer (wart).
  prv_drive(TouchEvent_Liftoff, 100, 4);
  cl_assert_equal_i(s_recognizer_manager.state, RecognizerManagerState_WaitForTouchdown);
}

// =============================================================================================
// Inertial fling (plain menus): a fast liftoff coasts the content; the selection never moves.
// Menu geometry here: 10 rows x 44px + 20px bottom padding = 460 content, 180 frame ->
// clamp range [-280, 0]. The stubbed animation never moves the content, so offset assertions see
// the released value while the captured to-target/duration pin the physics.

void test_menu_layer__touch_fling_plain_coasts_content_only(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  const MenuIndex before = menu_layer_get_selected_index(&l);
  prv_reset_touch_counters();
  // The drag left the content at -88; projection = -88 + (-500 * 240 / 1000) = -208, inside
  // [-280, 0] -> the unclamped coast runs the full 3*TAU duration.
  menu_layer_touch_handle_pan_update(&l, GPoint(0, 0), GPoint(0, -88));
  menu_layer_touch_handle_snap(&l, GPoint(0, 0), GPoint(0, -88), GPoint(0, -500));
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, -88);  // no liftoff jump
  cl_assert(l.touch_fling_active);
  Animation *anim = property_animation_get_animation(l.scroll_layer.animation);
  cl_assert(anim != NULL);
  cl_assert(animation_is_scheduled(anim));
  cl_assert_equal_i(s_anim_to.y, -208);
  cl_assert_equal_i(animation_get_duration(anim, false, false), TOUCH_FLING_MAX_DURATION_MS);
  // The selection contract is untouched: a coast is still "a pan scrolls, a tap selects".
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, before.row);
  cl_assert_equal_i(s_will_change_count, 0);
  cl_assert_equal_i(s_selection_changed_count, 0);
  cl_assert_equal_i(s_select_click_count, 0);
}

// The coast target is clamped by the menu clamp; the truncated distance shortens the duration
// (with the floor applied), so the coast does not crawl into the edge.
void test_menu_layer__touch_fling_plain_clamps_target(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  // The drag left the content at -88; upward coast projection = -88 + 720 = 632 -> clamped to 0;
  // d = 88 -> raw T = 3000*88/3000 = 88 -> floored to MIN_DURATION.
  menu_layer_touch_handle_pan_update(&l, GPoint(0, 0), GPoint(0, -88));
  menu_layer_touch_handle_snap(&l, GPoint(0, 0), GPoint(0, -88), GPoint(0, 3000));
  cl_assert(l.touch_fling_active);
  Animation *anim = property_animation_get_animation(l.scroll_layer.animation);
  cl_assert(anim != NULL);
  cl_assert(animation_is_scheduled(anim));
  cl_assert_equal_i(s_anim_to.y, 0);
  cl_assert_equal_i(animation_get_duration(anim, false, false), TOUCH_FLING_MIN_DURATION_MS);
}

// Below the fling threshold a liftoff behaves exactly as before: settle only, nothing scheduled.
void test_menu_layer__touch_fling_below_threshold_settles_only(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  menu_layer_touch_handle_snap(&l, GPoint(0, 0), GPoint(0, -88),
                               GPoint(0, -(TOUCH_FLING_MIN_VELOCITY_PX_S - 1)));
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, -88);
  cl_assert(!l.touch_fling_active);
  cl_assert(l.scroll_layer.animation == NULL);   // never animated: no coast was scheduled
}

// The coast's stopped handler clears the fling flag and restores the shared animation defaults,
// on a natural finish and on an unschedule alike.
void test_menu_layer__touch_fling_stopped_clears_state(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  menu_layer_touch_handle_snap(&l, GPoint(0, 0), GPoint(0, -88), GPoint(0, -500));
  cl_assert(l.touch_fling_active);
  Animation *anim = property_animation_get_animation(l.scroll_layer.animation);
  cl_assert(s_anim_handlers.stopped != NULL);
  cl_assert_equal_p(s_anim_handlers_context, &l);
  // Simulate the coast ending: the animation service unschedules, then fires stopped.
  animation_unschedule(anim);
  s_anim_handlers.stopped(anim, true /* finished */, s_anim_handlers_context);
  cl_assert(!l.touch_fling_active);
  cl_assert_equal_i(animation_get_duration(anim, false, false), ANIMATION_DEFAULT_DURATION_MS);
  cl_assert(s_anim_handlers.stopped == NULL);    // handlers cleared for the next plain scroll
}

// =============================================================================================
// Inertial fling (center-focused carousels): the coast tracks the row crossing the centre live,
// then the stopped handler settles the final row to the exact centre. Geometry: 10 rows x 44px,
// frame 180 -> row r centres at offset 68 - 44r; widened clamp [-370, 90]. Coast frames are
// simulated through the animation's actual setter (the T_STATIC internal offset setter), NOT
// scroll_layer_set_content_offset: the public call unschedules the running animation, which a
// real animation frame never does, and would mask a tracking path that kills the coast.

void prv_scroll_layer_set_content_offset_internal(ScrollLayer *scroll_layer, GPoint offset);

void test_menu_layer__touch_fling_center_tracks_rows_and_settles(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, 68);  // row 0 centred
  prv_reset_touch_counters();

  // The drag lands at -32 (centre inside row 2, tracked live by the pan); the fast liftoff then
  // schedules the coast toward -32 + (-800 * 240 / 1000) = -224, starting from the current offset
  // (no jump).
  menu_layer_touch_handle_pan_update(&l, GPoint(0, 68), GPoint(0, -100));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 2);
  cl_assert_equal_i(s_will_change_count, 1);
  menu_layer_touch_handle_snap(&l, GPoint(0, 68), GPoint(0, -100), GPoint(0, -800));
  cl_assert(l.touch_fling_active);
  cl_assert_equal_i(scroll_layer_get_content_offset(&l.scroll_layer).y, -32);  // no liftoff jump
  Animation *anim = property_animation_get_animation(l.scroll_layer.animation);
  cl_assert(anim != NULL);
  cl_assert(animation_is_scheduled(anim));
  cl_assert_equal_i(s_anim_to.y, -224);
  cl_assert_equal_i(animation_get_duration(anim, false, false), TOUCH_FLING_MAX_DURATION_MS);

  // A coast frame at -120 puts the centre (content y 210) inside row 4: the offset-changed
  // handler steps the selection there through the will_change contract.
  prv_scroll_layer_set_content_offset_internal(&l.scroll_layer, GPoint(0, -120));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 4);
  cl_assert_equal_i(s_will_change_count, 2);
  cl_assert_equal_i(s_selection_changed_count, 2);
  cl_assert_equal_i(s_select_click_count, 0);   // a coast never activates
  // Regression: the reselect on a row crossing must NOT unschedule the coast (the selection
  // highlight update's change_ongoing_animation path once killed the fling on the first crossing).
  cl_assert(animation_is_scheduled(anim));
  cl_assert(l.touch_fling_active);

  // The final coast frame lands at the target: centre (content y 314) is inside row 7.
  prv_scroll_layer_set_content_offset_internal(&l.scroll_layer, GPoint(0, -224));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 7);

  // Coast end: the stopped handler clears the flag, restores defaults, and glides row 7 to its
  // exact centre (offset 68 - 44*7 = -240).
  animation_unschedule(anim);
  s_anim_handlers.stopped(anim, true /* finished */, s_anim_handlers_context);
  cl_assert(!l.touch_fling_active);
  cl_assert(animation_is_scheduled(anim));      // the settle glide was scheduled
  cl_assert_equal_i(s_anim_to.y, -240);
  // (Defaults restoration is asserted in the unscheduled_no_settle test: the stubbed
  // property_animation_init zeroes the duration on the settle's re-init, unlike the real one.)
}

// A veto mid-coast keeps the old focus; the end-of-coast reconcile settles back to the vetoed
// row's centre instead of the row the content stopped under.
void test_menu_layer__touch_fling_center_veto_reconciles(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  s_will_change_mode = WillChange_Veto;

  menu_layer_touch_handle_pan_update(&l, GPoint(0, 68), GPoint(0, -100));
  menu_layer_touch_handle_snap(&l, GPoint(0, 68), GPoint(0, -100), GPoint(0, -500));
  cl_assert(l.touch_fling_active);
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 0);   // vetoed: focus stays
  Animation *anim = property_animation_get_animation(l.scroll_layer.animation);
  prv_scroll_layer_set_content_offset_internal(&l.scroll_layer, GPoint(0, -152));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 0);   // still vetoed mid-coast
  cl_assert_equal_i(s_selection_changed_count, 0);

  animation_unschedule(anim);
  s_anim_handlers.stopped(anim, true /* finished */, s_anim_handlers_context);
  cl_assert(!l.touch_fling_active);
  cl_assert(animation_is_scheduled(anim));
  cl_assert_equal_i(s_anim_to.y, 68);   // settle back to the vetoed row 0's centre
}

// A not-finished stop (caught, or something else took the offset) only clears state: whoever
// unscheduled the coast owns the settle, so none is scheduled here.
void test_menu_layer__touch_fling_center_unscheduled_no_settle(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  menu_layer_touch_handle_snap(&l, GPoint(0, 68), GPoint(0, -100), GPoint(0, -500));
  Animation *anim = property_animation_get_animation(l.scroll_layer.animation);
  cl_assert(l.touch_fling_active);
  animation_unschedule(anim);
  s_anim_handlers.stopped(anim, false /* finished */, s_anim_handlers_context);
  cl_assert(!l.touch_fling_active);
  cl_assert(!animation_is_scheduled(anim));
  cl_assert_equal_i(animation_get_duration(anim, false, false), ANIMATION_DEFAULT_DURATION_MS);
}

// =============================================================================================
// Catch-to-stop: a Touchdown during a coast stops it immediately, and the tap that gesture may
// become is swallowed (a catch is a stop, not a select).

void test_menu_layer__touch_catch_stops_fling_and_swallows_tap(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  menu_layer_touch_handle_snap(&l, GPoint(0, 0), GPoint(0, -88), GPoint(0, -500));
  cl_assert(l.touch_fling_active);
  Animation *anim = property_animation_get_animation(l.scroll_layer.animation);
  cl_assert(animation_is_scheduled(anim));

  // Finger down mid-coast: dead stop on a plain menu, and the gesture is marked as a catch.
  menu_layer_touch_handle_touchdown(&l);
  cl_assert(!l.touch_fling_active);
  cl_assert(!animation_is_scheduled(anim));
  cl_assert(l.touch_tap_swallow);
  cl_assert_equal_i(animation_get_duration(anim, false, false), ANIMATION_DEFAULT_DURATION_MS);

  // The catch's tap is swallowed: no selection contract, no activation; the flag is consumed.
  const MenuIndex before = menu_layer_get_selected_index(&l);
  menu_layer_touch_handle_tap(&l, GPoint(72, 22));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, before.row);
  cl_assert_equal_i(s_will_change_count, 0);
  cl_assert_equal_i(s_select_click_count, 0);
  cl_assert(!l.touch_tap_swallow);

  // A following gesture with no coast to catch taps normally (row 1: 44px rows, offset 0).
  menu_layer_touch_handle_touchdown(&l);
  cl_assert(!l.touch_tap_swallow);
  menu_layer_touch_handle_tap(&l, GPoint(72, 66));
  cl_assert_equal_i(s_will_change_count, 1);   // tap-select ran the contract this time
}

// A caught-then-abandoned gesture (the catch became a pan, not a tap) leaves a stale swallow mark;
// the next Touchdown reassigns it, so a later legitimate tap is never eaten.
void test_menu_layer__touch_catch_abandoned_swallow_reassigned(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  menu_layer_touch_handle_snap(&l, GPoint(0, 0), GPoint(0, -88), GPoint(0, -500));
  menu_layer_touch_handle_touchdown(&l);   // catch
  cl_assert(l.touch_tap_swallow);
  // ...the gesture becomes a pan (no tap consumes the flag)...
  menu_layer_touch_handle_pan_update(&l, GPoint(0, -88), GPoint(0, 20));
  menu_layer_touch_handle_snap(&l, GPoint(0, -88), GPoint(0, 20), GPointZero);
  // The next Touchdown has nothing to catch: the flag is assigned false, and the tap selects.
  menu_layer_touch_handle_touchdown(&l);
  cl_assert(!l.touch_tap_swallow);
  menu_layer_touch_handle_tap(&l, GPoint(72, 22));
  cl_assert_equal_i(s_will_change_count, 1);
}

// Catching a coasting carousel does not dead-stop off-grid: the catch glides the nearest row to
// the centre (a catch-and-hold past the tap window would otherwise freeze the carousel misaligned).
void test_menu_layer__touch_catch_center_focused_settles_to_row(void) {
  MenuLayer l;
  menu_layer_init(&l, &GRect(0, 0, 144, 180));
  menu_layer_set_center_focused(&l, true);
  prv_set_touch_callbacks(&l);
  menu_layer_reload_data(&l);
  prv_reset_touch_counters();
  menu_layer_touch_handle_snap(&l, GPoint(0, 68), GPoint(0, -100), GPoint(0, -500));
  cl_assert(l.touch_fling_active);
  // A coast frame lands at -120: the centre is inside row 4 (tracked live).
  prv_scroll_layer_set_content_offset_internal(&l.scroll_layer, GPoint(0, -120));
  cl_assert_equal_i(menu_layer_get_selected_index(&l).row, 4);

  menu_layer_touch_handle_touchdown(&l);
  cl_assert(!l.touch_fling_active);
  cl_assert(l.touch_tap_swallow);
  // The catch scheduled a settle toward row 4's exact centre: 68 - 44*4 = -108.
  Animation *anim = property_animation_get_animation(l.scroll_layer.animation);
  cl_assert(anim != NULL);
  cl_assert(animation_is_scheduled(anim));
  cl_assert_equal_i(s_anim_to.y, -108);
}
