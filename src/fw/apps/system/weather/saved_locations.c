/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "saved_locations.h"

#include "weather_data_source.h"
#include "weather_types.h"
#include "pbl/util/math.h"   // integer_sqrt — the launcher's chord-inset math

#define SAVED_LOCATIONS_ROW_HEIGHT 44
#define SAVED_LOCATIONS_TOUCH_AXIS_THRESHOLD_PX 5
#define SAVED_LOCATIONS_TOUCH_TAP_THRESHOLD_PX 10
#define SAVED_LOCATIONS_TOUCH_SWIPE_BACK_PX 20  // horizontal right-swipe = BACK to the globe
#define SAVED_LOCATIONS_FLING_PROJECT_MS 340    // how far a flick coasts (ms of velocity)
#define SAVED_LOCATIONS_FLING_MIN_VELOCITY 120  // px/s; below this a release just stops
typedef struct {
  Window *window;
  MenuLayer *menu_layer;
  SavedLocationsSelectCallback select_callback;
  void *select_context;
  int active_ds_index;       // pre-highlighted record, -1 = none
  int scroll_at_drag_start;
  int16_t touch_start_x;
  int16_t touch_start_y;
  bool touch_active;
  bool drag_axis_set;
  bool drag_is_vertical;
  int16_t last_drag_y;       // momentum: most recent drag sample position
  uint32_t last_drag_ms;     // ... and its time, for velocity
  int fling_velocity;        // most recent finger velocity, px/s (down = +)
} SavedLocationsView;

static SavedLocationsView *s_view;

// The phone's locations, snapshotted when the screen opens / the globe reloads.
// This is the ONLY model the rows and the globe pins are built from.
static SavedLocationEntry s_entries[SAVED_LOCATIONS_MAX_ENTRIES];
static int s_entry_count;

#define GLANCE_WEATHER_TYPES 9  // WeatherType_PartlyCloudy(0) .. WeatherType_RainAndSnow(8)
static GBitmap *s_glance_icons[GLANCE_WEATHER_TYPES];

static void prv_glance_destroy_icons(void) {
  for (int i = 0; i < GLANCE_WEATHER_TYPES; i++) {
    if (s_glance_icons[i]) {
      gbitmap_destroy(s_glance_icons[i]);
      s_glance_icons[i] = NULL;
    }
  }
}

static GBitmap *prv_glance_icon(uint8_t type) {
  if (type >= GLANCE_WEATHER_TYPES) return NULL;
  if (!s_glance_icons[type]) {
    s_glance_icons[type] =
        gbitmap_create_with_resource(weather_type_icon_tiny_resource(type));
  }
  return s_glance_icons[type];
}

static void prv_activate_saved_row(SavedLocationsView *view, int row);
#if WEATHER_PLATFORM_TOUCH_COLOR
static void prv_touch_handler(const TouchEvent *event, void *context);
#endif

int saved_locations_get_entries(SavedLocationEntry *entries, int max_entries) {
  if (!entries || max_entries <= 0) return 0;
  if (!weather_ds_supported()) return 0;
  const int total = weather_ds_location_count();
  if (total <= 0) return 0;

  WxDsForecast *ds = malloc_try(sizeof(*ds));   // ~400 B; keep off the task stack
  if (!ds) return 0;

  int count = 0;
  // Two passes so the phone's current-location record always leads the list;
  // the rest follow in the phone's own order.
  for (int pass = 0; pass < 2 && count < max_entries; pass++) {
    for (int i = 0; i < total && count < max_entries; i++) {
      if (!weather_ds_read_index(i, ds)) continue;
      const bool is_current = ds->is_current_location;
      if ((pass == 0) != is_current) continue;
      SavedLocationEntry *e = &entries[count++];
      *e = (SavedLocationEntry) {
        .ds_index = (int16_t)i,
        .is_current_location = is_current,
        .latitude_e2 = ds->latitude_e2,
        .longitude_e2 = ds->longitude_e2,
        // No coordinates => the globe cannot pin it (the watch has no geocoder).
        .has_coordinates = (ds->latitude_e2 != INT16_MIN &&
                            ds->longitude_e2 != INT16_MIN),
        .temp = (int16_t)ds->current_temp,
        .weather_type = ds->current_weather_type,
      };
      strncpy(e->label, ds->location_name, sizeof(e->label) - 1);
      e->label[sizeof(e->label) - 1] = '\0';
    }
  }
  free(ds);
  return count;
}

// ---- Row model: row i IS s_entries[i] ---- (count/persist desync). It must
// never occupy a row, or it draws as a blank "ghost" cell (observed after a
// voice-add). prv_compact_customs() heals the persisted store on load; these two
// helpers make the row model count/map only real slots, so even a phantom created
// mid-session (before the next compaction) can't render.
static void prv_refresh_entries(void) {
  s_entry_count = saved_locations_get_entries(s_entries, SAVED_LOCATIONS_MAX_ENTRIES);
}

// One placeholder row when the phone has synced nothing, so the screen never
// renders as a blank rectangle.
static int prv_num_rows(void) {
  return s_entry_count > 0 ? s_entry_count : 1;
}

static int prv_row_for_ds_index(int ds_index) {
  for (int i = 0; i < s_entry_count; i++) {
    if (s_entries[i].ds_index == ds_index) return i;
  }
  return -1;
}

static uint16_t prv_get_num_sections(MenuLayer *menu_layer, void *context) {
  (void)menu_layer;
  (void)context;
  return 1;
}

static uint16_t prv_get_num_rows(MenuLayer *menu_layer, uint16_t section,
                                 void *context) {
  (void)menu_layer;
  (void)section;
  (void)context;
  return prv_num_rows();
}

static int16_t prv_get_cell_height(MenuLayer *menu_layer, MenuIndex *cell_index,
                                   void *context) {
  (void)menu_layer;
  (void)cell_index;
  (void)context;
  return SAVED_LOCATIONS_ROW_HEIGHT;
}

// Dense "at a glance" row: [condition icon] City name          21°
// Falls back to the classic two-line menu cell when the city has no synced
// weather snapshot.
// [icon] City name                    21°
// Falls back to the plain menu cell when the record carries no temperature.
static void prv_draw_glance_row(GContext *ctx, const Layer *cell_layer,
                                const SavedLocationEntry *glance) {
  const char *title = glance->label[0] ? glance->label : "Location";
  const char *subtitle = glance->is_current_location ? "Current Location" : NULL;
  if (glance->temp == (int16_t)WX_DS_UNKNOWN_TEMP) {
    menu_cell_basic_draw(ctx, cell_layer, title, subtitle, NULL);
    return;
  }

  GRect bounds = layer_get_bounds(cell_layer);
#if PBL_ROUND && PBL_DISPLAY_HEIGHT >= 200
  // Rows follow the glass EXACTLY the way the gabbro app drawer's do. Mirrored from
  // launcher/default: menu_layer.c prv_menu_layer_draw_row measures the row's LIVE on-screen
  // centre each frame, and app_glance_structured.c prv_draw_processed insets the content frame
  // by base 10 + the chord shortfall (R - sqrt(R^2 - dy^2), R = PBL_DISPLAY_HEIGHT/2), both
  // sides, leaving the highlight bar full-width. drawing_box.origin.y is this row's top in
  // absolute screen coords — the MenuLayer adds the scroll offset AND the centre-focus bounce
  // before calling draw_row (applib/ui/menu_layer.c:306-331), so scrolling re-derives the
  // inset every frame with no extra plumbing, exactly like the launcher.
  {
    const int16_t radius = PBL_DISPLAY_HEIGHT / 2;
    const int16_t row_center_y = ctx->draw_state.drawing_box.origin.y + bounds.size.h / 2;
    const int16_t y_offset_from_center = row_center_y - radius;
    const int32_t y_offset_sq = (int32_t)y_offset_from_center * y_offset_from_center;
    const int32_t radius_sq = (int32_t)radius * radius;
    const int32_t sqrt_arg = radius_sq - y_offset_sq;
    const int16_t circle_inset = radius - integer_sqrt(sqrt_arg > 0 ? sqrt_arg : 0);
    const int16_t base_inset = 10;   // the launcher's padding beyond the geometric chord
    int16_t horizontal_inset = (int16_t)(base_inset + circle_inset);
    // DIVERGENCE from the launcher, and it matters: the launcher's menu frame is inset so its
    // rows never approach the glass edge, but THIS menu is full-screen — a row scrolled near
    // the top/bottom gets a chord inset wider than the row itself, and the un-clamped result
    // fed graphics_draw_text a negative-width rect, which reset the watch the moment the
    // screen opened. Clamp so at least 60px of content width always survives.
    const int16_t max_inset = (int16_t)((bounds.size.w - 60) / 2);
    if (horizontal_inset > max_inset) horizontal_inset = max_inset;
    bounds = grect_inset_internal(bounds, horizontal_inset, 0);
  }
#endif
  const bool highlighted = menu_cell_layer_is_highlighted(cell_layer);
  GFont font = fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD);
  graphics_context_set_text_color(ctx,
                                  highlighted ? GColorWhite : GColorBlack);

  char temp_text[12];
  snprintf(temp_text, sizeof(temp_text), "%d°", (int)glance->temp);
  const int temp_w = 48;
  const int text_y = (bounds.size.h - 28) / 2 - 3;
  graphics_draw_text(ctx, temp_text, font,
                     GRect(bounds.origin.x + bounds.size.w - temp_w - 4, text_y, temp_w, 30),
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentRight,
                     NULL);

  // Rows show the CITY only — the synced name is "New York, United States",
  // which truncates to an unreadable "New York, Un..." in a row this wide. The
  // globe label still carries the full region/country.
  char city[SAVED_LOCATION_LABEL_SIZE];
  strncpy(city, title, sizeof(city) - 1);
  city[sizeof(city) - 1] = 0;
  char *comma = strchr(city, ',');
  if (comma) *comma = 0;
  title = city;

  int title_x = 8;
  GBitmap *icon = prv_glance_icon(glance->weather_type);
  if (icon) {
    graphics_context_set_compositing_mode(ctx, GCompOpSet);
    graphics_draw_bitmap_in_rect(
        ctx, icon, GRect(bounds.origin.x + 5, (bounds.size.h - 25) / 2, 25, 25));
    title_x = 5 + 25 + 5;
  }

  const int16_t title_w = (int16_t)(bounds.size.w - title_x - temp_w - 8);
  if (title_w <= 0) return;
  graphics_draw_text(ctx, title, font,
                     GRect(bounds.origin.x + title_x, text_y, title_w, 30),
                     GTextOverflowModeTrailingEllipsis, GTextAlignmentLeft,
                     NULL);
}

static void prv_draw_row(GContext *ctx, const Layer *cell_layer,
                         MenuIndex *cell_index, void *context) {
  (void)context;
  const int row = cell_index->row;
  if (s_entry_count <= 0) {
    menu_cell_basic_draw(ctx, cell_layer, "No Locations",
                         "Add them in the app", NULL);
    return;
  }
  if (row < 0 || row >= s_entry_count) return;
  prv_draw_glance_row(ctx, cell_layer, &s_entries[row]);
}

static void saved_locations_dismiss(bool animated);   // defined below

// Selecting a row switches the app to that location and closes the screen.
// Locations are added/removed in the phone app — the watch only picks.
static void prv_activate_saved_row(SavedLocationsView *view, int row) {
  if (!view || row < 0 || row >= s_entry_count) return;
  if (view->select_callback) {
    view->select_callback(s_entries[row].ds_index, view->select_context);
  }
  saved_locations_dismiss(true);
}

#if WEATHER_PLATFORM_TOUCH_COLOR
static uint32_t prv_now_ms(void) {
  time_t s = 0;
  uint16_t ms = 0;
  time_ms(&s, &ms);
  return (uint32_t)s * 1000u + ms;
}

static int prv_touch_max_scroll(SavedLocationsView *view) {
  if (!view || !view->menu_layer) return 0;
  int rows = prv_num_rows();
  GRect bounds = layer_get_bounds(menu_layer_get_layer(view->menu_layer));
  int max_scroll = rows * SAVED_LOCATIONS_ROW_HEIGHT - bounds.size.h;
  return max_scroll > 0 ? max_scroll : 0;
}

static int prv_touch_clamp_scroll(SavedLocationsView *view, int amount) {
  if (amount < 0) return 0;
  int max_scroll = prv_touch_max_scroll(view);
  return amount > max_scroll ? max_scroll : amount;
}

static int prv_touch_scroll_amount(SavedLocationsView *view) {
  if (!view || !view->menu_layer) return 0;
  ScrollLayer *scroll_layer = menu_layer_get_scroll_layer(view->menu_layer);
  if (!scroll_layer) return 0;
  return -scroll_layer_get_content_offset(scroll_layer).y;
}

static void prv_touch_set_scroll(SavedLocationsView *view, int amount,
                                 bool animated) {
  if (!view || !view->menu_layer) return;
  ScrollLayer *scroll_layer = menu_layer_get_scroll_layer(view->menu_layer);
  if (!scroll_layer) return;
  amount = prv_touch_clamp_scroll(view, amount);
  scroll_layer_set_content_offset(scroll_layer, GPoint(0, -amount), animated);
}

static void prv_touch_select_row(SavedLocationsView *view, int row) {
  if (!view || !view->menu_layer) return;
  int rows = prv_num_rows();
  if (rows <= 0) return;
  if (row < 0) row = 0;
  if (row >= rows) row = rows - 1;
  menu_layer_set_selected_index(view->menu_layer, MenuIndex(0, row),
                                MenuRowAlignNone, false);
}

static int prv_touch_row_at_y(SavedLocationsView *view, int16_t y) {
  // y is SCREEN-space; the round frame starts 20px down, so convert into frame space first.
  const int fy = layer_get_frame(menu_layer_get_layer(view->menu_layer)).origin.y;
  int row = (prv_touch_scroll_amount(view) + y - fy) / SAVED_LOCATIONS_ROW_HEIGHT;
  int rows = prv_num_rows();
  if (row < 0 || row >= rows) return -1;
  return row;
}

static void prv_touch_handler(const TouchEvent *event, void *context) {
  SavedLocationsView *view = (SavedLocationsView *)context;
  if (!view || !view->menu_layer) {
    return;
  }

  if (event->type == TouchEvent_Touchdown) {
    view->touch_active = true;
    view->touch_start_x = event->x;
    view->touch_start_y = event->y;
    view->scroll_at_drag_start = prv_touch_scroll_amount(view);
    view->drag_axis_set = false;
    view->drag_is_vertical = false;
    view->last_drag_y = event->y;
    view->last_drag_ms = prv_now_ms();
    view->fling_velocity = 0;
    // Phone-style: don't highlight a row on touchdown. Only a deliberate tap
    // selects (decided on liftoff) — this stops the selection from jumping to
    // wherever your finger lands while you're really just starting a scroll.
    return;
  }

  if (event->type == TouchEvent_PositionUpdate && view->touch_active) {
    int16_t dx = event->x - view->touch_start_x;
    int16_t dy = event->y - view->touch_start_y;
    int16_t adx = dx < 0 ? -dx : dx;
    int16_t ady = dy < 0 ? -dy : dy;

    if (!view->drag_axis_set &&
        (adx > SAVED_LOCATIONS_TOUCH_AXIS_THRESHOLD_PX ||
         ady > SAVED_LOCATIONS_TOUCH_AXIS_THRESHOLD_PX)) {
      view->drag_axis_set = true;
      view->drag_is_vertical = ady >= adx;
    }

    if (view->drag_is_vertical) {
      // 1:1 finger tracking.
      int amount = prv_touch_clamp_scroll(
          view, view->scroll_at_drag_start - (int)dy);
      prv_touch_set_scroll(view, amount, false);
      // Track finger velocity over the latest segment (px/s, down = +) for the
      // release fling. Light smoothing rejects single-sample jitter; if the finger
      // pauses before lifting, velocity decays to ~0 so there's no fling (correct).
      uint32_t now = prv_now_ms();
      uint32_t seg_dt = now - view->last_drag_ms;
      if (seg_dt > 0) {
        int seg_v = ((int)(event->y - view->last_drag_y) * 1000) / (int)seg_dt;
        view->fling_velocity = (view->fling_velocity + seg_v * 2) / 3;
        view->last_drag_y = event->y;
        view->last_drag_ms = now;
      }
    }
    return;
  }

  if (event->type == TouchEvent_Liftoff && view->touch_active) {
    view->touch_active = false;
    int16_t dx = event->x - view->touch_start_x;
    int16_t dy = event->y - view->touch_start_y;
    if (dx >= -SAVED_LOCATIONS_TOUCH_TAP_THRESHOLD_PX &&
        dx <= SAVED_LOCATIONS_TOUCH_TAP_THRESHOLD_PX &&
        dy >= -SAVED_LOCATIONS_TOUCH_TAP_THRESHOLD_PX &&
        dy <= SAVED_LOCATIONS_TOUCH_TAP_THRESHOLD_PX) {
      int row = prv_touch_row_at_y(view, event->y);
      if (row >= 0) {
        prv_touch_select_row(view, row);
        prv_activate_saved_row(view, row);
      }
      return;
    }

    if (view->drag_axis_set && !view->drag_is_vertical &&
        dx > SAVED_LOCATIONS_TOUCH_SWIPE_BACK_PX) {
      window_stack_pop(true);   // swipe right = BACK to the globe cradle
      return;
    }

    if (view->drag_is_vertical) {
      // Momentum: coast in the fling direction and ease to a stop. Project a
      // target from the release velocity; the scroll layer's animated move
      // decelerates into it (a fast flick travels further than a gentle one).
      int v = view->fling_velocity;          // px/s, down = +
      int av = v < 0 ? -v : v;
      int target = prv_touch_scroll_amount(view);
      if (av >= SAVED_LOCATIONS_FLING_MIN_VELOCITY) {
        int dist = (v * SAVED_LOCATIONS_FLING_PROJECT_MS) / 1000;
        target = prv_touch_clamp_scroll(view, target - dist);
      }
#if PBL_ROUND
      // Every touch rest lands ON the row grid, so no row ever sits half-cut at the window
      // edge: round to the nearest 44px multiple and ease into it. (The scroll range's ends
      // are themselves multiples of 44 — rows*44 - 220 — so the clamp can't un-align it.)
      target = ((target + SAVED_LOCATIONS_ROW_HEIGHT / 2) / SAVED_LOCATIONS_ROW_HEIGHT)
               * SAVED_LOCATIONS_ROW_HEIGHT;
      target = prv_touch_clamp_scroll(view, target);
      prv_touch_set_scroll(view, target, true);
#else
      if (av >= SAVED_LOCATIONS_FLING_MIN_VELOCITY) {
        prv_touch_set_scroll(view, target, true);   // animated = ease-out deceleration
      } else {
        prv_touch_set_scroll(view, prv_touch_clamp_scroll(view, target), false);
      }
#endif
    }
  }
}
#endif

static void prv_select_click(MenuLayer *menu_layer, MenuIndex *cell_index,
                             void *context) {
  (void)menu_layer;
  SavedLocationsView *view = (SavedLocationsView *)context;
  if (!view || !cell_index) return;
  prv_activate_saved_row(view, cell_index->row);
}

static void prv_window_unload(Window *window) {
  SavedLocationsView *view = (SavedLocationsView *)window_get_user_data(window);
  if (!view) return;

  if (view->menu_layer) {
    menu_layer_destroy(view->menu_layer);
    view->menu_layer = NULL;
  }
  prv_glance_destroy_icons();
#if WEATHER_PLATFORM_TOUCH_COLOR
  touch_service_unsubscribe();
#endif
  window_destroy(view->window);
  s_view = NULL;
  free(view);
}

#if WEATHER_PLATFORM_TOUCH_COLOR
// Subscribe on APPEAR (not before the push): the covered window's disappear handler fires
// during the push transition and releases the single touch slot — a pre-push subscribe
// would be clobbered by it. Appear runs after every disappear/unload in the transition.
static void prv_window_appear(Window *window) {
  SavedLocationsView *view = (SavedLocationsView *)window_get_user_data(window);
  if (view) touch_service_subscribe(prv_touch_handler, view);
}
#endif

static void saved_locations_dismiss(bool animated) {
  if (!s_view || !s_view->window) return;
  window_stack_remove(s_view->window, animated);
}

void saved_locations_push(const SavedLocationsConfig *config) {
  if (s_view) {
    if (config) {
      s_view->active_ds_index = config->active_ds_index;
      s_view->select_callback = config->select_callback;
      s_view->select_context = config->select_context;
    }
    prv_refresh_entries();
    s_view->touch_active = false;
    s_view->drag_axis_set = false;
    s_view->drag_is_vertical = false;
    s_view->scroll_at_drag_start = 0;
    s_view->touch_start_x = 0;
    s_view->touch_start_y = 0;
    menu_layer_reload_data(s_view->menu_layer);
    window_stack_push(s_view->window, true);   // touch subscribe happens in .appear
    return;
  }

  SavedLocationsView *view = calloc(1, sizeof(SavedLocationsView));
  if (!view) return;
  s_view = view;
  prv_refresh_entries();

  view->active_ds_index = config ? config->active_ds_index : -1;
  view->select_callback = config ? config->select_callback : NULL;
  view->select_context = config ? config->select_context : NULL;

  view->window = window_create();
  if (!view->window) {
    free(view);
    s_view = NULL;
    return;
  }
  window_set_user_data(view->window, view);
  window_set_background_color(view->window, GColorWhite);
  window_set_window_handlers(view->window, (WindowHandlers) {
#if WEATHER_PLATFORM_TOUCH_COLOR
    .appear = prv_window_appear,
#endif
    .unload = prv_window_unload,
  });

  Layer *root = window_get_root_layer(view->window);
  GRect bounds = layer_get_bounds(root);
#if PBL_ROUND
  // Exactly 5 full rows (the launcher's own trick: its menu frame is inset so its row stack
  // fits precisely). 260 shows 5.9 44px rows full-screen, so rows peeked half-cut at both
  // edges; a (260 - 5*44)/2 = 20px vertical inset makes the ScrollLayer clip to a 220px
  // window = 5 rows exactly. Centre-focus then keeps every BUTTON rest grid-aligned for
  // free (uniform 44px rows centred on the frame centre land on multiples of 44).
  bounds = grect_inset_internal(bounds, 0,
      (int16_t)((bounds.size.h - 5 * SAVED_LOCATIONS_ROW_HEIGHT) / 2));
#endif
  view->menu_layer = menu_layer_create(bounds);
  if (!view->menu_layer) {
    window_destroy(view->window);
    free(view);
    s_view = NULL;
    return;
  }

  menu_layer_set_callbacks(view->menu_layer, view, (MenuLayerCallbacks) {
    .get_num_sections = prv_get_num_sections,
    .get_num_rows = prv_get_num_rows,
    .get_cell_height = prv_get_cell_height,
    .draw_row = prv_draw_row,
    .select_click = prv_select_click,
  });
  menu_layer_set_normal_colors(view->menu_layer, GColorWhite, GColorBlack);
  menu_layer_set_highlight_colors(view->menu_layer,
                                  PBL_IF_COLOR_ELSE(GColorVividCerulean, GColorBlack),
                                  GColorWhite);
  menu_layer_set_click_config_onto_window(view->menu_layer, view->window);
  layer_add_child(root, menu_layer_get_layer(view->menu_layer));

  int selected_row = (view->active_ds_index >= 0)
      ? prv_row_for_ds_index(view->active_ds_index) : -1;
  if (selected_row < 0 || selected_row >= prv_num_rows()) selected_row = 0;
  menu_layer_set_selected_index(view->menu_layer,
                                MenuIndex(0, selected_row),
                                MenuRowAlignCenter, false);

  window_stack_push(view->window, true);
}
