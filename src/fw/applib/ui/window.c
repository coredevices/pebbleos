/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "window_private.h"

#include "applib/app_logging.h"
#include "applib/graphics/graphics.h"
#include "applib/ui/app_window_click_glue.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/click.h"
#include "applib/ui/layer.h"
#include "applib/ui/layer_private.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/window_manager.h"
#include "applib/ui/window_stack.h"
#include "applib/applib_malloc.auto.h"
#include "applib/legacy2/ui/status_bar_legacy2.h"
#include "kernel/ui/kernel_ui.h"
#include "kernel/ui/modals/modal_manager.h"
#include "process_management/process_manager.h"
#include "process_state/app_state/app_state.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"
#include "syscall/syscall.h"

#include "status_bar_layer.h"

#include <string.h>

typedef enum {
  WindowHandlerOffsetLoad = offsetof(WindowHandlers, load),
  WindowHandlerOffsetAppear = offsetof(WindowHandlers, appear),
  WindowHandlerOffsetDisappear = offsetof(WindowHandlers, disappear),
  WindowHandlerOffsetUnload = offsetof(WindowHandlers, unload),
} WindowHandlerOffset;

void window_do_layer_update_proc(Layer *layer, GContext* ctx) {
  Window *window = layer_get_window(layer);

  const GColor bg_color = window->background_color;
  if (!gcolor_is_transparent(bg_color)) {
    GDrawState prev_state = graphics_context_get_drawing_state(ctx);
    graphics_context_set_fill_color(ctx, bg_color);
    graphics_fill_rect(ctx, &layer->bounds);
    graphics_context_set_drawing_state(ctx, prev_state);
  }
}

//! helper struct to move displacement logic out of window_render()
typedef struct {
  GPoint drawing_box_origin;
  GRect clip_box;
} DrawingStateOrigins;

static void prv_adjust_drawing_state_for_legacy2_apps(DrawingStateOrigins *saved_state,
                                                      GContext *ctx, Window *window) {
  GDrawState * const draw_state = &ctx->draw_state;
  *saved_state = (DrawingStateOrigins){
      .drawing_box_origin = draw_state->drawing_box.origin,
      .clip_box = draw_state->clip_box,
  };

  const int16_t full_screen_displacement = window->is_fullscreen ? 0 : STATUS_BAR_HEIGHT;
  draw_state->drawing_box.origin.y += full_screen_displacement;
  draw_state->clip_box.origin.y += full_screen_displacement;

  WindowStack *stack = window->parent_window_stack;
  if (window_transition_context_has_legacy_window_to(stack, window)) {
    // for 2.x apps, we cannot animate the window frame during a transition but need to use this
    // externalized state
    const GPoint displacement = stack->transition_context.window_to_displacement;
    gpoint_add_eq(&draw_state->drawing_box.origin, displacement);
    gpoint_add_eq(&draw_state->clip_box.origin, displacement);
  }

  // clip_box must respect screen boundaries
  grect_clip(&draw_state->clip_box, &saved_state->clip_box);
}

static void prv_restore_drawing_state(DrawingStateOrigins *saved_state, GContext *ctx) {
  ctx->draw_state.drawing_box.origin = saved_state->drawing_box_origin;
  ctx->draw_state.clip_box = saved_state->clip_box;
}

void prv_render_legacy2_system_status_bar(GContext *ctx, Window *window) {
  if (!window->is_fullscreen) {
    // adjust clipping rectangle so that rendering doesn't happen outside of the window
    // this prevents instant colors changes when going from one window to another
    GRect saved_clip_box = ctx->draw_state.clip_box;
    grect_clip(&ctx->draw_state.clip_box, &window->layer.frame);

    StatusBarLayerConfig config = {
        .foreground_color = GColorWhite,
        .background_color = GColorBlack,
        .mode = StatusBarLayerModeClock,
    };
    GRect frame = window->layer.frame;
    // window.frame.origin.y is 0 already (for 2.x compatibility reasons)
    // see prv_adjust_drawing_state_for_legacy2_apps()
    // so all we need to alter is the height of the frame
    frame.size.h = STATUS_BAR_HEIGHT;

    if (window_stack_is_animating_with_fixed_status_bar(window->parent_window_stack)) {
      frame.origin.x = 0;
    }

    status_bar_layer_render(ctx, &frame, &config);

    ctx->draw_state.clip_box = saved_clip_box;
  }
}

void window_render(Window *window, GContext *ctx) {
  PBL_ASSERTN(window);

  if (window->on_screen == false) {
    window->is_render_scheduled = false;
    return;
  }

  // workaround for 3rd-party apps
  // if a window is configured as non-fullscreen, it's frame needs to start at .origin={0,0}
  // to compensate for cases where clients configure a layer hierarchy with
  //   my_layer = layer_create(window.root_layer.frame) // ! wrong, should be .bounds
  // Of course on the screen, it still needs to start at {0, 16}. We adjust for that by
  // moving the GContext's draw_state before we traverse the layer hierarchy to render it.
  // Also see window_calc_frame()
  DrawingStateOrigins saved_state;
  prv_adjust_drawing_state_for_legacy2_apps(&saved_state, ctx, window);

  layer_render_tree(&window->layer, ctx);

  prv_restore_drawing_state(&saved_state, ctx);

  prv_render_legacy2_system_status_bar(ctx, window);

  window->is_render_scheduled = false;
}

void window_call_handler(Window *window, WindowHandlerOffset handler_offset) {
  if (window == NULL) {
    return;
  }
  WindowHandler handler = *(WindowHandler*)(((uint8_t*)&window->window_handlers) + handler_offset);
  if (handler) {
    handler(window);
  }
}

void window_schedule_render(Window *window) {
  window->is_render_scheduled = true;
}

GRect window_calc_frame(bool fullscreen) {
  GContext *ctx = graphics_context_get_current_context();
  GRect result = (GRect) {
    .origin = { 0, 0 },
    .size = graphics_context_get_framebuffer_size(ctx)
  };
  result.size.h -= fullscreen ? 0 : STATUS_BAR_HEIGHT;
  return result;
}

// FIXME: there is a problem in this function:
// This function initializes the root layer to be the screen size.  So, on a
// new window with a status bar, unless otherwise forced to with something
// like window_set_on_screen(), the window is first rendered at position (0,0);
// then this function shifts it to its correct position of (0, STATUS_BAR_HEIGHT).
// Either this function should set the window not on screen, or we should provide
// an alternate function for initializing the window that takes a frame dimension too.
void window_init(Window *window, const char* debug_name) {
  if (window == NULL) {
    PBL_LOG_ERR("Tried to init a NULL window");
    return;
  }
  *window = (Window){};
#ifndef CONFIG_RELEASE
  window->debug_name = debug_name;
#else
  (void)debug_name;
#endif

  bool fullscreen = !process_manager_compiled_with_legacy2_sdk();
  const GRect frame = window_calc_frame(fullscreen);
  layer_init(&window->layer, &frame);
  window->is_fullscreen = fullscreen;
  window->layer.window = window;
  window->layer.update_proc = window_do_layer_update_proc;
  window->background_color = GColorWhite;
  window->in_click_config_provider = false;
  window->is_waiting_for_click_config = false;
  window->parent_window_stack = NULL;
}

Window* window_create(void) {
  Window* window = applib_type_malloc(Window);
  if (window) {
    window_init(window, "");
  }
  return window;
}

void window_destroy(Window* window) {
  if (window == NULL) {
    return;
  }
  window_deinit(window);
  applib_free(window);
}

void window_deinit(Window *window) {
  PBL_ASSERTN(window);

  // Drop the recognizer manager's pointer to this window before it is torn down
  window_lost_input_focus(window);

  // FIXME: is there a way to cancel a pending render event?
  window_set_on_screen(window, false, true);

  layer_remove_child_layers(&window->layer);

  window_unload(window);
}

void window_set_overrides_back_button(Window *window, bool overrides_back_button) {
  if (overrides_back_button == window->overrides_back_button) {
    return;
  }
  window->overrides_back_button = overrides_back_button;
}

void window_set_touch_bridge_disabled(Window *window, bool disabled) {
  if (!window || (disabled == window->touch_bridge_disabled)) {
    return;
  }
  window->touch_bridge_disabled = disabled;
}

void window_set_touch_tap_requires_action_bar(Window *window, bool requires_action_bar) {
  if (!window || (requires_action_bar == window->touch_tap_requires_action_bar)) {
    return;
  }
  window->touch_tap_requires_action_bar = requires_action_bar;
}

static ClickManager* prv_get_current_click_manager(void) {
  return window_manager_get_window_click_manager(window_manager_get_top_window());
}

static void prv_call_click_provider(Window *window) {
  window->is_waiting_for_click_config = false;
  app_click_config_setup_with_window(prv_get_current_click_manager(), window);
  window->is_click_configured = true;
}

static void prv_check_is_in_click_config_provider(Window *window, char *type) {
  PBL_ASSERT(window->in_click_config_provider,
      "Click %s must be set from click config provider (Window %p)", type, window);
}

void window_setup_click_config_provider(Window *window) {
  prv_call_click_provider(window);
}

void window_set_click_config_provider_with_context(
    Window *window, ClickConfigProvider click_config_provider, void *context) {
  PBL_ASSERTN(window);
  window->click_config_provider = click_config_provider;
  window->click_config_context = context;

  if (window->on_screen && !window->is_unfocusable) {
    // We're already on screen, make the config provider get called.
    prv_call_click_provider(window);
  } else {
    window->is_waiting_for_click_config = true;
  }
}

void window_set_click_config_provider(Window *window, ClickConfigProvider click_config_provider) {
  window_set_click_config_provider_with_context(window, click_config_provider, NULL);
}

void window_set_click_context(ButtonId button_id, void *context) {
  prv_check_is_in_click_config_provider(window_manager_get_top_window(), "context");
  ClickManager *mgr = prv_get_current_click_manager();
  ClickConfig *cfg = &mgr->recognizers[button_id].config;

  cfg->context = context;
}

void window_single_click_subscribe(ButtonId button_id, ClickHandler handler) {
  Window *window = window_manager_get_top_window();
  prv_check_is_in_click_config_provider(window, "subscribe");
  ClickManager *mgr = prv_get_current_click_manager();
  ClickConfig *cfg = &mgr->recognizers[button_id].config;

  cfg->click.repeat_interval_ms = 0;
  cfg->click.handler = handler;

  if (button_id == BUTTON_ID_BACK) {
    window_set_overrides_back_button(window, true);
  }
}

void window_single_repeating_click_subscribe(ButtonId button_id, uint16_t repeat_interval_ms, ClickHandler handler) {
  prv_check_is_in_click_config_provider(window_manager_get_top_window(), "subscribe");
  if (button_id == BUTTON_ID_BACK) {
    PBL_LOG_ERR("Cannot register BUTTON_ID_BACK repeating click handler");
    return;
  }
  ClickManager *mgr = prv_get_current_click_manager();
  ClickConfig *cfg = &mgr->recognizers[button_id].config;

  cfg->click.repeat_interval_ms = repeat_interval_ms;
  cfg->click.handler = handler;
}

void window_multi_click_subscribe(ButtonId button_id, uint8_t min_clicks, uint8_t max_clicks, uint16_t timeout,
                                  bool last_click_only, ClickHandler handler) {
  Window *window = window_manager_get_top_window();
  prv_check_is_in_click_config_provider(window, "subscribe");
  ClickManager *mgr = prv_get_current_click_manager();
  ClickConfig *cfg = &mgr->recognizers[button_id].config;

  cfg->multi_click.min = (min_clicks == 0) ? 2 : min_clicks;
  cfg->multi_click.max = (max_clicks == 0) ? min_clicks : max_clicks;
  cfg->multi_click.timeout = (timeout == 0) ? 300 : timeout;
  cfg->multi_click.last_click_only = last_click_only;
  cfg->multi_click.handler = handler;

  if (button_id == BUTTON_ID_BACK) {
    window_set_overrides_back_button(window, true);
  }
}

void window_long_click_subscribe(ButtonId button_id, uint16_t delay_ms, ClickHandler down_handler, ClickHandler up_handler) {
  prv_check_is_in_click_config_provider(window_manager_get_top_window(), "subscribe");
  if (button_id == BUTTON_ID_BACK) {
    // We only want system apps to be able to override the back button for long
    // clicks. Allowing third-party apps to override the back button would make
    // long-pressing the back button a normal interaction method, and users may
    // unintentionally hold the button too long and force-quit the app.
    if (app_install_id_from_app_db(sys_process_manager_get_current_process_id())) {
      PBL_LOG_ERR("Cannot register BUTTON_ID_BACK long click handler");
      return;
    } else {
      Window *window = window_manager_get_top_window();
      window_set_overrides_back_button(window, true);
    }
  }
  ClickManager *mgr = prv_get_current_click_manager();
  ClickConfig *cfg = &mgr->recognizers[button_id].config;

  cfg->long_click.delay_ms = (delay_ms == 0) ? 500 : delay_ms;
  cfg->long_click.handler = down_handler;
  cfg->long_click.release_handler = up_handler;
}

void window_raw_click_subscribe(ButtonId button_id, ClickHandler down_handler, ClickHandler up_handler, void *context) {
  prv_check_is_in_click_config_provider(window_manager_get_top_window(), "subscribe");
  if (button_id == BUTTON_ID_BACK) {
    PBL_LOG_ERR("Cannot register BUTTON_ID_BACK raw handler");
    return;
  }
  ClickManager *mgr = prv_get_current_click_manager();
  ClickConfig *cfg = &mgr->recognizers[button_id].config;

  cfg->raw.up_handler = up_handler;
  cfg->raw.down_handler = down_handler;
  cfg->raw.context = context;
}

ClickConfigProvider window_get_click_config_provider(const Window *window) {
  return window->click_config_provider;
}

void *window_get_click_config_context(Window *window) {
  return window->click_config_context;
}

void window_set_window_handlers(Window *window, const WindowHandlers *handlers) {
  if (handlers) {
    window->window_handlers = *handlers;
  }
}

void window_set_window_handlers_by_value(Window *window, WindowHandlers handlers) {
  window_set_window_handlers(window, &handlers);
}

void window_set_user_data(Window *window, void *data) {
  window->user_data = data;
}

void* window_get_user_data(const Window *window) {
  return window->user_data;
}

struct Layer* window_get_root_layer(const Window *window) {
  return &((Window *)window)->layer;
}

static void prv_window_load(Window *window) {
  if (window->is_loaded) {
    return;
  }
  window_call_handler(window, WindowHandlerOffsetLoad);
  window->is_loaded = true;
}

void window_unload(Window *window) {
  if (!window->is_loaded) {
    return;
  }
  window->is_loaded = false;
  window_call_handler(window, WindowHandlerOffsetUnload);

  // Don't touch window after calling it's unload handler. We allow windows to free themselves on unload.
}

// TODO PBL-1769: deal with window unload. In app deinit? When low memory?

void window_set_on_screen(Window *window, bool new_on_screen, bool call_window_appear_handlers) {
  PBL_ASSERTN(window != NULL);    // This tripped me up for about a day
  if (new_on_screen == window->on_screen) {
    return;
  }

  // Window went from offscreen to onscreen (or vice versa)
  // Provides internal signaling to ui elements of appear/disappear
  layer_property_changed_tree(&window->layer);
  window->on_screen = new_on_screen;

  if (window->on_screen) {
    window_schedule_render(window);
    // The click provider was set but not updated
    if (window->is_waiting_for_click_config && !window->is_unfocusable) {
      prv_call_click_provider(window);
    }
  } else {
    window->is_render_scheduled = false;
    window->is_waiting_for_click_config = false;
    window->is_click_configured = false;
  }

  if (call_window_appear_handlers) {
    if (window->on_screen) {
      prv_window_load(window);
      // In our load handler, we may unload ourselves; this is perfectly fine!  However,
      // if we do that, we never appear on the screen!  In that case, window->on_screen
      // may have changed between the time we checked and after we called prv_window_load,
      // so we need to check it again!
      if (window->on_screen) {
        // Window has no cache, so when it appears, schedule (re)render:
        window_call_handler(window, WindowHandlerOffsetAppear);
      }
    } else if (window->is_loaded) {
      // We have to have loaded (and consequently appeared) to actually disappear because
      // we can actually set ourselves off-screen before we've ever been on-screen (this
      // happens if we unload ourselves in our load handler), so we have to double check.
      window_call_handler(window, WindowHandlerOffsetDisappear);
    }
  }
}

void window_set_background_color(Window *window, GColor background_color) {
  const GColor window_bg_color = window->background_color;
  if (gcolor_equal(background_color, window_bg_color)) {
    return;
  }
  window->background_color = background_color;
  layer_mark_dirty(&window->layer);
}

void window_set_background_color_2bit(Window *window, GColor2 background_color) {
  window_set_background_color(window, get_native_color(background_color));
}

void window_set_fullscreen(Window *window, bool enabled) {
  if (window->is_fullscreen == enabled) {
    return;
  }
  window->is_fullscreen = enabled;
  window->layer.frame = window_calc_frame(enabled);
  window->layer.bounds.size = window->layer.frame.size;

  layer_mark_dirty(&window->layer);
}

bool window_get_fullscreen(const Window *window) {
  return window->is_fullscreen;
}

void window_set_status_bar_icon(Window *window, const GBitmap *icon) {
  return;
}

bool window_is_on_screen(Window *window) {
  return (window->on_screen);
}

bool window_is_loaded(Window *window) {
  return (window->is_loaded);
}

void window_set_transparent(Window *window, bool transparent) {
  window->is_transparent = transparent;
}

bool window_is_transparent(Window *window) {
  return window->is_transparent;
}

void window_set_focusable(Window *window, bool focusable) {
  window->is_unfocusable = !focusable;
}

bool window_is_focusable(Window *window) {
  return !window->is_unfocusable;
}

const char* window_get_debug_name(Window *window) {
#ifndef CONFIG_RELEASE
    return window->debug_name;
#else
    return "?";
  (void)window;
#endif
}

// A simple wrapper so feedback can be given to developers if click config subscriptions are made from outside of the
// click config configuration callback.
void window_call_click_config_provider(Window *window, void *context) {
  window->in_click_config_provider = true;
  window->click_config_provider(context);
  window->in_click_config_provider = false;
}

static bool prv_find_status_bar_layer(Layer *layer, void *ctx) {
  if (layer_is_status_bar_layer(layer)) {
    *((StatusBarLayer **)ctx) = (StatusBarLayer *)layer;
    return false; // prevent further iterating
  }
  return true;
}

bool window_has_status_bar(Window *window) {
  if (!window) {
    return false;
  }

  if (!window->is_fullscreen) {
    return true;
  }

  StatusBarLayer *status_bar = NULL;
  layer_process_tree(&window->layer, &status_bar, prv_find_status_bar_layer);
  return status_bar && !layer_get_hidden(&status_bar->layer);
}

void window_attach_recognizer(Window *window, Recognizer *recognizer) {
  if (!window) {
    return;
  }
  layer_attach_recognizer(window_get_root_layer(window), recognizer);
}

void window_detach_recognizer(Window *window, Recognizer *recognizer) {
  if (!window) {
    return;
  }
  layer_detach_recognizer(window_get_root_layer(window), recognizer);
}

RecognizerList *window_get_recognizer_list(Window *window) {
  if (!window) {
    return NULL;
  }
  return layer_get_recognizer_list(window_get_root_layer(window));
}

RecognizerManager *window_get_recognizer_manager(Window *window) {
#ifdef CONFIG_TOUCH
  if (!window || !window->parent_window_stack) {
    return NULL;
  }
  if (window_manager_is_app_window(window)) {
    return app_state_get_recognizer_manager();
  }
  // A non-app (modal) window routes to the kernel (modal) recognizer manager.
  return modal_manager_get_recognizer_manager();
#else
  return NULL;
#endif
}

void window_became_input_focus(Window *window) {
#ifdef CONFIG_TOUCH
  RecognizerManager *manager = window_get_recognizer_manager(window);
  if (!manager) {
    return;
  }
  recognizer_manager_cancel_and_reset(manager);
  recognizer_manager_set_window(manager, window);
#endif
}

void window_lost_input_focus(Window *window) {
#ifdef CONFIG_TOUCH
  RecognizerManager *manager = window_get_recognizer_manager(window);
  if (!manager || (manager->window != window)) {
    // No-op unless this window currently holds the input focus
    return;
  }
  recognizer_manager_cancel_and_reset(manager);
  recognizer_manager_set_window(manager, NULL);
#endif
}
