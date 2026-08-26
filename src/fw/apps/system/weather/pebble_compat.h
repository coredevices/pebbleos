/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
//! Compatibility shim for porting the Weather app from a sandboxed SDK app
//! (which includes <pebble.h>) into the firmware-linked system app, which uses
//! the granular applib headers directly. Every ported .c/.h includes this
//! instead of <pebble.h>. Over-inclusive on purpose; the linker drops unused.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

// --- UI framework (umbrella: window, layer, text/bitmap/menu/scroll layers,
//     animation, click, app_window_stack, vibes, ...) ---
#include "applib/ui/ui.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/window_stack.h"
#include "applib/ui/click.h"
#include "applib/ui/recognizer/recognizer.h"

// --- Graphics ---
#include "applib/graphics/graphics.h"
#include "applib/graphics/gtypes.h"
#include "applib/graphics/gbitmap_png.h"
#include "applib/graphics/gdraw_command_image.h"
#include "applib/graphics/gdraw_command_sequence.h"
#include "applib/graphics/gdraw_command_frame.h"
#include "applib/graphics/gpath.h"  // sin_lookup / cos_lookup / TRIG_MAX_ANGLE
#include "applib/graphics/text.h"

// --- Fonts ---
#include "applib/fonts/fonts.h"

// --- App services / timers / events ---
#include "applib/app.h"
#include "applib/pbl_std/pbl_std.h"  // pbl_override_localtime
#include "applib/app_timer.h"
#include "applib/tick_timer_service.h"
#include "applib/event_service_client.h"
#include "applib/preferred_content_size.h"

// --- Touch (firmware applib app-facing touch service: TouchEvent{type,x,y}) ---
#include "applib/touch_service.h"

// --- Persistent storage ---
#include "applib/persist.h"

// --- Process / app state / heap ---
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "kernel/pbl_malloc.h"
#include "kernel/events.h"

// --- Resources (real generated ids; replaces the stored-app pinned header) ---
#include "resource/resource_ids.auto.h"
#include "applib/applib_resource.h"  // app-facing ResHandle resource API

// NOTE: the firmware weather headers (weather_service.h / weather_types.h /
// weather_db.h) are intentionally NOT included here — they define WeatherType
// and WeatherLocationForecast, which COLLIDE with the app's own same-named
// types in its weather_types.h. Only the data-source adapter
// (weather_data_source.c) touches the firmware weather types; the views and
// weather.c use the app's types + the neutral WxDsForecast struct.

// --- Misc utils the ported code uses ---
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/clock.h"
#include "pbl/util/math.h"
#include "pbl/util/trig.h"  // sin_lookup / cos_lookup / TRIG_MAX_ANGLE / TRIG_MAX_RATIO
#include "util/time/time.h"
#include "pbl/util/uuid.h"
#include "pbl/logging/logging.h"
#include "shell/system_theme.h"

// --- Window-stack call-site shims: the SDK names map 1:1 to the app-window
//     variants firmware apps must use. ---
#define window_stack_push(window, animated)        app_window_stack_push((window), (animated))
#define window_stack_pop(animated)                 app_window_stack_pop((animated))
#define window_stack_pop_all(animated)             app_window_stack_pop_all((animated))
#define window_stack_remove(window, animated)      app_window_stack_remove((window), (animated))
#define window_stack_get_top_window()              app_window_stack_get_top_window()

// ===========================================================================
// SDK→firmware applib SIGNATURE shims.
// The firmware applib passes GRect by const-pointer and uses out-params, while
// the SDK (which the ported code targets) passes GRect by value and returns it.
// The firmware ships `*_by_value` convenience variants that match the SDK
// signatures exactly, so we macro-map the SDK names onto them. Defined LAST,
// after all firmware headers are included (their declarations are processed
// before these function-like macros become active).
// ===========================================================================
#include "pbl/drivers/rtc.h"  // rtc_get_time

#define layer_get_bounds(layer)                  layer_get_bounds_by_value(layer)
#define layer_get_frame(layer)                   layer_get_frame_by_value(layer)
#define layer_set_frame(layer, ...)              layer_set_frame_by_value((layer), __VA_ARGS__)
#define menu_layer_set_callbacks(ml, ctx, ...)   menu_layer_set_callbacks_by_value((ml), (ctx), __VA_ARGS__)
// Variadic: a WindowHandlers brace-literal arg contains commas that the
// preprocessor would otherwise split into multiple macro arguments.
#define window_set_window_handlers(w, ...)       window_set_window_handlers_by_value((w), __VA_ARGS__)
#define graphics_text_layout_get_content_size(...) app_graphics_text_layout_get_content_size(__VA_ARGS__)
#define graphics_fill_rect(ctx, rect, rad, mask) graphics_fill_round_rect_by_value((ctx), (rect), (rad), (mask))
#define graphics_draw_rect(ctx, rect)            graphics_draw_rect_by_value((ctx), (rect))
#define graphics_draw_round_rect(ctx, rect, rad) graphics_draw_round_rect_by_value((ctx), (rect), (rad))
#define graphics_draw_bitmap_in_rect(ctx, bmp, rect) graphics_draw_bitmap_in_rect_by_value((ctx), (bmp), (rect))

// Raw resource loading: the firmware's bare resource_* take a ResAppNum; the
// app-facing applib_resource_* variants match the SDK's by-id/by-handle forms.
#define resource_get_handle(id)             applib_resource_get_handle(id)
#define resource_size(handle)               applib_resource_size(handle)
#define resource_load(handle, buf, max)     applib_resource_load((handle), (buf), (max))

// App-task heap (the ported code uses the libc names). NOTE: the *_check variants
// CROAK (kill the app) on OOM and never return NULL — any `if (!p)` after these is
// unreachable. Sites with a real fallback path (skip-the-effect scratch buffers,
// optional resource loads) must use malloc_try instead.
#define calloc(n, sz) app_calloc_check((n), (sz))
#define malloc(sz)    app_malloc_check(sz)
#define malloc_try(sz) app_malloc(sz)
#define free(p)       app_free(p)

// Current wall-clock epoch (ported code calls time(NULL)).
#define time(p)       ((void)(p), rtc_get_time())
// localtime isn't linked for apps; the applib provides the override.
#define localtime(t)  pbl_override_localtime(t)
