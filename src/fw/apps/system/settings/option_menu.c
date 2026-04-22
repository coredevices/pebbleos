/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "option_menu.h"

#include "menu.h"

#include "kernel/pbl_malloc.h"
#include "pbl/services/i18n/i18n.h"

static void prv_menu_unload(OptionMenu *option_menu, void *context) {
  SettingsOptionMenuData *data = context;
  if (data->callbacks.unload) {
    data->callbacks.unload(option_menu, data);
  }
  option_menu_destroy(option_menu);
  i18n_free_all(option_menu);
  task_free(context);
}

static uint16_t prv_menu_get_num_rows(OptionMenu *option_menu, void *context) {
  return ((SettingsOptionMenuData *)context)->num_rows;
}

static void prv_menu_draw_row(OptionMenu *option_menu, GContext *ctx, const Layer *cell_layer,
                              const GRect *cell_frame, uint32_t row, bool selected, void *context) {
  SettingsOptionMenuData *data = context;
  const char *title = i18n_get(data->rows[row], option_menu);
  option_menu_system_draw_row(option_menu, ctx, cell_layer, cell_frame, title, selected, context);
}

OptionMenu *settings_option_menu_create(
    const char *i18n_title_key, OptionMenuContentType content_type, int choice,
    const OptionMenuCallbacks *callbacks_ref, uint16_t num_rows, bool icons_enabled,
    const char **rows, void *context) {
  OptionMenu *option_menu = option_menu_create();
  if (!option_menu) {
    return NULL;
  }
  GColor highlight_bg = shell_prefs_get_theme_highlight_color();
  const OptionMenuConfig config = {
    .title = i18n_get(i18n_title_key, option_menu),
    .content_type = content_type,
    .choice = choice,
    .status_colors = { system_theme_get_bg_color(), system_theme_get_fg_color() },
    .highlight_colors = { highlight_bg, gcolor_legible_over(highlight_bg) },
    .icons_enabled = icons_enabled,
  };
  option_menu_configure(option_menu, &config);
  SettingsOptionMenuData *data = task_malloc_check(sizeof(SettingsOptionMenuData));
  OptionMenuCallbacks callbacks = *callbacks_ref;
  *data = (SettingsOptionMenuData) {
    .callbacks = callbacks,
    .context = context,
    .num_rows = num_rows,
    .rows = rows,
  };
  callbacks.draw_row = prv_menu_draw_row;
  callbacks.get_num_rows = prv_menu_get_num_rows;
  callbacks.unload = prv_menu_unload;
  option_menu_set_callbacks(option_menu, &callbacks, data);
  return option_menu;
}

OptionMenu *settings_option_menu_push(
    const char *i18n_title_key, OptionMenuContentType content_type, int choice,
    const OptionMenuCallbacks *callbacks_ref, uint16_t num_rows, bool icons_enabled,
    const char **rows, void *context) {
  OptionMenu * const option_menu = settings_option_menu_create(
      i18n_title_key, content_type, choice, callbacks_ref, num_rows, icons_enabled, rows, context);
  if (option_menu) {
    const bool animated = true;
    app_window_stack_push(&option_menu->window, animated);
  }
  return option_menu;
}

void *settings_option_menu_get_context(SettingsOptionMenuData *data) {
  return data->context;
}
