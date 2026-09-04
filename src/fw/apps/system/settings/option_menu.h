/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "applib/ui/option_menu_window.h"

typedef struct {
  OptionMenuCallbacks callbacks;
  void *context;
  const char **rows;
  uint16_t num_rows;
} SettingsOptionMenuData;

OptionMenu *settings_option_menu_create(
    const char *i18n_title_key, OptionMenuContentType content_type, int choice,
    const OptionMenuCallbacks *callbacks, uint16_t num_rows, bool icons_enabled, const char **rows,
    void *context);

OptionMenu *settings_option_menu_push(
    const char *i18n_title_key, OptionMenuContentType content_type, int choice,
    const OptionMenuCallbacks *callbacks, uint16_t num_rows, bool icons_enabled, const char **rows,
    void *context);

void *settings_option_menu_get_context(SettingsOptionMenuData *data);
