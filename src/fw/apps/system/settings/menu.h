/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/graphics/gtypes.h"
#include "applib/ui/layer.h"
#include "applib/ui/window.h"
#include "shell/prefs.h"

#include <stdint.h>


typedef enum {
  SettingsMenuItemBluetooth = 0,
  SettingsMenuItemNotifications,
  SettingsMenuItemVibrations,
  SettingsMenuItemQuietTime,
  SettingsMenuItemTimeline,
  SettingsMenuItemQuickLaunch,
  SettingsMenuItemDateTime,
  SettingsMenuItemDisplay,
  SettingsMenuItemHealth,
#ifdef CONFIG_THEMING
  SettingsMenuItemThemes,
#endif
  SettingsMenuItemActivity,
#ifdef CONFIG_MIC
  SettingsMenuItemRecordings,
#endif
  SettingsMenuItemSystem,
  SettingsMenuItem_Count,
  SettingsMenuItem_Invalid
} SettingsMenuItem;

struct SettingsCallbacks;
typedef struct SettingsCallbacks SettingsCallbacks;

typedef void (*SettingsDeinit)(SettingsCallbacks *context);
typedef uint16_t (*SettingsGetInitialSelection)(SettingsCallbacks *context);
typedef void (*SettingsSelectionChangedCallback)(SettingsCallbacks *context, uint16_t new_row,
                                                 uint16_t old_row);
typedef void (*SettingsSelectionWillChangeCallback)(SettingsCallbacks *context, uint16_t *new_row,
                                                    uint16_t old_row);
typedef void (*SettingsSelectClickCallback)(SettingsCallbacks *context, uint16_t row);
typedef void (*SettingsDrawRowCallback)(SettingsCallbacks *context, GContext *ctx,
                                        const Layer *cell_layer, uint16_t row, bool selected);
typedef uint16_t (*SettingsNumRowsCallback)(SettingsCallbacks *context);
typedef int16_t (*SettingsRowHeightCallback)(SettingsCallbacks *context, uint16_t row,
                                             bool is_selected);
typedef void (*SettingsExpandCallback)(SettingsCallbacks *context);
typedef void (*SettingsAppearCallback)(SettingsCallbacks *context);
typedef void (*SettingsHideCallback)(SettingsCallbacks *context);

struct SettingsCallbacks {
  SettingsDeinit deinit;
  SettingsDrawRowCallback draw_row;
  SettingsGetInitialSelection get_initial_selection;
  SettingsSelectionChangedCallback selection_changed;
  SettingsSelectionWillChangeCallback selection_will_change;
  SettingsSelectClickCallback select_click;
  SettingsNumRowsCallback num_rows;
  SettingsRowHeightCallback row_height;
  SettingsExpandCallback expand;
  SettingsAppearCallback appear;
  SettingsHideCallback hide;
};

typedef Window *(*SettingsInitFunction)(void);

typedef struct {
  const char *name;
  SettingsInitFunction init;
} SettingsModuleMetadata;

typedef const SettingsModuleMetadata *(*SettingsModuleGetMetadata)(void);

void settings_menu_mark_dirty(SettingsMenuItem category);
void settings_menu_reload_data(SettingsMenuItem category);
int16_t settings_menu_get_selected_row(SettingsMenuItem category);

const SettingsModuleMetadata *settings_menu_get_submodule_info(SettingsMenuItem category);
const char *settings_menu_get_status_name(SettingsMenuItem category);
void settings_menu_push(SettingsMenuItem category);
