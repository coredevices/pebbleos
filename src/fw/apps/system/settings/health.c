/* SPDX-License-Identifier: Apache-2.0 */

#include "health.h"
#include "menu.h"
#include "option_menu.h"
#include "window.h"

#include "applib/ui/option_menu_window.h"
#include "kernel/pbl_malloc.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/i18n/i18n.h"
#include "pbl/services/activity/activity.h"
#include "shell/prefs.h"
#include "system/passert.h"
#include "pbl/util/size.h"

#include <string.h>

typedef struct SettingsHealthData {
    SettingsCallbacks callbacks;
} SettingsHealthData;

static const char *s_units_distance_labels[] = {
    i18n_noop("Kilometers"),
    i18n_noop("Miles"),
};

#ifdef CONFIG_HRM
static const char *s_hrm_interval_labels[] = {
    i18n_noop("10 Minutes"),
    i18n_noop("30 Minutes"),
    i18n_noop("1 Hour"),
    i18n_noop("Disabled"),
};
#endif

enum SettingsHealthItem {
    SettingsHealthTrackingEnabled,
    SettingsHealthUnitDistance,
#ifdef CONFIG_HRM
    SettingsHealthHRMonitoringInterval,
    SettingsHealthHRActivityTracking,
#endif
#ifdef CONFIG_BLE_HRM_SERVICE
    SettingsHealthBLEHrmSharing,
#endif
    NumSettingsHealthItems
};

#ifdef CONFIG_HRM
// HRM Interval option menu
/////////////////////////////

static void prv_hrm_interval_menu_select(OptionMenu *option_menu, int selection, void *context) {
    activity_prefs_set_hrm_measurement_interval((HRMonitoringInterval)selection);
    app_window_stack_remove(&option_menu->window, true /*animated*/);
}

static void prv_hrm_interval_menu_push(SettingsHealthData *data) {
    const int index = (int)activity_prefs_get_hrm_measurement_interval();
    const OptionMenuCallbacks callbacks = {
        .select = prv_hrm_interval_menu_select,
    };
    const char *title = i18n_noop("HR Monitoring");
    settings_option_menu_push(
        title, OptionMenuContentType_SingleLine, index, &callbacks,
        ARRAY_LENGTH(s_hrm_interval_labels), true /* icons_enabled */,
        s_hrm_interval_labels, data);
}
#endif

// Menu Callbacks
/////////////////////////////

static void prv_deinit_cb(SettingsCallbacks *context) {
    SettingsHealthData *data = (SettingsHealthData*)context;

    i18n_free_all(data);
    app_free(data);
}

static void prv_draw_row_cb(SettingsCallbacks *context, GContext *ctx,
                            const Layer *cell_layer, uint16_t row, bool selected) {
    SettingsHealthData *data = (SettingsHealthData*) context;

    const char *title = NULL;
    const char *subtitle = NULL;

    switch (row) {
        case SettingsHealthTrackingEnabled: {
            title = i18n_noop("Health Tracking");
            subtitle = activity_prefs_tracking_is_enabled()
                ? i18n_noop("On") : i18n_noop("Off");
            break;
        }
        case SettingsHealthUnitDistance: {
            title = i18n_noop("Distance Unit");
            UnitsDistance unit = shell_prefs_get_units_distance();
            if (unit >= UnitsDistanceCount) {
                subtitle = i18n_noop("Unknown");
            } else {
                subtitle = s_units_distance_labels[unit];
            }
            break;
        }
#ifdef CONFIG_HRM
        case SettingsHealthHRMonitoringInterval: {
            title = i18n_noop("HR Monitoring");
            HRMonitoringInterval interval = activity_prefs_get_hrm_measurement_interval();
            if (interval >= HRMonitoringIntervalCount) {
                subtitle = i18n_noop("Unknown");
            } else {
                subtitle = s_hrm_interval_labels[interval];
            }
            break;
        }
        case SettingsHealthHRActivityTracking: {
            title = i18n_noop("HR During Activity");
            subtitle = activity_prefs_hrm_activity_tracking_is_enabled()
                ? i18n_noop("On") : i18n_noop("Off");
            break;
        }
#endif
#ifdef CONFIG_BLE_HRM_SERVICE
        case SettingsHealthBLEHrmSharing: {
            title = i18n_noop("Activity HR Sharing");
            subtitle = activity_prefs_ble_hrm_sharing_is_enabled()
                ? i18n_noop("On") : i18n_noop("Off");
            break;
        }
#endif
        default:
            WTF;
    }
    menu_cell_basic_draw(ctx, cell_layer, i18n_get(title, data), i18n_get(subtitle, data), NULL);
}

static void prv_select_click_cb(SettingsCallbacks *context, uint16_t row) {
    switch (row) {
        case SettingsHealthTrackingEnabled: {
            bool new_value = !activity_prefs_tracking_is_enabled();
            activity_prefs_tracking_set_enabled(new_value);
            if (new_value) {
                activity_start_tracking(false);
            } else {
                activity_stop_tracking();
            }
            break;
        }
        case SettingsHealthUnitDistance: {
            UnitsDistance unit = shell_prefs_get_units_distance();
            unit = (unit + 1) % UnitsDistanceCount;
            shell_prefs_set_units_distance(unit);
            break;
        }
#ifdef CONFIG_HRM
        case SettingsHealthHRMonitoringInterval:
            prv_hrm_interval_menu_push((SettingsHealthData*)context);
            break;
        case SettingsHealthHRActivityTracking:
            activity_prefs_set_hrm_activity_tracking_enabled(
                !activity_prefs_hrm_activity_tracking_is_enabled());
            break;
#endif
#ifdef CONFIG_BLE_HRM_SERVICE
        case SettingsHealthBLEHrmSharing:
            activity_prefs_ble_hrm_sharing_set_enabled(
                !activity_prefs_ble_hrm_sharing_is_enabled());
            break;
#endif
        default:
            WTF;
    }
    settings_menu_reload_data(SettingsMenuItemHealth);
    settings_menu_mark_dirty(SettingsMenuItemHealth);
}

static uint16_t prv_num_rows_cb(SettingsCallbacks *context) {
    if (!activity_prefs_tracking_is_enabled()) {
        return 1; // Only show the Health Tracking toggle
    }
#ifdef CONFIG_BLE_HRM_SERVICE
    // Only show "Activity HR Sharing" when "HR During Activity" is enabled
    if (!activity_prefs_hrm_activity_tracking_is_enabled()) {
        return NumSettingsHealthItems - 1;
    }
#endif
    return NumSettingsHealthItems;
}

static void prv_appear_cb(SettingsCallbacks *context) {
}

static void prv_hide_cb(SettingsCallbacks *context) {
}

static Window *prv_init(void) {
    SettingsHealthData *data = app_malloc_check(sizeof(*data));
    *data = (SettingsHealthData){};

    data->callbacks = (SettingsCallbacks) {
        .deinit = prv_deinit_cb,
        .draw_row = prv_draw_row_cb,
        .select_click = prv_select_click_cb,
        .num_rows = prv_num_rows_cb,
        .appear = prv_appear_cb,
        .hide = prv_hide_cb,
    };

    return settings_window_create(SettingsMenuItemHealth, &data->callbacks);
}

const SettingsModuleMetadata *settings_health_get_info(void) {
    static const SettingsModuleMetadata s_module_info = {
        .name = i18n_noop("Health"),
        .init = prv_init,
    };

    return &s_module_info;
}