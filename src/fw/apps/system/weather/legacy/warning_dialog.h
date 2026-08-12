/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/ui/dialogs/expandable_dialog.h"

#include <stdbool.h>

typedef ExpandableDialog WeatherAppWarningDialog;

typedef void (*WeatherAppWarningDialogDismissedCallback)(void);

WeatherAppWarningDialog *weather_app_warning_dialog_push(const char *localized_string,
    WeatherAppWarningDialogDismissedCallback dismissed_cb);
