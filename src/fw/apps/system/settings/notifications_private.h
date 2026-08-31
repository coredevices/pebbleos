/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "applib/preferred_content_size.h"

//! TODO PBL-41920: This mapping should be an opt in set in a platform specific location
typedef enum SettingsContentSize {
  SettingsContentSize_Small,
  SettingsContentSize_Default,
  SettingsContentSize_Large,
  SettingsContentSizeCount,
} SettingsContentSize;

static inline SettingsContentSize settings_content_size_from_preferred_size(
    PreferredContentSize preferred_size) {
  return preferred_size + (SettingsContentSize_Default - PreferredContentSizeDefault);
}

static inline PreferredContentSize settings_content_size_to_preferred_size(
    SettingsContentSize settings_size) {
  return settings_size + (PreferredContentSizeDefault - SettingsContentSize_Default);
}
