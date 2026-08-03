/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pebble_compat.h"
#include "weather_platform.h"

//! Locations are owned by the PHONE. The watch neither adds nor removes them —
//! it only selects between the records the phone has synced into the weather
//! DB. Every entry here is a snapshot of one such record, and `ds_index` is its
//! identity (the index weather_ds_read_index() takes), so a selection never has
//! to be re-derived by matching names or coordinates.
#define SAVED_LOCATIONS_MAX_ENTRIES 12
#define SAVED_LOCATION_LABEL_SIZE 48

typedef struct {
  int16_t ds_index;           //!< weather-ds record index — the stable identity
  bool    is_current_location; //!< the phone's "current location" record
  char    label[SAVED_LOCATION_LABEL_SIZE];
  int16_t latitude_e2;        //!< latitude * 100, INT16_MIN unknown
  int16_t longitude_e2;       //!< longitude * 100, INT16_MIN unknown
  bool    has_coordinates;    //!< false => cannot be pinned on the globe
  int16_t temp;               //!< current temp for the row glance, or WX_DS_UNKNOWN_TEMP
  uint8_t weather_type;       //!< WeatherType for the row glance icon
} SavedLocationEntry;

//! Fired when the user picks a location. `ds_index` indexes the weather-ds
//! records directly — no matching required.
typedef void (*SavedLocationsSelectCallback)(int ds_index, void *context);

typedef struct {
  int active_ds_index;   //!< pre-highlight this record's row; -1 = none
  SavedLocationsSelectCallback select_callback;
  void *select_context;
} SavedLocationsConfig;

void saved_locations_push(const SavedLocationsConfig *config);

//! Snapshot the phone's synced locations into `entries` (current location
//! first, then the rest in the phone's own order).
//! @return the number of entries written (0 if the phone has synced none).
int saved_locations_get_entries(SavedLocationEntry *entries, int max_entries);
