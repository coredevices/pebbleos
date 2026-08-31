/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/activity/activity.h"
#include "system/status_codes.h"

//! Get the typical metric value for a given day.
//! If you want "typical steps" you probably want health_db_get_typical_step_averages
bool health_db_get_typical_value(ActivityMetric metric,
                                 DayInWeek day,
                                 int32_t *value_out);

//! Get the average metric value over the last month
bool health_db_get_monthly_average_value(ActivityMetric metric,
                                         int32_t *value_out);

//! Often referred to as "typical steps"
bool health_db_get_typical_step_averages(DayInWeek day,
                                         ActivityMetricAverages *averages);



//! For test / debug purposes only
bool health_db_set_typical_values(ActivityMetric metric,
                                  DayInWeek day,
                                  uint16_t *values,
                                  int num_values);

///////////////////////////////////////////
// BlobDB Boilerplate (see blob_db/api.h)
///////////////////////////////////////////

void health_db_init(void);

status_t health_db_insert(const uint8_t *key, int key_len, const uint8_t *val, int val_len);

int health_db_get_len(const uint8_t *key, int key_len);

status_t health_db_read(const uint8_t *key, int key_len, uint8_t *val_out, int val_out_len);

status_t health_db_delete(const uint8_t *key, int key_len);

status_t health_db_flush(void);

status_t health_db_compact(void);
