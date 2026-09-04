/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "pbl/services/battery/battery_monitor.h"

//! Represents an angle relative to get to a reference direction, e.g. (magnetic) north.
//! The angle value is scaled linearly, such that a value of TRIG_MAX_ANGLE
//! corresponds to 360 degrees or 2 PI radians.
//! Thus, if heading towards north, north is 0, west is TRIG_MAX_ANGLE/4,
//! south is TRIG_MAX_ANGLE/2, and so on.
typedef int32_t CompassHeading;

//! Enum describing the current state of the Compass Service
typedef enum {
  //! The Compass Service is unavailable.
  CompassStatusUnavailable = -1,
  //! Compass is calibrating: data is invalid and should not be used
  //! Data will become valid once calibration is complete
  CompassStatusDataInvalid = 0,
  //! Compass is calibrating: the data is valid but the calibration is still being refined
  CompassStatusCalibrating,
  //! Compass data is valid and the calibration has completed
  CompassStatusCalibrated
} CompassStatus;

//! Structure containing a single heading towards magnetic and true north.
typedef struct {
  //! Measured angle that increases counter-clockwise from magnetic north
  //! (use `int clockwise_heading = TRIG_MAX_ANGLE - heading_data.magnetic_heading;`
  //! for example to find your heading clockwise from magnetic north).
  CompassHeading magnetic_heading;
  //! Currently same value as magnetic_heading (reserved for future implementation).
  CompassHeading true_heading;
  //! Indicates the current state of the Compass Service calibration.
  CompassStatus compass_status;
  //! Currently always false (reserved for future implementation).
  bool is_declination_valid;
} CompassHeadingData;

//! Register the ecompass service with the event service system
extern void ecompass_service_init(void);
extern void ecompass_service_handle(void);
extern void ecompass_handle_battery_state_change_event(PreciseBatteryChargeState new_state);
//! ecompass private routines

typedef enum {
  MagCalStatusNoSolution,
  MagCalStatusSavedSampleMatch,
  MagCalStatusNewSolutionAvail,
  MagCalStatusNewLockedSolutionAvail
} MagCalStatus;

//! Takes in raw 16 bit samples of mag data. From the samples, selects a good
//! set of points and runs a spherical fit, returning the origin in new_corr
//! whenever a new solution set is found.
//!
//! Possible returns are:
//!   NoSolution - No new hard iron correction estimate available
//!   SavedSampleMatch - iff saved_corr is specified, and several fits
//!       have been found which are close to this value have been found
//!   NewSolutionAvail - New solution set available
//!   NewLockedSolutionAvail - A set of solutions close to one another have
//!       been found. Result in new_corr is the average of these values
extern MagCalStatus ecomp_corr_add_raw_mag_sample(int16_t *sample,
    int16_t *saved_corr, int16_t *new_corr);

//! Drops any samples which have been collected as part of
//! ecomp_corr_add_raw_mag_sample and resets any state tracking
extern void ecomp_corr_reset(void);

//! @return True if the current task is subscribed to the compass service, false otherwise.
bool sys_ecompass_service_subscribed(void);

//! Populate the provided data struct with compass data from the service.
//! @param[out] data The struct to populate
void sys_ecompass_get_last_heading(CompassHeadingData *data);
