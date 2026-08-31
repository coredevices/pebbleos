/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include "app_install_manager.h"

//! @file app_install_manager_private.h
//! These are the "private" functions used by submodules of app_install_manager.

typedef void (*InstallCallbackDoneCallback)(void*);

//! Used by app_custom_icon and app_db, so invoke add/remove/update/app_db_clear callbacks.
//! This function takes care of calling the callbacks on the proper task, so this function
//! can be called from any task.
//! @return false if a callback is already in progress
bool app_install_do_callbacks(InstallEventType event_type, AppInstallId install_id, Uuid *uuid,
    InstallCallbackDoneCallback done_callback, void* done_callback_data);
