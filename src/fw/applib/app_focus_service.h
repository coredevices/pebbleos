/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include <stdbool.h>

//! @addtogroup Foundation
//! @{
//!   @addtogroup EventService
//!   @{
//!     @addtogroup AppFocusService
//!
//!
//! \brief Handling app focus
//! The AppFocusService allows developers to be notified when their apps become visible on the
//! screen. Common reasons your app may be running but  not be on screen are: it's still in the
//! middle of launching and being revealed by a system animation, or it is being covered by a system
//! window such as a notification. This service is useful for apps that require a high degree of
//! user interactivity, like a game where you'll want to pause when a notification covers your app
//! window. It can be also used for apps that want to sync up an intro animation to the end of the
//! system animation that occurs before your app is visible.
//!
//! @{

//! Callback type for focus events
//! @param in_focus True if the app is gaining focus, false otherwise.
typedef void (*AppFocusHandler)(bool in_focus);

//! There are two different focus events which take place when transitioning to and from an app
//! being in focus. Below is an example of when these events will occur:
//! 1) The app is launched. Once the system animation to the app has completed and the app is
//! completely in focus, the did_focus handler is called with in_focus set to true.
//! 2) A notification comes in and the animation to show the notification starts. The will_focus
//! handler is called with in_focus set to false.
//! 3) The animation completes and the notification is in focus, with the app being completely
//! covered. The did_focus handler is called with in_focus set to false.
//! 4) The notification is dismissed and the animation to return to the app starts. The will_focus
//! handler is called with in_focus set to true.
//! 5) The animation completes and the app is in focus. The did_focus handler is called with
//! in_focus set to true.
typedef struct {
  //! Handler which will be called right before an app will lose or gain focus.
  //! @note This will be called with in_focus set to true when a window which is covering the app is
  //! about to close and return focus to the app.
  //! @note This will be called with in_focus set to false when a window which will cover the app is
  //! about to open, causing the app to lose focus.
  AppFocusHandler will_focus;
  //! Handler which will be called when an animation finished which has put the app into focus or
  //! taken the app out of focus.
  //! @note This will be called with in_focus set to true when a window which was covering the app
  //! has closed and the app has gained focus.
  //! @note This will be called with in_focus set to false when a window has opened which is now
  //! covering the app, causing the app to lose focus.
  AppFocusHandler did_focus;
} AppFocusHandlers;

//! Subscribe to the focus event service. Once subscribed, the handlers get called every time the
//! app gains or loses focus.
//! @param handlers Handlers which will be called on will-focus and did-focus events.
//! @see AppFocusHandlers
void app_focus_service_subscribe_handlers(AppFocusHandlers handlers);

//! Subscribe to the focus event service. Once subscribed, the handler
//! gets called every time the app focus changes.
//! @note Calling this function is equivalent to
//! \code{.c}
//! app_focus_service_subscribe_handlers((AppFocusHandlers){
//!   .will_focus = handler,
//! });
//! \endcode
//! @note Out focus events are triggered when a modal window is about to open and cover the app.
//! @note In focus events are triggered when a modal window which is covering the app is about to
//! close.
//! @param handler A callback to be called on will-focus events.
void app_focus_service_subscribe(AppFocusHandler handler);

//! Unsubscribe from the focus event service. Once unsubscribed, the previously
//! registered handlers will no longer be called.
void app_focus_service_unsubscribe(void);

//!     @} // end addtogroup AppFocusService
//!   @} // end addtogroup EventService
//! @} // end addtogroup Foundation
