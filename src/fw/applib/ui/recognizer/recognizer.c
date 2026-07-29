/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "recognizer.h"
#include "recognizer_impl.h"
#include "recognizer_list.h"
#include "recognizer_manager.h"
#include "recognizer_private.h"

#include "applib/applib_malloc.auto.h"
#include "applib/ui/layer.h"
#include "system/passert.h"
#include "pbl/util/list.h"

#include <stddef.h>

_Static_assert(RECOGNIZER_INSTANCE_SIZE >= sizeof(Recognizer),
               "RECOGNIZER_INSTANCE_SIZE too small for Recognizer");

static void prv_set_state(Recognizer *recognizer, RecognizerState new_state) {
  switch (recognizer->state) {
    case RecognizerState_Possible:
      PBL_ASSERTN((new_state == RecognizerState_Failed) ||
                  (new_state == RecognizerState_Possible) ||
                  (new_state == RecognizerState_Completed) ||
                  (new_state == RecognizerState_Started));
      break;

    case RecognizerState_Started:
      PBL_ASSERTN((new_state == RecognizerState_Possible) ||
                  (new_state == RecognizerState_Cancelled) ||
                  (new_state == RecognizerState_Completed) ||
                  (new_state == RecognizerState_Updated));
      break;

    case RecognizerState_Updated:
      PBL_ASSERTN((new_state == RecognizerState_Possible) ||
                  (new_state == RecognizerState_Cancelled) ||
                  (new_state == RecognizerState_Completed) ||
                  (new_state == RecognizerState_Updated));
      break;

    case RecognizerState_Cancelled:
    case RecognizerState_Completed:
    case RecognizerState_Failed:
      PBL_ASSERTN(new_state == RecognizerState_Possible);
      break;

    default:
      WTF;
  }
  recognizer->state = new_state;
}

static bool prv_should_handle_touches(Recognizer *recognizer, const TouchEvent *touch_event) {
  if (recognizer_get_state(recognizer->fail_after) != RecognizerState_Failed) {
    return false;
  }

  if (!recognizer->subscriber.filter) {
    return true;
  }
  return recognizer->subscriber.filter(recognizer, touch_event);
}

static void prv_send_subscriber_event(Recognizer *recognizer) {
  RecognizerEvent event;
  switch (recognizer->state) {
    case RecognizerState_Started:
      event = RecognizerEvent_Started;
      break;

    case RecognizerState_Updated:
      event = RecognizerEvent_Updated;
      break;

    case RecognizerState_Completed:
      event = RecognizerEvent_Completed;
      break;

    case RecognizerState_Cancelled:
      event = RecognizerEvent_Cancelled;
      break;

    default:
      WTF;
  }
  recognizer->subscriber.event(recognizer, event);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation interface

// TODO: we may need to make wrap these calls for the SDK access so that the asserts on invalid
// data/transitions are not triggered. I'd like to preserve them for internal use to quickly catch
// errors (https://pebbletechnology.atlassian.net/browse/PBL-31359)

static void prv_init_recognizer(Recognizer *recognizer, const RecognizerImpl *impl,
                                const void *data, size_t data_size, RecognizerEventCb event_cb,
                                void *user_data) {
  *recognizer = (Recognizer) {
    .state = RecognizerState_Possible,
    .impl = impl,
    .subscriber = {
      .event = event_cb,
      .data = user_data,
    }
  };
  memcpy(recognizer->impl_data, data, data_size);
}

Recognizer *recognizer_create_with_data(const RecognizerImpl *impl, const void *data,
                                        size_t data_size, RecognizerEventCb event_cb,
                                        void *user_data) {

  // These are passed from the implementation interface, so they must be valid
  PBL_ASSERTN(impl);
  PBL_ASSERTN(impl->handle_touch_event && impl->cancel && impl->reset);
  PBL_ASSERTN(data && (data_size > 0));

  // This might be passed from the public interface, so just return NULL
  if (!event_cb) {
    return NULL;
  }

  // TODO: Use applib_malloc_size to get the size of Recognizer when we have an implementation for
  // 4.x and an API for recognizers
  Recognizer *recognizer = applib_malloc(sizeof(Recognizer) + data_size);
  if (!recognizer) {
    return NULL;
  }
  prv_init_recognizer(recognizer, impl, data, data_size, event_cb, user_data);

  return recognizer;
}

Recognizer *recognizer_init_static_with_data(void *storage, const RecognizerImpl *impl,
                                             const void *data, size_t data_size,
                                             RecognizerEventCb event_cb, void *user_data) {
  PBL_ASSERTN(storage);
  PBL_ASSERTN(impl);
  PBL_ASSERTN(impl->handle_touch_event && impl->cancel && impl->reset);
  PBL_ASSERTN(data && (data_size > 0));

  if (!event_cb) {
    return NULL;
  }

  Recognizer *recognizer = storage;
  prv_init_recognizer(recognizer, impl, data, data_size, event_cb, user_data);
  recognizer->is_static = true;

  return recognizer;
}

void *recognizer_get_impl_data(Recognizer *recognizer, const RecognizerImpl *impl) {
  // NULL-safe: this is reached from SDK-exported typed getters with app-supplied pointers, which
  // must reject bad input rather than assert or crash.
  if (!recognizer || (recognizer->impl != impl)) {
    return NULL;
  }
  return recognizer->impl_data;
}

void recognizer_transition_state(Recognizer *recognizer, RecognizerState new_state) {
  PBL_ASSERTN(recognizer);
  PBL_ASSERTN(new_state < RecognizerStateCount);
  PBL_ASSERTN(new_state != RecognizerState_Possible);

  prv_set_state(recognizer, new_state);
  if (new_state != RecognizerState_Failed) {
    prv_send_subscriber_event(recognizer);
  }
  if (!recognizer->handling_touch_event) {
    recognizer_manager_handle_state_change(recognizer->manager, recognizer);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private interface

void recognizer_handle_touch_event(Recognizer *recognizer, const TouchEvent *touch_event) {
  PBL_ASSERTN(recognizer_is_active(recognizer));
  PBL_ASSERTN(recognizer->impl);
  PBL_ASSERTN(touch_event);

  if (!prv_should_handle_touches(recognizer, touch_event)) {
    return;
  }

  recognizer->handling_touch_event = true;
  recognizer->impl->handle_touch_event(recognizer, touch_event);
  recognizer->handling_touch_event = false;
}

void recognizer_reset(Recognizer *recognizer) {
  PBL_ASSERTN(recognizer && recognizer->impl);

  recognizer_cancel(recognizer);

  recognizer->impl->reset(recognizer);
  prv_set_state(recognizer, RecognizerState_Possible);
  // Only clear the touch-handling flag; preserve is_owned so the recognizer stays linked to its
  // list and is not freed while owned
  recognizer->handling_touch_event = false;
}

void recognizer_cancel(Recognizer *recognizer) {
  PBL_ASSERTN(recognizer && recognizer->impl);
  if (!recognizer_is_active(recognizer)) {
    return;
  }

  if (recognizer->state == RecognizerState_Possible) {
    // Nothing to cancel
    return;
  }

  if (recognizer->impl->cancel(recognizer)) {
    recognizer->subscriber.event(recognizer, RecognizerEvent_Cancelled);
  }
  prv_set_state(recognizer, RecognizerState_Cancelled);
}

void recognizer_set_failed(Recognizer *recognizer) {
  PBL_ASSERTN(recognizer);
  PBL_ASSERTN(recognizer->state == RecognizerState_Possible);

  prv_set_state(recognizer, RecognizerState_Failed);

  if (recognizer->impl->on_fail) {
    recognizer->impl->on_fail(recognizer);
  }
}

void recognizer_set_manager(Recognizer *recognizer, RecognizerManager *manager) {
  PBL_ASSERTN(recognizer);
  recognizer->manager = manager;
}

RecognizerManager *recognizer_get_manager(Recognizer *recognizer) {
  PBL_ASSERTN(recognizer);
  return recognizer->manager;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public interface

RecognizerState recognizer_get_state(const Recognizer *recognizer) {
  if (!recognizer) {
    return RecognizerState_Failed;
  }
  return recognizer->state;
}

void recognizer_set_fail_after(Recognizer *recognizer, Recognizer *fail_after) {
  if (!recognizer || !fail_after) {
    return;
  }

  // Reject any cycle, not just a direct A<->B pair: walking the proposed chain from fail_after
  // must never reach recognizer, or every member would wait on another's failure forever. The
  // walk terminates because this guard keeps existing chains acyclic.
  for (const Recognizer *r = fail_after; r; r = r->fail_after) {
    if (r == recognizer) {
      return;
    }
  }
  recognizer->fail_after = fail_after;
}

Recognizer *recognizer_get_fail_after(const Recognizer *recognizer) {
  if (!recognizer) {
    return NULL;
  }
  return recognizer->fail_after;
}

void recognizer_set_simultaneous_with(Recognizer *recognizer,
                                      RecognizerSimultaneousWithCb simultaneous_with_cb) {
  if (!recognizer || !simultaneous_with_cb) {
    return;
  }
  recognizer->simultaneous_with_cb = simultaneous_with_cb;
}

bool recognizer_should_evaluate_simultaneously(const Recognizer *recognizer,
                                               const Recognizer *test) {
  if (!recognizer || !test || !recognizer->simultaneous_with_cb) {
    return false;
  }
  return recognizer->simultaneous_with_cb(recognizer, test);
}

bool recognizer_is_active(const Recognizer *recognizer) {
  if (!recognizer) {
    return false;
  }
  return !((recognizer->state == RecognizerState_Failed) ||
           (recognizer->state == RecognizerState_Completed) ||
           (recognizer->state == RecognizerState_Cancelled));
}

bool recognizer_has_triggered(const Recognizer *recognizer) {
  if (!recognizer) {
    return false;
  }
  return (recognizer->state > RecognizerState_Possible);
}

void recognizer_set_user_data(Recognizer *recognizer, void *data) {
  if (!recognizer) {
    return;
  }
  recognizer->subscriber.data = data;
}

void *recognizer_get_user_data(const Recognizer *recognizer) {
  if (!recognizer) {
    return NULL;
  }
  return recognizer->subscriber.data;
}

void recognizer_set_touch_filter(Recognizer *recognizer, RecognizerTouchFilterCb filter_cb) {
  if (!recognizer) {
    return;
  }
  recognizer->subscriber.filter = filter_cb;
}

void recognizer_set_on_destroy(Recognizer *recognizer, RecognizerOnDestroyCb on_destroy_cb) {
  if (!recognizer) {
    return;
  }
  recognizer->subscriber.on_destroy = on_destroy_cb;
}

void recognizer_destroy(Recognizer *recognizer) {
  if (!recognizer || recognizer->is_owned) {
    return;
  }
  if (recognizer->subscriber.on_destroy) {
    recognizer->subscriber.on_destroy(recognizer);
  }
  if (recognizer->impl->on_destroy) {
    recognizer->impl->on_destroy(recognizer);
  }

  if (!recognizer->is_static) {
    applib_free(recognizer);
  }
}

bool recognizer_is_owned(Recognizer *recognizer) {
  if (!recognizer) {
    return false;
  }
  return recognizer->is_owned;
}

void recognizer_add_to_list(Recognizer *recognizer, RecognizerList *list) {
  if (!recognizer || !list || recognizer->is_owned) {
    return;
  }

  recognizer->is_owned = true;
  list->node = list_get_head(list_append(list->node, &recognizer->node));
}

void recognizer_remove_from_list(Recognizer *recognizer, RecognizerList *list) {
  if (!recognizer || !list || !recognizer->is_owned) {
    return;
  }

  recognizer->is_owned = false;
  list_remove(&recognizer->node, &list->node, NULL);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Recognizer list

void recognizer_list_init(RecognizerList *list) {
  if (!list) {
    return;
  }
  *list = (RecognizerList){};
}

bool recognizer_list_iterate(RecognizerList *list, RecognizerListIteratorCb iter_cb,
                             void *context) {
  if (!list || !iter_cb) {
    return true;
  }
  ListNode *node = list->node;
  while (node) {
    ListNode *next = list_get_next(node);
    if (!iter_cb((Recognizer *)node, context)) {
      return false;
    }
    node = next;
  }

  return true;
}
