/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "applib/ui/recognizer/recognizer.h"
#include "applib/ui/recognizer/recognizer_impl.h"
#include "applib/ui/recognizer/recognizer_manager.h"
#include "applib/ui/recognizer/recognizer_private.h"
#include "pbl/util/size.h"

// Stubs
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"
#include "stubs_logging.h"
#include "test_recognizer_impl.h"

static bool s_manager_state_change = false;
void recognizer_manager_handle_state_change(RecognizerManager *manager, Recognizer *changed) {
  s_manager_state_change = true;
}

static TestImplData s_test_impl_data;

// setup and teardown
void test_recognizer__initialize(void) {
  s_test_impl_data = (TestImplData) {};
}

void test_recognizer__cleanup(void) {

}

// tests
void test_recognizer__create_with_data(void) {
  int sub_data;
  void *dummy = &sub_data;
  RecognizerImpl s_test_impl = {
    .handle_touch_event = dummy,
    .cancel = dummy,
    .reset = dummy
  };
  Recognizer *r = recognizer_create_with_data(&s_test_impl, &s_test_impl_data,
                                              sizeof(s_test_impl_data), dummy,
                                              &sub_data);
  cl_assert(r != NULL);
  cl_assert_equal_p(r->impl, &s_test_impl);
  cl_assert_equal_m(r->impl_data, &s_test_impl_data, sizeof(s_test_impl_data));
  cl_assert_equal_p(r->subscriber.event, dummy);
  cl_assert_equal_p(r->subscriber.data, &sub_data);
  cl_assert_equal_i(r->state, RecognizerState_Possible);
  cl_assert_equal_i(r->flags, 0);
  cl_assert_equal_p(r->simultaneous_with_cb, NULL);
  cl_assert_equal_p(r->fail_after, NULL);

  cl_assert_passert(recognizer_create_with_data(NULL, &s_test_impl_data,
                                                sizeof(s_test_impl_data), dummy,
                                                NULL));
  cl_assert_passert(recognizer_create_with_data(&s_test_impl, NULL,
                                                sizeof(s_test_impl_data), dummy,
                                                NULL));
  cl_assert_passert(recognizer_create_with_data(&s_test_impl, &s_test_impl_data,
                                                0, dummy, NULL));
  cl_assert_equal_p(NULL, recognizer_create_with_data(&s_test_impl, &s_test_impl_data,
                                                      sizeof(s_test_impl_data), NULL, NULL));
  s_test_impl.handle_touch_event = NULL;
  cl_assert_passert(recognizer_create_with_data(&s_test_impl, &s_test_impl_data,
                                                sizeof(s_test_impl_data), dummy,
                                                NULL));
  s_test_impl.handle_touch_event = dummy;
  s_test_impl.reset = NULL;
  cl_assert_passert(recognizer_create_with_data(&s_test_impl, &s_test_impl_data,
                                                sizeof(s_test_impl_data), dummy,
                                                NULL));
  s_test_impl.reset = dummy;
  s_test_impl.cancel = NULL;
  cl_assert_passert(recognizer_create_with_data(&s_test_impl, &s_test_impl_data,
                                                sizeof(s_test_impl_data), dummy,
                                                NULL));

  recognizer_destroy(r);
}

void test_recognizer__transition_state(void) {
  RecognizerEvent event_type = -1;
  NEW_RECOGNIZER(r) = test_recognizer_create(&s_test_impl_data, &event_type);
  // Test that manager state change handler is called when not called from a touch event handler
  recognizer_transition_state(r, RecognizerState_Failed);
  cl_assert_equal_i(r->state, RecognizerState_Failed);
  cl_assert(s_manager_state_change);
  cl_assert_equal_i(event_type, -1);

  r->state = RecognizerState_Possible;
  s_manager_state_change = false;
  recognizer_transition_state(r, RecognizerState_Completed);
  cl_assert_equal_i(r->state, RecognizerState_Completed);
  cl_assert(s_manager_state_change);
  cl_assert_equal_i(event_type, RecognizerEvent_Completed);

  s_manager_state_change = false;
  r->handling_touch_event = true;
  r->state = RecognizerState_Possible;
  event_type = -1;
  recognizer_transition_state(r, RecognizerState_Failed);
  cl_assert(!s_manager_state_change);
  cl_assert_equal_i(r->state, RecognizerState_Failed);
  cl_assert_equal_i(event_type, -1);

  // Test that invalid state transitions get caught by asserts
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Completed));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Started));

  r->state = RecognizerState_Possible;
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Possible));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Updated));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Cancelled));

  recognizer_transition_state(r, RecognizerState_Started);
  cl_assert_equal_i(r->state, RecognizerState_Started);
  cl_assert_equal_i(event_type, RecognizerEvent_Started);
  cl_assert(!s_manager_state_change);
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Failed));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Possible));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Started));

  recognizer_transition_state(r, RecognizerState_Updated);
  cl_assert_equal_i(r->state, RecognizerState_Updated);
  cl_assert_equal_i(event_type, RecognizerEvent_Updated);
  cl_assert(!s_manager_state_change);
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Failed));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Possible));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Started));

  event_type = -1;
  recognizer_transition_state(r, RecognizerState_Updated);
  cl_assert_equal_i(event_type, RecognizerEvent_Updated);
  cl_assert_equal_i(r->state, RecognizerState_Updated);
  cl_assert(!s_manager_state_change);

  recognizer_transition_state(r, RecognizerState_Completed);
  cl_assert_equal_i(event_type, RecognizerEvent_Completed);
  cl_assert_equal_i(r->state, RecognizerState_Completed);
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Failed));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Possible));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Started));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Updated));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Cancelled));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Completed));

  r->state = RecognizerState_Updated;
  recognizer_transition_state(r, RecognizerState_Cancelled);
  cl_assert_equal_i(event_type, RecognizerEvent_Cancelled);
  cl_assert_equal_i(r->state, RecognizerState_Cancelled);
  cl_assert(!s_manager_state_change);
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Failed));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Possible));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Started));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Updated));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Cancelled));
  cl_assert_passert(recognizer_transition_state(r, RecognizerState_Completed));

  r->state = RecognizerState_Started;
  event_type = -1;
  recognizer_transition_state(r, RecognizerState_Cancelled);
  cl_assert_equal_i(event_type, RecognizerEvent_Cancelled);
  cl_assert_equal_i(r->state, RecognizerState_Cancelled);
  cl_assert(!s_manager_state_change);

  r->state = RecognizerState_Started;
  recognizer_transition_state(r, RecognizerState_Completed);
  cl_assert_equal_i(event_type, RecognizerEvent_Completed);
  cl_assert_equal_i(r->state, RecognizerState_Completed);
  cl_assert(!s_manager_state_change);

  r->state = RecognizerState_Possible;
  recognizer_transition_state(r, RecognizerState_Completed);
  cl_assert_equal_i(r->state, RecognizerState_Completed);
  cl_assert(!s_manager_state_change);
}

void test_recognizer__set_failed(void) {
  bool failed = false;
  s_test_impl_data.failed = &failed;
  NEW_RECOGNIZER(r) = test_recognizer_create(&s_test_impl_data, NULL);

  recognizer_set_failed(r);
  cl_assert_equal_i(r->state, RecognizerState_Failed);
  cl_assert(failed);

  // Failed -> Failed invalid transition
  cl_assert_passert(recognizer_set_failed(r));

  // (!Possible) -> Failed invalid transition
  r->state = RecognizerState_Started;
  cl_assert_passert(recognizer_set_failed(r));

  r->state = RecognizerState_Completed;
  cl_assert_passert(recognizer_set_failed(r));
}

static void prv_sub_destroy(const Recognizer *r) {
  bool *destroyed = recognizer_get_user_data(r);
  *destroyed = true;
}

void test_recognizer__destroy(void) {
  bool impl_destroyed = false;
  s_test_impl_data.destroyed = &impl_destroyed;

  bool sub_destroyed = false;
  Recognizer *r = test_recognizer_create(&s_test_impl_data, &sub_destroyed);
  test_recognizer_enable_on_destroy();
  recognizer_set_on_destroy(r, prv_sub_destroy);

  // can't destroy a recognizer if it is owned
  r->is_owned = true;
  recognizer_destroy(r);
  cl_assert_equal_b(impl_destroyed, false);
  cl_assert_equal_b(sub_destroyed, false);

  r->is_owned = false;
  recognizer_destroy(r);
  cl_assert_equal_b(impl_destroyed, true);
  cl_assert_equal_b(sub_destroyed, true);
}

void test_recognizer__reset(void) {
  bool reset = false;
  bool cancelled = false;
  s_test_impl_data.reset = &reset;
  s_test_impl_data.cancelled = &cancelled;

  NEW_RECOGNIZER(r) = test_recognizer_create(&s_test_impl_data, NULL);
  recognizer_reset(r);
  cl_assert_equal_i(r->state, RecognizerState_Possible);
  cl_assert_equal_b(reset, true);
  cl_assert_equal_b(cancelled, false);

  reset = false;
  r->state = RecognizerState_Failed;
  recognizer_reset(r);
  cl_assert_equal_i(r->state, RecognizerState_Possible);
  cl_assert_equal_b(reset, true);
  cl_assert_equal_b(cancelled, false);

  reset = false;
  r->state = RecognizerState_Cancelled;
  recognizer_reset(r);
  cl_assert_equal_i(r->state, RecognizerState_Possible);
  cl_assert_equal_b(reset, true);
  cl_assert_equal_b(cancelled, false);

  reset = false;
  r->state = RecognizerState_Completed;
  recognizer_reset(r);
  cl_assert_equal_i(r->state, RecognizerState_Possible);
  cl_assert_equal_b(reset, true);
  cl_assert_equal_b(cancelled, false);

  reset = false;
  r->state = RecognizerState_Started;
  recognizer_reset(r);
  cl_assert_equal_i(r->state, RecognizerState_Possible);
  cl_assert_equal_b(reset, true);
  cl_assert_equal_b(cancelled, true);

  reset = false;
  cancelled = false;
  r->state = RecognizerState_Updated;
  recognizer_reset(r);
  cl_assert_equal_i(r->state, RecognizerState_Possible);
  cl_assert_equal_b(reset, true);
  cl_assert_equal_b(cancelled, true);
}

void test_recognizer__reset_preserves_owned(void) {
  RecognizerList list = { NULL };
  NEW_RECOGNIZER(r) = test_recognizer_create(&s_test_impl_data, NULL);

  recognizer_add_to_list(r, &list);
  cl_assert(r->is_owned);
  cl_assert(list_contains(list.node, &r->node));

  // Reset must not clear is_owned (it used to wipe the whole flags word)
  recognizer_reset(r);
  cl_assert(r->is_owned);
  cl_assert(list_contains(list.node, &r->node));

  // With ownership preserved, removal actually unlinks the node
  recognizer_remove_from_list(r, &list);
  cl_assert(!r->is_owned);
  cl_assert(!list_contains(list.node, &r->node));
  cl_assert_equal_p(list.node, NULL);
}

void test_recognizer__reset_owned_not_freed_on_destroy(void) {
  bool impl_destroyed = false;
  s_test_impl_data.destroyed = &impl_destroyed;

  RecognizerList list = { NULL };
  Recognizer *r = test_recognizer_create(&s_test_impl_data, NULL);
  test_recognizer_enable_on_destroy();

  recognizer_add_to_list(r, &list);

  // Reset while owned used to clear is_owned, letting destroy free a still-linked node
  recognizer_reset(r);
  recognizer_destroy(r);
  cl_assert_equal_b(impl_destroyed, false);
  cl_assert(r->is_owned);
  cl_assert(list_contains(list.node, &r->node));

  // Detaching first leaves no freed-but-linked node behind
  recognizer_remove_from_list(r, &list);
  cl_assert_equal_p(list.node, NULL);
  recognizer_destroy(r);
  cl_assert_equal_b(impl_destroyed, true);
}

static void prv_static_event(const Recognizer *recognizer, RecognizerEvent event) {
}

static bool s_static_impl_destroyed;
static void prv_static_impl_on_destroy(Recognizer *recognizer) {
  s_static_impl_destroyed = true;
}

static void prv_static_sub_on_destroy(const Recognizer *recognizer) {
  bool *destroyed = recognizer_get_user_data(recognizer);
  *destroyed = true;
}

static void prv_static_handle(Recognizer *recognizer, const TouchEvent *touch_event) {
}

static bool prv_static_cancel(Recognizer *recognizer) {
  return false;
}

static void prv_static_reset(Recognizer *recognizer) {
}

void test_recognizer__init_static(void) {
  RecognizerImpl impl = {
    .handle_touch_event = prv_static_handle,
    .cancel = prv_static_cancel,
    .reset = prv_static_reset,
    .on_destroy = prv_static_impl_on_destroy,
  };
  int impl_data = 42;
  bool sub_destroyed = false;
  s_static_impl_destroyed = false;

  // Stack storage - if destroy erroneously frees it, free() would abort the test
  RECOGNIZER_STATIC_STORAGE(storage, sizeof(impl_data));
  Recognizer *r = recognizer_init_static_with_data(storage, &impl, &impl_data, sizeof(impl_data),
                                                   prv_static_event, &sub_destroyed);
  cl_assert(r != NULL);
  cl_assert_equal_p(r, (Recognizer *)storage);
  cl_assert_equal_i(r->state, RecognizerState_Possible);
  cl_assert(r->is_static);
  cl_assert(!r->is_owned);
  cl_assert_equal_p(r->impl, &impl);
  cl_assert_equal_i(*(int *)recognizer_get_impl_data(r, &impl), 42);

  recognizer_set_on_destroy(r, prv_static_sub_on_destroy);

  // Both destructors run, but the caller storage is not freed
  recognizer_destroy(r);
  cl_assert(s_static_impl_destroyed);
  cl_assert(sub_destroyed);
  // Storage still valid (not freed/scrubbed)
  cl_assert_equal_p(r->impl, &impl);

  // Invalid args are still rejected
  cl_assert_passert(recognizer_init_static_with_data(NULL, &impl, &impl_data, sizeof(impl_data),
                                                     prv_static_event, NULL));
  cl_assert_equal_p(NULL, recognizer_init_static_with_data(storage, &impl, &impl_data,
                                                           sizeof(impl_data), NULL, NULL));
}

void test_recognizer__cancel(void) {
  bool cancelled = false;
  s_test_impl_data.cancelled = &cancelled;
  RecognizerEvent rec_event = -1;

  NEW_RECOGNIZER(r) = test_recognizer_create(&s_test_impl_data, &rec_event);
  recognizer_cancel(r);
  cl_assert_equal_i(r->state, RecognizerState_Possible);
  cl_assert_equal_b(cancelled, false);
  cl_assert_equal_i(rec_event, -1);

  r->state = RecognizerState_Failed;
  recognizer_cancel(r);
  cl_assert_equal_i(r->state, RecognizerState_Failed);
  cl_assert_equal_b(cancelled, false);
  cl_assert_equal_i(rec_event, -1);

  r->state = RecognizerState_Cancelled;
  recognizer_cancel(r);
  cl_assert_equal_i(r->state, RecognizerState_Cancelled);
  cl_assert_equal_b(cancelled, false);
  cl_assert_equal_i(rec_event, -1);

  r->state = RecognizerState_Completed;
  recognizer_cancel(r);
  cl_assert_equal_i(r->state, RecognizerState_Completed);
  cl_assert_equal_b(cancelled, false);
  cl_assert_equal_i(rec_event, -1);

  r->state = RecognizerState_Started;
  recognizer_cancel(r);
  cl_assert_equal_i(r->state, RecognizerState_Cancelled);
  cl_assert_equal_b(cancelled, true);
  cl_assert_equal_i(rec_event, RecognizerEvent_Cancelled);

  cancelled = false;
  r->state = RecognizerState_Updated;
  rec_event = -1;
  recognizer_cancel(r);
  cl_assert_equal_i(r->state, RecognizerState_Cancelled);
  cl_assert_equal_b(cancelled, true);
  cl_assert_equal_i(rec_event, RecognizerEvent_Cancelled);
}

void test_recognizer__handle_touch_events(void) {
  RecognizerEvent rec_event = -1;
  TouchEvent last_touch_event = { .type = TouchEvent_Liftoff };
  RecognizerState new_state;
  bool updated = false;
  s_test_impl_data.last_touch_event = &last_touch_event;
  s_test_impl_data.new_state = &new_state;
  s_test_impl_data.updated = &updated;
  NEW_RECOGNIZER(r) = test_recognizer_create(&s_test_impl_data, &rec_event);

  new_state = RecognizerState_Possible;
  recognizer_handle_touch_event(r, &(TouchEvent) { .type = TouchEvent_Touchdown });
  cl_assert_equal_i(last_touch_event.type, TouchEvent_Touchdown);
  cl_assert_equal_b(updated, false);

  new_state = RecognizerState_Completed;
  recognizer_handle_touch_event(r, &(TouchEvent) { .type = TouchEvent_Liftoff });
  cl_assert_equal_i(last_touch_event.type, TouchEvent_Liftoff);
  cl_assert_equal_b(updated, true);
  cl_assert_equal_i(rec_event, RecognizerEvent_Completed);

  r->state = RecognizerState_Possible;
  updated = false;
  new_state = RecognizerState_Started;
  recognizer_handle_touch_event(r, &(TouchEvent) { .type = TouchEvent_Touchdown });
  cl_assert_equal_i(last_touch_event.type, TouchEvent_Touchdown);
  cl_assert_equal_b(updated, true);
  cl_assert_equal_i(rec_event, RecognizerEvent_Started);

  updated = false;
  new_state = RecognizerState_Updated;
  recognizer_handle_touch_event(r, &(TouchEvent) { .type = TouchEvent_PositionUpdate });
  cl_assert_equal_i(last_touch_event.type, TouchEvent_PositionUpdate);
  cl_assert_equal_b(updated, true);
  cl_assert_equal_i(rec_event, RecognizerEvent_Updated);

  updated = false;
  new_state = RecognizerState_Cancelled;
  recognizer_handle_touch_event(r, &(TouchEvent) { .type = TouchEvent_Liftoff });
  cl_assert_equal_i(last_touch_event.type, TouchEvent_Liftoff);
  cl_assert_equal_b(updated, true);
  cl_assert_equal_i(rec_event, RecognizerEvent_Cancelled);

  // Should not pass touch events to recognizers that are not active
  cl_assert_passert(recognizer_handle_touch_event(r, &(TouchEvent) {}));

  // Should not pass null touch events
  r->state = RecognizerState_Possible;
  cl_assert_passert(recognizer_handle_touch_event(r, NULL));
}

void test_recognizer__handle_touch_events_fail_after(void) {
  RecognizerEvent rec_event = -1;
  RecognizerState new_state;
  bool updated = false;
  TouchEvent last_touch_event = { .type = TouchEvent_Liftoff };
  s_test_impl_data.new_state = &new_state;
  s_test_impl_data.updated = &updated;
  s_test_impl_data.last_touch_event = &last_touch_event;

  NEW_RECOGNIZER(r) = test_recognizer_create(&s_test_impl_data, &rec_event);
  NEW_RECOGNIZER(fail) = test_recognizer_create(&s_test_impl_data, NULL);
  recognizer_set_fail_after(r, fail);

  new_state = RecognizerState_Completed;
  recognizer_handle_touch_event(r, &(TouchEvent) { .type = TouchEvent_Touchdown });
  cl_assert_equal_i(last_touch_event.type, TouchEvent_Liftoff);
  cl_assert_equal_b(updated, false);

  fail->state = RecognizerState_Failed;
  recognizer_handle_touch_event(r, &(TouchEvent) { .type = TouchEvent_Touchdown });
  cl_assert_equal_i(last_touch_event.type, TouchEvent_Touchdown);
  cl_assert_equal_b(updated, true);
  cl_assert_equal_i(rec_event, RecognizerEvent_Completed);
}

static bool prv_filter(const Recognizer *recognizer, const TouchEvent *touch_event) {
  bool *allow = recognizer_get_user_data(recognizer);
  return *allow;
}

void test_recognizer__handle_touch_events_filter_cb(void) {
  RecognizerState new_state;
  bool updated = false;
  TouchEvent last_touch_event = { .type = TouchEvent_Liftoff };
  s_test_impl_data.new_state = &new_state;
  s_test_impl_data.updated = &updated;
  s_test_impl_data.last_touch_event = &last_touch_event;

  bool allow = false;
  NEW_RECOGNIZER(r) = test_recognizer_create(&s_test_impl_data, &allow);
  recognizer_set_touch_filter(r, prv_filter);

  new_state = RecognizerState_Completed;
  recognizer_handle_touch_event(r, &(TouchEvent) { .type = TouchEvent_Touchdown });
  cl_assert_equal_i(last_touch_event.type, TouchEvent_Liftoff);
  cl_assert_equal_b(updated, false);

  allow = true;
  recognizer_handle_touch_event(r, &(TouchEvent) { .type = TouchEvent_Touchdown });
  cl_assert_equal_i(last_touch_event.type, TouchEvent_Touchdown);
  cl_assert_equal_b(updated, true);
  cl_assert_equal_i(r->state, new_state);
}

bool s_simultaneous = false;
static bool prv_simultaneous_with_cb(const Recognizer *recognizer,
                                     const Recognizer *simultaneous_with) {
  return s_simultaneous;
}

void test_recognizer__set_simultaneous_with(void) {
  NEW_RECOGNIZER(r1) = test_recognizer_create(&s_test_impl_data, NULL);
  NEW_RECOGNIZER(r2) = test_recognizer_create(&s_test_impl_data, NULL);

  s_simultaneous = false;
  cl_assert(!recognizer_should_evaluate_simultaneously(NULL, NULL));
  cl_assert(!recognizer_should_evaluate_simultaneously(r1, NULL));
  cl_assert(!recognizer_should_evaluate_simultaneously(NULL, r2));
  cl_assert(!recognizer_should_evaluate_simultaneously(r1, r2));

  recognizer_set_simultaneous_with(r1, prv_simultaneous_with_cb);
  cl_assert(!recognizer_should_evaluate_simultaneously(NULL, NULL));
  cl_assert(!recognizer_should_evaluate_simultaneously(r1, NULL));
  cl_assert(!recognizer_should_evaluate_simultaneously(NULL, r2));
  cl_assert(!recognizer_should_evaluate_simultaneously(r1, r2));

  s_simultaneous = true;
  cl_assert(!recognizer_should_evaluate_simultaneously(NULL, NULL));
  cl_assert(!recognizer_should_evaluate_simultaneously(r1, NULL));
  cl_assert(!recognizer_should_evaluate_simultaneously(NULL, r2));
  cl_assert(recognizer_should_evaluate_simultaneously(r1, r2));
}

void test_recognizer__add_remove_list(void) {
  RecognizerList list = { NULL };
  NEW_RECOGNIZER(r1) = test_recognizer_create(&s_test_impl_data, NULL);
  NEW_RECOGNIZER(r2) = test_recognizer_create(&s_test_impl_data, NULL);

  recognizer_add_to_list(r1, &list);
  recognizer_add_to_list(r2, &list);
  cl_assert_equal_i(list_count(list.node), 2);
  cl_assert(list_contains(list.node, &r1->node));
  cl_assert(list_contains(list.node, &r2->node));
  cl_assert(r1->is_owned);
  cl_assert(r2->is_owned);

  recognizer_remove_from_list(r1, &list);
  cl_assert(!list_contains(list.node, &r1->node));
  cl_assert(!r1->is_owned);

  recognizer_remove_from_list(r1, &list);
  cl_assert(!r1->is_owned);
}

static int s_list_idx = 0;
static bool prv_list_iterator(Recognizer *recognizer, void *context) {
  const char *names[] = { "R1", "R2", "R3" };
  cl_assert(s_list_idx < ARRAY_LENGTH(names));
  char s[20];
  snprintf(s, sizeof(s), "%s != %s", recognizer->subscriber.data, names[s_list_idx]);
  cl_assert_(strcmp(recognizer->subscriber.data, names[s_list_idx++]) == 0, s);

  return (s_list_idx < *((int *)context));
}

void test_recognizer__list_iterate(void) {
  NEW_RECOGNIZER(r1) = test_recognizer_create(&s_test_impl_data, "R1");
  NEW_RECOGNIZER(r2) = test_recognizer_create(&s_test_impl_data, "R2");
  NEW_RECOGNIZER(r3) = test_recognizer_create(&s_test_impl_data, "R3");

  RecognizerList list = { NULL };
  recognizer_add_to_list(r1, &list);
  recognizer_add_to_list(r2, &list);
  recognizer_add_to_list(r3, &list);

  s_list_idx = 0;
  int end = 4;
  recognizer_list_iterate(&list, prv_list_iterator, &end);
  cl_assert_equal_i(s_list_idx, 3);

  end = 2;
  s_list_idx = 0;
  recognizer_list_iterate(&list, prv_list_iterator, &end);
  cl_assert_equal_i(s_list_idx, 2);
}

void test_recognizer__set_fail_after_rejects_cycles(void) {
  NEW_RECOGNIZER(a) = test_recognizer_create(&s_test_impl_data, NULL);
  NEW_RECOGNIZER(b) = test_recognizer_create(&s_test_impl_data, NULL);
  NEW_RECOGNIZER(c) = test_recognizer_create(&s_test_impl_data, NULL);

  recognizer_set_fail_after(a, b);
  recognizer_set_fail_after(b, c);
  cl_assert_equal_p(a->fail_after, b);
  cl_assert_equal_p(b->fail_after, c);

  // Direct two-cycle rejected: b -> a would close a <-> b.
  recognizer_set_fail_after(b, a);
  cl_assert_equal_p(b->fail_after, c);

  // Transitive cycle rejected too: c -> a would close a -> b -> c -> a, and every member of the
  // ring would wait forever on another's failure.
  recognizer_set_fail_after(c, a);
  cl_assert_equal_p(c->fail_after, NULL);
}

void test_recognizer__get_impl_data_rejects_bad_input(void) {
  NEW_RECOGNIZER(r) = test_recognizer_create(&s_test_impl_data, NULL);
  static const RecognizerImpl s_other_impl = {};

  // NULL recognizer: reachable from the SDK typed getters with app input; reject, don't assert.
  cl_assert_equal_p(recognizer_get_impl_data(NULL, r->impl), NULL);
  // Wrong impl: NULL rather than the raw impl_data of a different recognizer type.
  cl_assert_equal_p(recognizer_get_impl_data(r, &s_other_impl), NULL);
  // Matching impl still resolves.
  cl_assert(recognizer_get_impl_data(r, r->impl) != NULL);
}
