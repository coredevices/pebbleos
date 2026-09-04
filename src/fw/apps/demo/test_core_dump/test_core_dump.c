/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "pbl/kernel/irq.h"
#include "applib/app.h"
#include "applib/ui/ui.h"
#include "kernel/core_dump.h"
#include "kernel/pbl_malloc.h"
#include "kernel/util/sleep.h"
#include "process_management/pebble_process_md.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "pbl/services/new_timer/new_timer.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"



#define NUM_MENU_ITEMS 13

static bool s_call_core_dump_from_isr = false;

// =================================================================================
// Application Data
typedef struct {
  Window *window;
  SimpleMenuLayer *menu_layer;
  SimpleMenuSection menu_section;
  SimpleMenuItem menu_items[NUM_MENU_ITEMS];
} TestTimersAppData;

static TestTimersAppData *s_app_data = 0;


// =================================================================================
static void stuck_timer_callback(void* data)
{
  PBL_LOG_DBG("STT: Entering infinite loop in timer callback");
  while (true) {
    psleep(100);
  }
}

static void stuck_system_task_callback(void *data) {
  PBL_LOG_DBG("Entering infinite loop in system task callback");
  while (true) {
    psleep(100);
  }
}

// ================================================================================
// We install this ISR which does an infinite loop to test that the core dump captures the right
//  task information if we get it while stuck in an ISR
void OTG_FS_WKUP_IRQHandler(void) {
  if (s_call_core_dump_from_isr) {
    core_dump_reset(false /* don't force overwrite */);
  } else {
    dbgserial_putstr("Entering infinite loop in ISR");
    while (true) ;
  }
}

// =================================================================================
// You can capture when the user selects a menu icon with a menu item select callback
static void menu_select_callback(int index, void *ctx) {
  PBL_LOG_DBG("Selected menu item %d", index);
  
  // Here we just change the subtitle to a literal string
  s_app_data->menu_items[index].subtitle = "You've hit select here!";
  
  // Mark the layer to be updated
  layer_mark_dirty(simple_menu_layer_get_layer(s_app_data->menu_layer));
  

  // ---------------------------------------------------------------------------
  // Run the appropriate test
  if (index == 0) {

    // CROAK
    PBL_CROAK("CROAK");

  } else if (index == 1) {

    // stuck timer callback
    TimerID timer = new_timer_create();
    PBL_LOG_INFO("Entering infinite loop in Timer callback");
    bool success = new_timer_start(timer, 100, stuck_timer_callback, NULL, 0 /*flags*/);
    PBL_ASSERTN(success);

  } else if (index == 2) {

    // call directly
    core_dump_reset(false /* don't force overwrite */);

  } else if (index == 3) {

    // stuck app
    PBL_LOG_INFO("Entering infinite loop in App Task");
    while (true) ;

  } else if (index == 4) {

    PBL_LOG_INFO("Entering infinite loop in FreeRTOS ISR");

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_WKUP_IRQn;
    // Lower values are higher priority - make this same or lower priority than a FreeRTOS ISR
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (PBL_IRQ_PRIO_MAX_SYSCALL >> 4);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_EnableIRQ(OTG_FS_WKUP_IRQn);

    // Trigger it. This transfers control to our ISR handler OTG_FS_WKUP_IRQHandler
    NVIC_SetPendingIRQ(OTG_FS_WKUP_IRQn);

  } else if (index == 5) {

    PBL_LOG_INFO("Entering infinite loop in non-FreeRTOS ISR.");

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_WKUP_IRQn;
    // Lower values are higher priority - make this higher priority than a FreeRTOS ISR
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (PBL_IRQ_PRIO_MAX_SYSCALL >> 4) - 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_EnableIRQ(OTG_FS_WKUP_IRQn);

    // Trigger it. This transfers control to our ISR handler OTG_FS_WKUP_IRQHandler
    NVIC_SetPendingIRQ(OTG_FS_WKUP_IRQn);

  } else if (index == 6) {

    PBL_LOG_INFO("Forcing bus fault during core dump");
    core_dump_test_force_bus_fault();
    core_dump_reset(false /* don't force overwrite */);

  } else if (index == 7) {

    PBL_LOG_INFO("Forcing inf loop during core dump");
    core_dump_test_force_inf_loop();
    core_dump_reset(false /* don't force overwrite */);

  } else if (index == 8) {

    PBL_LOG_INFO("Forcing assert loop during core dump");
    core_dump_test_force_assert();
    core_dump_reset(false /* don't force overwrite */);

  } else if (index == 9) {

    PBL_LOG_INFO("Calling core_dump FreeRTOS ISR");
    s_call_core_dump_from_isr = true;

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_WKUP_IRQn;
    // Lower values are higher priority - make this same or lower priority than a FreeRTOS ISR
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (PBL_IRQ_PRIO_MAX_SYSCALL >> 4);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_EnableIRQ(OTG_FS_WKUP_IRQn);

    // Trigger it. This transfers control to our ISR handler OTG_FS_WKUP_IRQHandler
    NVIC_SetPendingIRQ(OTG_FS_WKUP_IRQn);

  } else if (index == 10) {

    PBL_LOG_INFO("Causing bus fault in app");
    typedef void (*KaboomCallback)(void);
    KaboomCallback kaboom = 0;
    kaboom();

  } else if (index == 11) {
    PBL_LOG_INFO("Infinite Loop on system task");
    system_task_add_callback(stuck_system_task_callback, NULL);

  } else if (index == 12) {
    PBL_LOG_INFO("Generate hard-fault");

    // Modify behavior of the ARM so that bus faults generate a hard fault
    SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;

    // Write to protected memory
    extern uint32_t __isr_stack_start__[];
    __isr_stack_start__[0] = 0x55;
  }

}



// =================================================================================
static void prv_window_load(Window *window) {
  
  TestTimersAppData *data = s_app_data;
  
  int i = 0;
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "croak",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "stuck timer",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "call core_dump_reset",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "stuck app",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "stuck RTOS ISR",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "stuck non-RTOS ISR",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "BusFault in CD",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "stuck in CD",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "assert in CD",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "call from ISR",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "BusFault in app",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "stuck system task",
    .callback = menu_select_callback,
  };
  data->menu_items[i++] = (SimpleMenuItem) {
    .title = "hard fault",
    .callback = menu_select_callback,
  };
  PBL_ASSERTN(i == NUM_MENU_ITEMS);
  
  // The menu sections
  data->menu_section = (SimpleMenuSection) {
    .num_items = NUM_MENU_ITEMS,
    .items = data->menu_items,
  };
  
  Layer *window_layer = window_get_root_layer(data->window);
  GRect bounds = window_layer->bounds;
  
  data->menu_layer = simple_menu_layer_create(bounds, data->window, &data->menu_section, 1, 
                                              NULL);
  layer_add_child(window_layer, simple_menu_layer_get_layer(data->menu_layer));
}


// =================================================================================
// Deinitialize resources on window unload that were initialized on window load
static void prv_window_unload(Window *window) {
  simple_menu_layer_destroy(s_app_data->menu_layer);
}


// =================================================================================
static void handle_init(void) {
  TestTimersAppData *data = app_malloc_check(sizeof(TestTimersAppData));
  memset(data, 0, sizeof(TestTimersAppData));
  s_app_data = data;

  data->window = window_create();
  if (data->window == NULL) {
    return;
  }
  window_init(data->window, "");
  window_set_window_handlers(data->window, &(WindowHandlers) {
    .load = prv_window_load,
    .unload = prv_window_unload,
  });
  app_window_stack_push(data->window, true /*animated*/);
}

static void handle_deinit(void) {
  // Don't bother freeing anything, the OS should be re-initing the heap.
}


// =================================================================================
static void s_main(void) {
  handle_init();
  app_event_loop();
  handle_deinit();
}

// =================================================================================
const PebbleProcessMd* test_core_dump_app_get_info() {
  static const PebbleProcessMdSystem s_app_info = {
    .common.main_func = &s_main,
    .name = "Core Dump Test"
  };
  return (const PebbleProcessMd*) &s_app_info;
}

