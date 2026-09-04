/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/debounced_button.h>

#include "board/board.h"
#include <pbl/drivers/button.h>
#include <pbl/drivers/exti.h>
#include "kernel/events.h"
#include "system/bootbits.h"
#include "system/reset.h"
#include "util/bitset.h"

#include <nrfx.h>

#include "projdefs.h"

// We want TIM4 to run at 32KHz
static const uint32_t TIMER_FREQUENCY_HZ = 31250;
// Sample the buttons every 2ms to debounce
static const uint32_t TIMER_PERIOD_TICKS = 64;
// A button must be stable for 20 samples (40ms) to be accepted.
static const uint32_t NUM_DEBOUNCE_SAMPLES = 20;

#define RESET_BUTTONS ((1 << BUTTON_ID_SELECT) | (1 << BUTTON_ID_BACK))

#define DEBOUNCE_SAMPLES_PER_SECOND (TIMER_FREQUENCY_HZ / TIMER_PERIOD_TICKS)

// This reset-buttons-held timeout must be lower than the PMIC's back-button-reset timeout,
// which is ~8-11s. The spacing between these timeouts should be large enough to avoid accidentally
// shutting down the device when a customer is attempting to reset. Therefore the FW's
// reset-buttons-held timeout is set to 5 seconds:
#define RESET_THRESHOLD_SAMPLES (5 * DEBOUNCE_SAMPLES_PER_SECOND)

static void prv_timer_handler(nrf_timer_event_t evt, void *ctx);

static void initialize_button_timer(void) {
  nrfx_timer_config_t config = {
    .frequency = TIMER_FREQUENCY_HZ,
    .mode = NRF_TIMER_MODE_TIMER,
    .bit_width = NRF_TIMER_BIT_WIDTH_32,
    .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
  };
  nrfx_timer_init(&BOARD_CONFIG_BUTTON.timer, &config, prv_timer_handler);
  nrfx_timer_extended_compare(&BOARD_CONFIG_BUTTON.timer, NRF_TIMER_CC_CHANNEL0, TIMER_PERIOD_TICKS, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true /* enable interrupt */);
}

static bool prv_check_timer_enabled(void) {
  return nrfx_timer_is_enabled(&BOARD_CONFIG_BUTTON.timer);
}

static void disable_button_timer(void) {
  if (prv_check_timer_enabled()) {
    nrfx_timer_disable(&BOARD_CONFIG_BUTTON.timer);
  }
}

static void prv_enable_button_timer(void) {
  // Don't let the timer interrupt us while we're mucking with it.
  __disable_irq();
  if (!prv_check_timer_enabled()) {
    nrfx_timer_enable(&BOARD_CONFIG_BUTTON.timer);
    nrfx_timer_resume(&BOARD_CONFIG_BUTTON.timer);
  }
  __enable_irq();
}

static void prv_button_interrupt_handler(bool *should_context_switch) {
  prv_enable_button_timer();
}

static void clear_stuck_button(ButtonId button_id) {
  __disable_irq();
#if 0
  const uint32_t button_counter_register = RTC_ReadBackupRegister(STUCK_BUTTON_REGISTER);
  if (button_counter_register != 0) {
    // Create bitmask with all 1s, except on the counter byte for this button_id in button_counter_register. AND to mask out:
    const uint32_t updated_button_counter_register = button_counter_register & ~(0xff << (button_id << 3));
    if (button_counter_register != updated_button_counter_register) {
      RTC_WriteBackupRegister(STUCK_BUTTON_REGISTER, updated_button_counter_register);
    }
  }
#endif
  __enable_irq();
}

void debounced_button_init(void) {
  button_init();

  for (int i = 0; i < NUM_BUTTONS; ++i) {
    const ExtiConfig config = BOARD_CONFIG_BUTTON.buttons[i].gpiote;
    exti_configure_pin(config, ExtiTrigger_RisingFalling, prv_button_interrupt_handler);
    exti_enable(config);
  }

  initialize_button_timer();

  // If someone is holding down a button, we need to start up the timer immediately ourselves as
  // we won't get a button down interrupt to start it.
  if (button_get_state_bits() != 0) {
     prv_enable_button_timer();
  }
}


// Interrupt Service Routines
///////////////////////////////////////////////////////////
static void prv_timer_handler(nrf_timer_event_t evt, void *ctx) {
  // This array holds the number of samples we have for the button being in a different state than
  // the current debounced state of the button.
  static uint32_t s_button_timers[] = {0, 0, 0, 0};
  // A bitset of the current states of the buttons after the debouncing is done.
  static uint32_t s_debounced_button_state = 0;

  // Should we tell the scheduler to attempt to context switch after this function has completed?
  bool should_context_switch = pdFALSE;
  // Should we power down this interrupt timer once we're done here or should we leave it on?
  bool can_power_down_tim4 = true;

  // We handle all 4 buttons every time this interrupt is fired.
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    // What stable state is the button in, according to the debouncing algorithm?
    bool debounced_button_state = bitset32_get(&s_debounced_button_state, i);
    // What is the current physical state of the button?
    bool is_pressed = button_is_pressed(i);

    if (is_pressed == debounced_button_state) {
      // If the state is not changing, skip this button.
      s_button_timers[i] = 0;
      continue;
    }

    // Leave the timer running so we can track this button that's changing state.
    can_power_down_tim4 = false;

    s_button_timers[i] += 1;

    // If the button has been in a stable state that's different than the debounced state for enough
    // samples, change the debounced state to the stable state and generate an event.
    if (s_button_timers[i] == NUM_DEBOUNCE_SAMPLES) {
      s_button_timers[i] = 0;

      bitset32_update(&s_debounced_button_state, i, is_pressed);

      if (!is_pressed) {
        // A button has been released. Make sure we weren't tracking this as a stuck button.
        clear_stuck_button(i);
      }

      PebbleEvent e = {
        .type = (is_pressed) ? PEBBLE_BUTTON_DOWN_EVENT : PEBBLE_BUTTON_UP_EVENT,
        .button.button_id = i
      };
      should_context_switch = event_put_isr(&e);
    }
  }

#if !defined(CONFIG_MFG)
  // Now that s_debounced_button_state is updated, check to see if the user is holding down the reset
  // combination.
  static uint32_t s_hard_reset_timer = 0;
  if ((s_debounced_button_state & RESET_BUTTONS) == RESET_BUTTONS) {
    s_hard_reset_timer += 1;
    can_power_down_tim4 = false;

    if (s_hard_reset_timer > RESET_THRESHOLD_SAMPLES) {
      __disable_irq();

      // If the UP button is held at the moment the timeout is hit, set the force-PRF bootbit:
      const bool force_prf = (s_debounced_button_state & (1 << BUTTON_ID_UP));
      if (force_prf) {
        boot_bit_set(BOOT_BIT_FORCE_PRF);
      }

      RebootReason reason = {
        .code = force_prf ? RebootReasonCode_PrfResetButtonsHeld :
                            RebootReasonCode_ResetButtonsHeld
      };
      reboot_reason_set(&reason);

      // Don't use system_reset here. This back door absolutely must work. Just hard reset.
      system_hard_reset();
    }
  } else {
    s_hard_reset_timer = 0;
  }
#endif


  if (can_power_down_tim4) {
    __disable_irq();
    disable_button_timer();
    __enable_irq();
  }

  portEND_SWITCHING_ISR(should_context_switch);
}

// Serial commands
///////////////////////////////////////////////////////////
void command_put_raw_button_event(const char* button_index, const char* is_button_down_event) {
  PebbleEvent e;
  int is_down = atoi(is_button_down_event);
  int button = atoi(button_index);

  if ((button < 0 || button > NUM_BUTTONS) || (is_down != 1 && is_down != 0)) {
    return;
  }
  e.type = (is_down) ? PEBBLE_BUTTON_DOWN_EVENT : PEBBLE_BUTTON_UP_EVENT;
  e.button.button_id = button;
  event_put(&e);
}
