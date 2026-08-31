#include <pbl/drivers/button.h>

#include "board/board.h"
#include "console/prompt.h"
#include "kernel/events.h"
#include "system/passert.h"

// watch rotation
static bool s_rotated_180 = false;

void button_set_rotated(bool rotated) {
  s_rotated_180 = rotated;
}

bool button_is_pressed(ButtonId id) {
  if (s_rotated_180 && id == BUTTON_ID_UP) {
    id = BUTTON_ID_DOWN;
  } else if (s_rotated_180 && id == BUTTON_ID_DOWN) {
    id = BUTTON_ID_UP;
  }

  const ButtonConfig *button_config = &BOARD_CONFIG_BUTTON.buttons[id];
  
  uint32_t bit = nrf_gpio_pin_read(button_config->gpiote.gpio_pin);
  return (BOARD_CONFIG_BUTTON.active_high) ? bit : !bit;
}

uint8_t button_get_state_bits(void) {
  uint8_t button_state = 0x00;
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    button_state |= (button_is_pressed(i) ? 0x01 : 0x00) << i;
  }
  return button_state;
}

void button_init(void) {
  if (BOARD_CONFIG_BUTTON.button_com.gpio_pin)
    WTF; // NYI

  for (int i = 0; i < NUM_BUTTONS; ++i) {
    nrf_gpio_cfg_input(BOARD_CONFIG_BUTTON.buttons[i].gpiote.gpio_pin, BOARD_CONFIG_BUTTON.buttons[i].pull);
  }
}

void command_button_read(const char* button_id_str) {
  int button = atoi(button_id_str);

  if (button < 0 || button >= NUM_BUTTONS) {
    prompt_send_response("Invalid button");
    return;
  }

  if (button_is_pressed(button)) {
    prompt_send_response("down");
  } else {
    prompt_send_response("up");
  }
}
