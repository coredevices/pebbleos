#include "drivers/display/display.h"

uint32_t display_baud_rate_change(uint32_t new_frequency_hz) {
  return new_frequency_hz;
}

void display_init(void) {
}

void display_clear(void) {
}

void display_set_enabled(bool enabled) {
}

bool display_update_in_progress(void) {
  return true;
}

void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {
}

void display_pulse_vcom(void) {
}

void display_show_splash_screen(void) {
  // The bootloader has already drawn the splash screen for us; nothing to do!
}

void display_show_panic_screen(uint32_t error_code)
{
}

// Stubs for display offset
void display_set_offset(GPoint offset) {}

GPoint display_get_offset(void) { return GPointZero; }
