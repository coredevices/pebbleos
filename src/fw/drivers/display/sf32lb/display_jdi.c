/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include <pbl/drivers/display/sf32lb/display_jdi.h>

#include "board/board.h"
#include <pbl/drivers/display/display.h>
#include <pbl/drivers/gpio.h>
#include "kernel/events.h"
#include "kernel/util/delay.h"
#include "pbl/soc/sf32lb/sleep.h"
#include <pbl/drivers/rtc.h>
#include "pbl/mcu/cache.h"
#include "pbl/services/new_timer/new_timer.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

#include "FreeRTOS.h"
#include "pbl/kernel/sem.h"

#include "bf0_hal_lcdc.h"
#include "bf0_hal_lptim.h"
#include "bf0_hal_rtc.h"

PBL_LOG_MODULE_DEFINE(driver_display_jdi, CONFIG_DRIVER_DISPLAY_LOG_LEVEL);

#define POWER_SEQ_DELAY_TIME_US  11000
#define POWER_RESET_CYCLE_DELAY_TIME_US 500000

// Timeout for detecting the SiFli HAL silent-loss bug: the LCDC kicks off a
// transfer but never delivers the EOF callback, so the compositor would block
// forever. A normal full-frame transfer takes well under 20ms; 500ms is a
// conservative threshold that won't fire on transient stalls but catches a
// real wedge well before the user perceives the freeze.
#define DISPLAY_SILENT_LOSS_TIMEOUT_MS 500

// Bytes of the LCDC peripheral register file snapshotted into BSS before
// crashing. The PebbleOS coredump captures all of HPSYS SRAM but does not
// capture MMIO; copying registers into SRAM gets them into the coredump that
// gets shared with Sifli. 2KB covers IRQ/SETTING/COMMAND/CANVAS/LAYER0/JDI_PAR
// (lcd_if.h struct ends around offset 0x100).
#define DISPLAY_LCDC_REG_DUMP_BYTES 2048

// Pointer to the compositor's framebuffer - we convert in-place to save 44KB RAM
static uint8_t *s_framebuffer;
static uint16_t s_update_y0;
static uint16_t s_update_y1;
static bool s_initialized;
static bool s_updating;
static UpdateCompleteCallback s_uccb;
static PBL_SEM_DEFINE(s_sem, 0, 1);
static TimerID s_silent_loss_timer = TIMER_INVALID_ID;
// Set from HAL_LCDC_SendLayerDataCpltCbk (ISR). The silent-loss handler checks
// this to distinguish "EOF truly never fired" (real bug → crash) from "EOF
// fired but terminate hasn't drained off the KernelMain queue yet" (race
// against scheduler latency → don't crash).
static volatile bool s_eof_observed;
// Captured at silent-loss time so the coredump shipped to Sifli carries the
// LCDC register state at the moment of the wedge. uint32_t storage (not
// uint8_t) so the linker word-aligns it — prv_snapshot_lcdc_regs reads MMIO
// as 32-bit volatiles and would fault on a misaligned destination. Marked
// volatile because the only consumer is the postmortem coredump tool, not
// any C code in this image — without it, the compiler eliminates the stores.
static volatile uint32_t s_lcdc_pre_crash_regs[DISPLAY_LCDC_REG_DUMP_BYTES / sizeof(uint32_t)];

// Ring buffer of recent LCDC interrupts, recorded by display_jdi_irq_handler
#define DISPLAY_IRQ_LOG_ENTRIES 32

typedef struct {
  uint32_t timestamp;     // rtc_get_ticks() low 32 bits
  uint32_t irq_before;    // LCD_IF.IRQ as the HAL handler will read it
  uint32_t irq_after;     // LCD_IF.IRQ after the HAL handler cleared bits
  uint32_t jdi_par_ctrl;  // LCD_IF.JDI_PAR_CTRL before the handler (INT_LINE_NUM)
  uint32_t status;        // LCD_IF.STATUS before the handler
  uint32_t flags;         // bit0: EOF callback fired; bit1: s_updating at entry
} DisplayIrqLogEntry;

typedef struct {
  uint32_t write_count;   // total IRQs logged; newest = (write_count-1) % N
  DisplayIrqLogEntry entries[DISPLAY_IRQ_LOG_ENTRIES];
} DisplayIrqLog;

static volatile DisplayIrqLog s_lcdc_irq_log;

// Set by HAL_LCDC_SendLayerDataCpltCbk (the EOF callback) so the IRQ logger can
// record, per interrupt, whether the HAL reached the completion path.
static volatile bool s_lcdc_eof_cb_fired;

#ifndef CONFIG_RELEASE
// Test hook: arm a one-shot drop of the next LCDC transfer-complete callback,
// simulating the silent-loss failure mode. The silent-loss timer should fire
// ~DISPLAY_SILENT_LOSS_TIMEOUT_MS later and PBL_CROAK.
static volatile bool s_test_drop_next_complete;
#endif

#if DISPLAY_ORIENTATION_ROTATED_180
static bool s_rotated_180 = true;
#else
static bool s_rotated_180 = false;
#endif

static void prv_power_cycle(void){
  OutputConfig cfg = {
    .gpio = hwp_gpio1,
    .active_high = true,
  };

  // This will disable all JDI pull-ups/downs so that VLCD can fully turn off,
  // allowing for a clean power cycle.

  cfg.gpio_pin = DISPLAY->pinmux.b1.pad - PAD_PA00;
  gpio_output_init(&cfg, GPIO_OType_PP);
  gpio_output_set(&cfg, false);

  cfg.gpio_pin = DISPLAY->pinmux.vck.pad - PAD_PA00;
  gpio_output_init(&cfg, GPIO_OType_PP);
  gpio_output_set(&cfg, false);

  cfg.gpio_pin = DISPLAY->pinmux.xrst.pad - PAD_PA00;
  gpio_output_init(&cfg, GPIO_OType_PP);
  gpio_output_set(&cfg, false);

  cfg.gpio_pin = DISPLAY->pinmux.hck.pad - PAD_PA00;
  gpio_output_init(&cfg, GPIO_OType_PP);
  gpio_output_set(&cfg, false);

  cfg.gpio_pin = DISPLAY->pinmux.r2.pad - PAD_PA00;
  gpio_output_init(&cfg, GPIO_OType_PP);
  gpio_output_set(&cfg, false);

  gpio_output_set(&DISPLAY->vddp, false);
  gpio_output_set(&DISPLAY->vlcd, false);

  delay_us(POWER_RESET_CYCLE_DELAY_TIME_US);
}

static void prv_display_on() {
  gpio_output_set(&DISPLAY->vlcd, true);
  delay_us(POWER_SEQ_DELAY_TIME_US);
  gpio_output_set(&DISPLAY->vddp, true);
  delay_us(POWER_SEQ_DELAY_TIME_US);

  LPTIM_TypeDef *lptim = DISPLAY->vcom.lptim;

  lptim->CFGR |= LPTIM_INTCLOCKSOURCE_LPCLOCK;
  lptim->ARR = RC10K_FREQ / DISPLAY->vcom.freq_hz;
  lptim->CMP = lptim->ARR / 2;
  lptim->CR |= LPTIM_CR_ENABLE;
  lptim->CR |= LPTIM_CR_CNTSTRT;
}

static void prv_display_off() {
  DisplayJDIState *state = DISPLAY->state;
  HAL_LCDC_DeInit(&state->hlcdc);

  LPTIM_TypeDef *lptim = DISPLAY->vcom.lptim;

  lptim->CR &= ~LPTIM_CR_ENABLE;
  lptim->CR &= ~LPTIM_CR_CNTSTRT;

  delay_us(POWER_SEQ_DELAY_TIME_US);
  gpio_output_set(&DISPLAY->vddp, false);
  delay_us(POWER_SEQ_DELAY_TIME_US);
  gpio_output_set(&DISPLAY->vlcd, false);
}

static HAL_StatusTypeDef prv_display_update_start(void) {
  DisplayJDIState *state = DISPLAY->state;

  // The LCDC reads the framebuffer over DMA, which bypasses the D-cache.
  // Flush dirty lines for the rows we're about to send so the LCDC sees
  // the 332-converted pixels instead of stale SRAM.
  uintptr_t fb_addr = (uintptr_t)&s_framebuffer[s_update_y0 * PBL_DISPLAY_WIDTH];
  size_t fb_size = (size_t)(s_update_y1 - s_update_y0 + 1) * PBL_DISPLAY_WIDTH;
  dcache_align(&fb_addr, &fb_size);
  dcache_flush((const void *)fb_addr, fb_size);

  // Only send the dirty region that was converted to RGB332 format
  HAL_LCDC_SetROIArea(&state->hlcdc, 0, s_update_y0, PBL_DISPLAY_WIDTH - 1, s_update_y1);
  HAL_LCDC_LayerSetData(&state->hlcdc, HAL_LCDC_LAYER_DEFAULT, s_framebuffer, 0, s_update_y0,
                        PBL_DISPLAY_WIDTH - 1, s_update_y1);
  return HAL_LCDC_SendLayerData_IT(&state->hlcdc);
}

static void prv_handle_send_failure(const char *ctx, HAL_StatusTypeDef status) {
  DisplayJDIState *state = DISPLAY->state;
  PBL_LOG_ERR("display: %s SendLayerData_IT=%d State=%d ErrorCode=0x%08lx",
              ctx, (int)status, (int)state->hlcdc.State,
              (unsigned long)state->hlcdc.ErrorCode);
  state->hlcdc.State = HAL_LCDC_STATE_READY;
  state->hlcdc.ErrorCode = HAL_LCDC_ERROR_NONE;
}

static void prv_snapshot_lcdc_regs(const LCDC_HandleTypeDef *hlcdc) {
  // Peripheral MMIO must be read as aligned 32-bit volatile loads; plain
  // memcpy may emit byte-wise loads on some toolchains and fault.
  const volatile uint32_t *src = (const volatile uint32_t *)hlcdc->Instance;
  for (size_t i = 0; i < DISPLAY_LCDC_REG_DUMP_BYTES / sizeof(uint32_t); i++) {
    s_lcdc_pre_crash_regs[i] = src[i];
  }
}

// Runs on PebbleTask_NewTimers if no LCDC EOF callback arrives within
// DISPLAY_SILENT_LOSS_TIMEOUT_MS of kickoff. Known trigger: SiFli HAL's
// ICB-overflow path (bf0_hal_lcdc.c HAL_LCDC_IRQHandler) sets
// HAL_LCDC_ERROR_OVERFLOW without invoking XferCpltCallback / XferErrorCallback,
// so the firmware silently loses the completion and the compositor wedges.
// Capture LCDC registers into BSS so they ride in the coredump, then crash —
// the user sees a reboot instead of a frozen screen, and the coredump
// captures the state Sifli needs to diagnose the underlying HAL bug.
static void prv_silent_loss_handler(void *data) {
  if (s_eof_observed) {
    // EOF fired between us arming the timer and the timeout — terminate is
    // queued on KernelMain but hasn't run yet. Don't crash on a scheduler
    // latency artifact.
    return;
  }
  DisplayJDIState *state = DISPLAY->state;
  prv_snapshot_lcdc_regs(&state->hlcdc);
  // HPSYS SRAM is Normal cacheable (since the MPU region shrink in
  // hal_sifli/sf32lb52 system_bf0_ap.c), so the snapshot stores above land
  // in D-cache. PebbleOS captures coredump RAM after the crash path may
  // have already invalidated the cache, so flush the dirty lines back to
  // physical SRAM before we trip PBL_CROAK.
  uintptr_t snap_addr = (uintptr_t)s_lcdc_pre_crash_regs;
  size_t snap_size = sizeof(s_lcdc_pre_crash_regs);
  dcache_align(&snap_addr, &snap_size);
  dcache_flush((const void *)snap_addr, snap_size);
  PBL_CROAK("LCDC silent loss: no EOF in %ums (State=%d Err=0x%lx y=%u..%u)",
            (unsigned)DISPLAY_SILENT_LOSS_TIMEOUT_MS,
            (int)state->hlcdc.State,
            (unsigned long)state->hlcdc.ErrorCode,
            (unsigned)s_update_y0, (unsigned)s_update_y1);
}

static void prv_display_update_terminate(void *data) {
  new_timer_stop(s_silent_loss_timer);

  // Convert the updated region back from 332 to 222 format
  for (uint16_t y = s_update_y0; y <= s_update_y1; y++) {
    uint8_t *row = &s_framebuffer[y * PBL_DISPLAY_WIDTH];

  if (s_rotated_180) {
    // Undo HMirror before converting back
    for (uint16_t x = 0; x < PBL_DISPLAY_WIDTH / 2; x++) {
      uint8_t tmp = row[x];
      row[x] = row[PBL_DISPLAY_WIDTH - 1 - x];
      row[PBL_DISPLAY_WIDTH - 1 - x] = tmp;
    }
  }

    // Convert this row in-place from 332 to 222 using word-level bit manipulation
    // 332 format: RR 0G GG BB (bits 7-6 R, 4-3 G, 1-0 B)
    // 222 format: XX RR GG BB (bits 7-6 unused, 5-4 R, 3-2 G, 1-0 B)
    uint32_t *row32 = (uint32_t *)row;
    for (uint16_t x = 0; x < PBL_DISPLAY_WIDTH / 4; x++) {
      uint32_t p = row32[x];
      row32[x] = ((p >> 2) & 0x30303030) |  // R: bits 6-7 → 4-5
                 ((p >> 1) & 0x0C0C0C0C) |  // G: bits 3-4 → 2-3
                 (p & 0x03030303);          // B: bits 0-1 stay
    }
  }

  s_updating = false;
  s_uccb();
  soc_sf32lb_sleep_release(SOC_SF32LB_DEEPWFI);
}

void display_jdi_irq_handler(DisplayJDIDevice *disp) {
  DisplayJDIState *state = DISPLAY->state;
  LCD_IF_TypeDef *regs = state->hlcdc.Instance;

  // Snapshot LCDC state before/after the HAL handler into the ring buffer so a
  // coredump shows the interrupt interleaving behind the lost-EOF wedge.
  volatile DisplayIrqLogEntry *entry =
      &s_lcdc_irq_log.entries[s_lcdc_irq_log.write_count % DISPLAY_IRQ_LOG_ENTRIES];
  entry->timestamp = (uint32_t)rtc_get_ticks();
  entry->irq_before = regs->IRQ;
  entry->jdi_par_ctrl = regs->JDI_PAR_CTRL;
  entry->status = regs->STATUS;
  entry->flags = s_updating ? 0x2u : 0x0u;

  s_lcdc_eof_cb_fired = false;
  HAL_LCDC_IRQHandler(&state->hlcdc);

  entry->irq_after = regs->IRQ;
  if (s_lcdc_eof_cb_fired) {
    entry->flags |= 0x1u;
  }
  s_lcdc_irq_log.write_count++;
}

void HAL_LCDC_SendLayerDataCpltCbk(LCDC_HandleTypeDef *lcdc) {
  portBASE_TYPE woken = pdFALSE;

  // Tell the IRQ logger the HAL reached the completion path for this interrupt.
  s_lcdc_eof_cb_fired = true;

#ifndef CONFIG_RELEASE
  if (s_test_drop_next_complete && s_updating) {
    s_test_drop_next_complete = false;
    // Simulate the lost-completion failure mode: leave s_eof_observed false
    // and don't post the terminate event. The silent-loss timer should fire
    // ~DISPLAY_SILENT_LOSS_TIMEOUT_MS later and PBL_CROAK.
    portEND_SWITCHING_ISR(woken);
    return;
  }
#endif

  // Mark EOF before doing anything else so the silent-loss handler's race
  // check sees it even if the terminate event sits on the KernelMain queue
  // longer than the timeout.
  s_eof_observed = true;

  if (s_updating) {
    PebbleEvent e = {
        .type = PEBBLE_CALLBACK_EVENT,
        .callback =
            {
                .callback = prv_display_update_terminate,
            },
    };

    woken = event_put_isr(&e) ? pdTRUE : pdFALSE;
  } else {
    pbl_sem_give(&s_sem);
  }

  portEND_SWITCHING_ISR(woken);
}

void display_init(void) {
  if (s_initialized) {
    return;
  }

  DisplayJDIState *state = DISPLAY->state;

  gpio_output_init(&DISPLAY->vddp, GPIO_OType_PP);
  gpio_output_init(&DISPLAY->vlcd, GPIO_OType_PP);

  prv_power_cycle();

  HAL_PIN_Set(DISPLAY->pinmux.xrst.pad, DISPLAY->pinmux.xrst.func, DISPLAY->pinmux.xrst.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.vst.pad, DISPLAY->pinmux.vst.func, DISPLAY->pinmux.vst.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.vck.pad, DISPLAY->pinmux.vck.func, DISPLAY->pinmux.vck.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.enb.pad, DISPLAY->pinmux.enb.func, DISPLAY->pinmux.enb.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.hst.pad, DISPLAY->pinmux.hst.func, DISPLAY->pinmux.hst.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.hck.pad, DISPLAY->pinmux.hck.func, DISPLAY->pinmux.hck.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.r1.pad, DISPLAY->pinmux.r1.func, DISPLAY->pinmux.r1.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.r2.pad, DISPLAY->pinmux.r2.func, DISPLAY->pinmux.r2.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.g1.pad, DISPLAY->pinmux.g1.func, DISPLAY->pinmux.g1.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.g2.pad, DISPLAY->pinmux.g2.func, DISPLAY->pinmux.g2.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.b1.pad, DISPLAY->pinmux.b1.func, DISPLAY->pinmux.b1.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.b2.pad, DISPLAY->pinmux.b2.func, DISPLAY->pinmux.b2.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.vcom_frp.pad, DISPLAY->pinmux.vcom_frp.func, DISPLAY->pinmux.vcom_frp.flags, 1);
  HAL_PIN_Set(DISPLAY->pinmux.xfrp.pad, DISPLAY->pinmux.xfrp.func, DISPLAY->pinmux.xfrp.flags, 1);

  HAL_LCDC_Init(&state->hlcdc);
  HAL_LCDC_LayerReset(&state->hlcdc, HAL_LCDC_LAYER_DEFAULT);
  HAL_LCDC_LayerSetCmpr(&state->hlcdc, HAL_LCDC_LAYER_DEFAULT, 0);
  HAL_LCDC_LayerSetFormat(&state->hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB332);
  HAL_LCDC_LayerVMirror(&state->hlcdc, HAL_LCDC_LAYER_DEFAULT, s_rotated_180);

  HAL_NVIC_SetPriority(DISPLAY->irqn, DISPLAY->irq_priority, 0);
  HAL_NVIC_EnableIRQ(DISPLAY->irqn);


  prv_display_on();

  s_initialized = true;
}

void display_set_enabled(bool enabled) {
  if (enabled) {
    prv_display_on();
  } else {
    prv_display_off();
  }
}

bool display_update_in_progress(void) {
  return s_updating;
}

void display_set_rotated(bool rotated) {
  DisplayJDIState *state = DISPLAY->state;

#if DISPLAY_ORIENTATION_ROTATED_180
  s_rotated_180 = !rotated;
#else
  s_rotated_180 = rotated;
#endif
  HAL_LCDC_LayerVMirror(&state->hlcdc, HAL_LCDC_LAYER_DEFAULT, s_rotated_180);

}

void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {
  DisplayRow row;
  bool first_row = true;

  PBL_ASSERTN(!s_updating);

  // Convert rows in-place from 222 to 332 format
  // We use the compositor's framebuffer directly to save RAM
  while (nrcb(&row)) {
    if (first_row) {
      // Capture pointer to compositor's framebuffer from first row
      s_framebuffer = row.data;
      s_update_y0 = row.address;
      first_row = false;
    }
    s_update_y1 = row.address;

    // Convert this row in-place from 222 to 332 using word-level bit manipulation
    // 222 format: XX RR GG BB (bits 7-6 unused, 5-4 R, 3-2 G, 1-0 B)
    // 332 format: RR 0G GG BB (bits 7-6 R, 4-3 G, 1-0 B)
    uint32_t *row32 = (uint32_t *)row.data;
    for (uint16_t x = 0; x < PBL_DISPLAY_WIDTH / 4; x++) {
      uint32_t p = row32[x];
      row32[x] = ((p & 0x30303030) << 2) |  // R: bits 4-5 → 6-7
                 ((p & 0x0C0C0C0C) << 1) |  // G: bits 2-3 → 3-4
                 (p & 0x03030303);          // B: bits 0-1 stay
    }

    if (s_rotated_180) {
      // HMirror in software (VMirror is done by hardware)
      uint8_t *row_data = row.data;
      for (uint16_t x = 0; x < PBL_DISPLAY_WIDTH / 2; x++) {
        uint8_t tmp = row_data[x];
        row_data[x] = row_data[PBL_DISPLAY_WIDTH - 1 - x];
        row_data[PBL_DISPLAY_WIDTH - 1 - x] = tmp;
      }
    }
  }

  if (first_row) {
    // No rows to update
    uccb();
    return;
  }

  // Adjust framebuffer pointer to start of buffer (row 0)
  s_framebuffer = s_framebuffer - (s_update_y0 * PBL_DISPLAY_WIDTH);

  // Lazy-create the silent-loss timer. display_init runs via boot_splash_start
  // before new_timer_service_init, so creating it in display_init would
  // silently fail. The compositor doesn't reach this path until well after
  // the timer service is up.
  if (s_silent_loss_timer == TIMER_INVALID_ID) {
    s_silent_loss_timer = new_timer_create();
  }

  s_uccb = uccb;
  s_updating = true;
  s_eof_observed = false;

  soc_sf32lb_sleep_block(SOC_SF32LB_DEEPWFI);
  // Arm the timer before kickoff so the EOF IRQ, which can fire as soon as
  // SendLayerData_IT returns, never races us into a state where the timer
  // isn't yet armed. prv_display_update_terminate stops it on the normal
  // completion path; the kickoff-failure path below stops it via the same
  // terminate call.
  new_timer_start(s_silent_loss_timer, DISPLAY_SILENT_LOSS_TIMEOUT_MS,
                  prv_silent_loss_handler, NULL, 0);
  HAL_StatusTypeDef status = prv_display_update_start();
  if (status != HAL_OK) {
    prv_handle_send_failure("update", status);
    prv_display_update_terminate(NULL);
  }
}

void display_update_boot_frame(uint8_t *framebuffer) {
  if (s_rotated_180) {
    // HMirror in software (VMirror is done by hardware)
    for (uint16_t y = 0; y < PBL_DISPLAY_HEIGHT; y++) {
      uint8_t *row = &framebuffer[y * PBL_DISPLAY_WIDTH];
      for (uint16_t x = 0; x < PBL_DISPLAY_WIDTH / 2; x++) {
        uint8_t tmp = row[x];
        row[x] = row[PBL_DISPLAY_WIDTH - 1 - x];
        row[PBL_DISPLAY_WIDTH - 1 - x] = tmp;
      }
    }
  }

  s_framebuffer = framebuffer;
  s_update_y0 = 0;
  s_update_y1 = PBL_DISPLAY_HEIGHT - 1;

  soc_sf32lb_sleep_block(SOC_SF32LB_DEEPWFI);
  HAL_StatusTypeDef status = prv_display_update_start();
  if (status == HAL_OK) {
    pbl_sem_take(&s_sem, PBL_FOREVER);
  } else {
    // Without this guard a failed kickoff would block boot forever on s_sem,
    // since the EOF IRQ that gives the semaphore never fires.
    prv_handle_send_failure("boot", status);
  }
  soc_sf32lb_sleep_release(SOC_SF32LB_DEEPWFI);
}

void display_clear(void) {}

#ifndef CONFIG_RELEASE
void display_jdi_test_drop_next_complete(void) {
  s_test_drop_next_complete = true;
}
#endif