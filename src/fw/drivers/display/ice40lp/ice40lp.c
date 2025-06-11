/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "drivers/display/ice40lp/ice40lp.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "applib/graphics/framebuffer.h"
#include "applib/graphics/gtypes.h"
#include "board/board.h"
#include "board/display.h"
#include "debug/power_tracking.h"
#include "drivers/clocksource.h"
#include "drivers/display/ice40lp/fpga_bitstream.auto.h"
#include "drivers/display/ice40lp/ice40lp_definitions.h"
#include "drivers/display/ice40lp/ice40lp_internal.h"
#include "drivers/display/ice40lp/snowy_boot.h"
#include "drivers/dma.h"
#include "drivers/exti.h"
#include "drivers/gpio.h"
#include "drivers/periph_config.h"
#include "drivers/pmic.h"
#include "drivers/spi.h"
#include "drivers/spi_dma.h"
#include "kernel/events.h"
#include "kernel/event_loop.h"
#include "kernel/util/sleep.h"
#include "kernel/util/stop.h"
#include "os/mutex.h"
#include "os/tick.h"
#include "semphr.h"
#include "services/common/analytics/analytics.h"
#include "services/common/compositor/compositor.h"
#include "services/common/compositor/compositor_display.h"
#include "services/common/system_task.h"
#include "system/logging.h"
#include "system/passert.h"
#include "system/profiler.h"
#include "task.h"
#include "util/attributes.h"
#include "util/net.h"
#include "util/reverse.h"
#include "util/size.h"

#define STM32F4_COMPATIBLE
#define STM32F7_COMPATIBLE
#include <mcu.h>

#if DISPLAY_ORIENTATION_ROW_MAJOR || DISPLAY_ORIENTATION_ROW_MAJOR_INVERTED
#define DISP_LINES DISP_ROWS
#define DISP_PIXELS DISP_COLS
#elif DISPLAY_ORIENTATION_COLUMN_MAJOR_INVERTED
#define DISP_LINES DISP_COLS
#define DISP_PIXELS DISP_ROWS
#else
#error "Unknown or missing display orientation define"
#endif

typedef void (*PopulateLineCB)(
    int column, uint8_t * restrict line_buffer, const void *cb_data);

//! 2 buffers to hold line data being transferred.
static uint8_t DMA_READ_BSS s_line_buffer[2][DISP_PIXELS];
//! buffer index keeps track of which line buffer is in use
static uint32_t s_buffer_idx;
//! line index is the line of the display currently being updated
static uint32_t s_line_index;

//! offset for shifting the image origin from the display's origin
//! display coordinates (0,0) are top-left,
//! offset positive values shift right and down
static GPoint s_disp_offset;

static bool s_update_in_progress;
static bool s_terminate_pending;

static RtcTicks s_start_ticks;

static bool s_initialized = false;

//! lockout to prevent display updates when the panic screen is shown
static bool s_panic_screen_lockout;

static PebbleMutex *s_display_update_mutex;
static SemaphoreHandle_t s_fpga_busy;

static UpdateCompleteCallback s_update_complete_callback;

static void prv_start_dma_transfer(uint8_t *addr, uint32_t length);
static void display_interrupt_intn(bool *should_context_switch);

static inline OPTIMIZE_FUNC(2) void prv_pixel_scramble(
    uint8_t * restrict line_buf, const uint8_t px_odd, const uint8_t px_even,
    const int offset) {
  uint8_t msb, lsb;
  msb = (px_odd & 0b00101010) | (px_even & 0b00101010) >> 1;
  lsb = (px_odd & 0b00010101) << 1 | (px_even & 0b00010101);
  line_buf[offset/2] = msb;
  line_buf[offset/2 + DISP_PIXELS/2] = lsb;
}

static inline OPTIMIZE_FUNC(2) void prv_row_major_get_line(
    uint8_t * restrict line, const uint8_t * restrict image_buf,
    int index) {
#if DISPLAY_ORIENTATION_ROW_MAJOR_INVERTED
  // Optimized line renderer for Robert.
  // Could easily apply to the other screens, but only Robert really needs it.
  // By loading both pixels with a single load, we can cut down code size (cache benefit)
  // and decrease number of bus accesses.
  // Theoretically loading 4 pixels at a time should be better, but GCC generated much
  // worse code that way.

#if DISPLAY_ORIENTATION_ROTATED_180
  // Setup srcbuf to be the end, since we need to scan backwards horizontally
  const uint16_t *srcbuf = (const uint16_t *)(image_buf + DISP_PIXELS * index);
  for (int dst_offset = 0; dst_offset < DISP_PIXELS/2; dst_offset++) {
    // Get the two pixels
    const uint16_t pix = *srcbuf++;

    // Swizzle the pixels
    line[0]             = ((pix)      & 0b101010) | ((pix >> 9) & 0b010101);
    line[DISP_PIXELS/2] = ((pix << 1) & 0b101010) | ((pix >> 8) & 0b010101);
    line++;
  }
#else
  // Setup srcbuf to be the end, since we need to scan backwards horizontally
  const uint16_t *srcbuf = (const uint16_t *)(image_buf + DISP_PIXELS * (DISP_LINES - index) - 2);
  for (int dst_offset = 0; dst_offset < DISP_PIXELS/2; dst_offset++) {
    // Get the two pixels
    const uint16_t pix = *srcbuf--;

    // Swizzle the pixels
    line[0]             = ((pix >> 8) & 0b101010) | ((pix >> 1) & 0b010101);
    line[DISP_PIXELS/2] = ((pix >> 7) & 0b101010) | ((pix)      & 0b010101);
    line++;
  }
#endif // DISPLAY_ORIENTATION_ROTATED_180
#else
  // adjust line index according to display offset,
  // populate blank (black) line if this exceeds the source framebuffer
  index -= s_disp_offset.y;
  if (!WITHIN(index, 0, DISP_LINES - 1)) {
    memset(line, 0, DISP_PIXELS);
    return;
  }

  uint8_t odd, even;
#if PLATFORM_SPALDING
  const GBitmapDataRowInfoInternal *row_infos = g_gbitmap_spalding_data_row_infos;
  const uint8_t *row_start = image_buf + row_infos[index].offset;
#endif

  // Line starts with MSB of each color in all pixels
  // Line finishes with LSB of each color in all pixels
  // separate src_offset is adjusted according to manufacturing offset,
  // loop condition/continue makes sure we don't read past boundaries of src framebuffer
  for (int src_offset = -s_disp_offset.x, dst_offset = 0;
       src_offset < DISP_PIXELS && dst_offset < DISP_PIXELS;
       src_offset += 2, dst_offset += 2) {
#if !PLATFORM_SPALDING
    if (src_offset < 0) {
      continue;
    }
#endif

#if DISPLAY_ORIENTATION_ROW_MAJOR
  #if PLATFORM_SPALDING
    even = WITHIN(src_offset + 1, row_infos[index].min_x, row_infos[index].max_x) ?
           row_start[src_offset + 1] : 0;
    odd  = WITHIN(src_offset, row_infos[index].min_x, row_infos[index].max_x) ?
           row_start[src_offset] : 0;
  #else
    #error Unsupported display
  #endif
#elif DISPLAY_ORIENTATION_COLUMN_MAJOR_INVERTED
    even = image_buf[DISP_COLS * (DISP_ROWS-2 - src_offset)     + index];
    odd  = image_buf[DISP_COLS * (DISP_ROWS-2 - src_offset + 1) + index];
#endif
    prv_pixel_scramble(line, odd, even, dst_offset);
  }
#endif
}

static void prv_framebuffer_populate_line(
    int index, uint8_t * restrict line) {
  const uint8_t *frame_buffer = compositor_get_framebuffer()->buffer;
  prv_row_major_get_line(line, frame_buffer, index);
}

static void enable_display_dma_clock(void) {
  power_tracking_start(PowerSystemMcuDma2);
}

static void disable_display_dma(void) {
  // Properly disable DMA interrupts and deinitialize the DMA controller to prevent pending
  // interrupts from firing when the clock is re-enabled (this could possibly cause a stray
  // terminate callback being added to kernel main)

  spi_ll_slave_write_dma_stop(ICE40LP->spi_port);
  power_tracking_stop(PowerSystemMcuDma2);
}

static void prv_terminate_transfer(void *data) {
  if (s_panic_screen_lockout) {
    return;
  }

  // Only need intn when communicating with the display.
  // Disable EXTI interrupt before ending the frame to prevent possible race condition resulting
  // from a almost empty FIFO on the FPGA triggering a terminate call before the interrupt is
  // disabled
  exti_disable(ICE40LP->busy_exti);

  disable_display_dma();
  display_spi_end_transaction();

  analytics_stopwatch_stop(ANALYTICS_APP_METRIC_DISPLAY_WRITE_TIME);

  s_update_in_progress = false;
  s_terminate_pending = false;

  mutex_unlock(s_display_update_mutex);

  // Store temporary variable, then NULL, to protect against the case where the compositor calls
  // into the display driver from the callback, then we NULL out the update complete callback
  // afterwards.
  UpdateCompleteCallback update_complete_cb = s_update_complete_callback;
  s_update_complete_callback = NULL;
  if (update_complete_cb) {
    update_complete_cb();
  }
}

static uint32_t prv_get_next_buffer_idx(uint32_t idx) {
  return (idx + 1) % ARRAY_LENGTH(s_line_buffer);
}

//! Wait for the FPGA to finish updating the display.
//! @returns true if the FPGA is busy on exit
static bool prv_wait_busy(void) {
  // Make sure that semaphore token count is zero before we wait on it and before we check the state
  // of the FPGA busy line to prevent the semaphore take/give from getting out of sync (not exactly
  // sure what race condition causes the out of sync bug, but it seems to happen after a while).
  // See https://pebbletechnology.atlassian.net/browse/PBL-21904
  xSemaphoreTake(s_fpga_busy, 0);

  if (!display_busy()) {
    return false;
  }

  // A full frame should take no longer than 33 msec to draw. If we are waiting
  // longer than that, something is very wrong.
  TickType_t max_wait_time_ticks = milliseconds_to_ticks(40);
  bool busy_on_exit = false;
  if (xSemaphoreTake(s_fpga_busy, max_wait_time_ticks) != pdTRUE) {
    PBL_LOG(LOG_LEVEL_ERROR, "Display not coming out of a busy state.");
    // Nothing needs to be done to recover the FPGA from a bad state. The
    // falling edge of SCS (to start a new frame) resets the FPGA logic,
    // clearing the error state.
    busy_on_exit = true;
  }
  return busy_on_exit;
}

static void prv_reprogram_display(void) {
  // CDONE is expected to go low during reprogramming. Don't pollute the logs
  // with "CDONE has gone low" messages.
  analytics_inc(ANALYTICS_DEVICE_METRIC_FPGA_REPROGRAM_COUNT,
                AnalyticsClient_System);
  exti_disable(ICE40LP->cdone_exti);
  display_program(s_fpga_bitstream, sizeof(s_fpga_bitstream));
  exti_enable(ICE40LP->cdone_exti);
}

static void prv_cdone_low_handler(void *context) {
  PBL_LOG(LOG_LEVEL_ERROR,
          "CDONE has gone low. The FPGA has lost its configuration.");

  if (!mutex_lock_with_timeout(s_display_update_mutex, 200)) {
    PBL_LOG(LOG_LEVEL_DEBUG,
            "Couldn't lock out display driver to reprogram FPGA.");
    return;
  }
  prv_reprogram_display();
  PBL_ASSERTN(!display_busy());
  mutex_unlock(s_display_update_mutex);
}

static void prv_cdone_low_isr(bool *should_context_switch) {
  system_task_add_callback_from_isr(prv_cdone_low_handler, NULL,
                                    should_context_switch);
}

void display_init(void) {
  if (s_initialized) {
    return;
  }

  clocksource_MCO1_enable(true);

  s_panic_screen_lockout = false;
  s_display_update_mutex = mutex_create();
  s_fpga_busy = xSemaphoreCreateBinary();
  s_update_in_progress = false;
  s_terminate_pending = false;
  s_update_complete_callback = NULL;

  display_start();
  display_program(s_fpga_bitstream, sizeof(s_fpga_bitstream));
  // enable the power rails
  display_power_enable();

  // Set up our INT_N interrupt, aka the "busy line" from the FPGA
  exti_configure_pin(ICE40LP->busy_exti, ExtiTrigger_Falling, display_interrupt_intn);
  // Set up an interrupt to detect the FPGA forgetting its configuration due to
  // e.g. an ESD event.
  exti_configure_pin(ICE40LP->cdone_exti, ExtiTrigger_Falling, prv_cdone_low_isr);
  exti_enable(ICE40LP->cdone_exti);

  s_initialized = true;
}

bool display_update_in_progress(void) {
  // Set this timeout to a relatively large value so that we don't unlock the mutex too early when
  // the DMA controller that is used by the display is being heavily used by another driver (e.g.
  // the bluetooth HCI port) and delays the completion of the update or kernel_main is busy with
  // other tasks (e.g. voice encoding).
  // (see https://pebbletechnology.atlassian.net/browse/PBL-21923)
  static const RtcTicks MAX_BUSY_TICKS = 200;

  bool in_progress = !mutex_lock_with_timeout(s_display_update_mutex, 0);
  if (!in_progress) {
    mutex_unlock(s_display_update_mutex);
  } else if (!s_panic_screen_lockout) {
    if ((rtc_get_ticks() - s_start_ticks) > MAX_BUSY_TICKS) {
      // Ensure that terminate transfer is not enqueued on kernel_main twice when it is busy to
      // prevent terminate transfer from being invoked twice
      // see https://pebbletechnology.atlassian.net/browse/PBL-22084
      // Read and set the termination flag in a critical section to prevent an interrupt pending
      // a transfer if these events occur simultaneously
      bool pend_terminate = false;
      portENTER_CRITICAL();
      if (!s_terminate_pending) {
        s_terminate_pending = true;
        pend_terminate = true;
      }
      portEXIT_CRITICAL();
      if (pend_terminate) {
        PROFILER_NODE_STOP(display_transfer);
        launcher_task_add_callback(prv_terminate_transfer, NULL);
      }
    }
  }
  return in_progress;
}

static void prv_do_display_update(void) {

  if (!mutex_lock_with_timeout(s_display_update_mutex, 0)) {
    PBL_LOG(LOG_LEVEL_DEBUG, "Couldn't start update.");
    return;
  }
  if (s_panic_screen_lockout) {
    mutex_unlock(s_display_update_mutex);
    return;
  }

  analytics_stopwatch_start(ANALYTICS_APP_METRIC_DISPLAY_WRITE_TIME, PebbleTask_App);
  analytics_inc(ANALYTICS_DEVICE_METRIC_DISPLAY_UPDATES_PER_HOUR, AnalyticsClient_System);

  // Communicating with the display, need intn.
  exti_enable(ICE40LP->busy_exti);

  enable_display_dma_clock();

  // Send the first line...
  prv_framebuffer_populate_line(0, s_line_buffer[s_buffer_idx]);

  prv_wait_busy();
  display_spi_begin_transaction();
  display_start_frame();
  if (display_busy()) {
    // If the FPGA was stuck busy before, starting the frame (SCS falling edge)
    // should get it unstuck. If BUSY is still asserted, the FPGA might be
    // unprogrammed or malfunctioning. Either way, reprogramming it should get
    // it back into working order.
    PBL_LOG(LOG_LEVEL_WARNING,
            "Reprogramming FPGA because busy is stuck asserted");
    prv_reprogram_display();
    bool is_busy = display_busy();
#ifdef TARGET_QEMU
    // Bold light-red text on a black background
#define M(s) PBL_LOG(LOG_LEVEL_ALWAYS, "\033[1;91;40m" s "\033[0m")
    if (is_busy) {
      M("################################################");
      M("#             THIS IS A QEMU BUILD             #");
      M("################################################");
      M("#                                              #");
      M("#  The QEMU display driver \033[1;4mdoes not work\033[24m on    #");
      M("#  physical hardware. You must build without   #");
      M("# the --qemu switch when flashing a bigboard.  #");
      M("################################################");
#undef M
      psleep(3000);
    }
#endif
    PBL_ASSERTN(!is_busy);
    // The SPI clock is disabled by prv_reprogram_display.
    display_spi_begin_transaction();
    display_start_frame();
  }
  // set line index after waiting for display to free up
  uint32_t current_idx = s_buffer_idx;
  s_buffer_idx = prv_get_next_buffer_idx(s_buffer_idx);

  // populate the second line and set the next line to be processed as the third line
  prv_framebuffer_populate_line(1, s_line_buffer[s_buffer_idx]);
  s_line_index = 2;

  stop_mode_disable(InhibitorDisplay);

  PROFILER_NODE_START(display_transfer);

  s_update_in_progress = true;
  s_start_ticks = rtc_get_ticks();
  // Start the DMA last to prevent possible race conditions caused by unfortunately timed context
  // switch
  prv_start_dma_transfer(s_line_buffer[current_idx], DISP_PIXELS);
}

//!
//! Starts a redraw of the entire framebuffer to the screen.
//!
//! Currently does NOT:
//!   - make use of nrcb due to rotation requirements; instead accesses the framebuffer directly
//!   - support partial screen updates
//!
void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {
  PBL_ASSERTN(uccb != NULL);

  s_update_complete_callback = uccb;
  prv_do_display_update();
}

static void prv_do_display_update_cb(void *ignored) {
  prv_do_display_update();
}

void display_clear(void) {
  // Set compositor buffer to the powered off color (black) and redraw.
  // Note that compositor owns this framebuffer!
  memset(compositor_get_framebuffer()->buffer, 0x00, FRAMEBUFFER_SIZE_BYTES);

  // The display ISRs pend events on KernelMain and thus implicitly assume
  // that the display update operation began on KernelMain. If we are already
  // running on KernelMain, then just run the display update, otherwise schedule
  // a callback to run on KernelMain that performs the update
  if (pebble_task_get_current() == PebbleTask_KernelMain) {
    prv_do_display_update();
  } else {
    launcher_task_add_callback(prv_do_display_update_cb, NULL);
  }
}

void display_set_enabled(bool enabled) {
  // TODO: Implement this function to enable/disable the display.
}

void display_pulse_vcom(void) { }

//! @return false if there are no more lines to transfer, true if a new line transfer was started
static bool prv_write_next_line(bool *should_context_switch) {
  if (s_line_index == 0 && (!s_terminate_pending)) {
    s_terminate_pending = true;
    PROFILER_NODE_STOP(display_transfer);

    PebbleEvent e = {
      .type = PEBBLE_CALLBACK_EVENT,
      .callback = {
        .callback = prv_terminate_transfer,
      },
    };
    *should_context_switch = event_put_isr(&e);
    return false;
  }

  prv_start_dma_transfer(s_line_buffer[s_buffer_idx], DISP_PIXELS);

  if (s_line_index < DISP_LINES) {
    s_buffer_idx = prv_get_next_buffer_idx(s_buffer_idx);
    prv_framebuffer_populate_line(s_line_index, s_line_buffer[s_buffer_idx]);
    s_line_index++;
  } else {
    // done
    s_line_index = 0;
  }
  return true;
}

//! When the FPGA leaves the busy state while frame data is being sent, this interrupt will
//! signal that the next line can be sent to the display.
static void display_interrupt_intn(bool *should_context_switch) {
  if (s_update_in_progress) {
    if (!spi_ll_slave_dma_in_progress(ICE40LP->spi_port)) {
      // DMA transfer is complete
      if (prv_write_next_line(should_context_switch)) {
        stop_mode_disable(InhibitorDisplay);
      }
    }
  } else {
    // Only release the semaphore after the end of an update
    signed portBASE_TYPE was_higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_fpga_busy, &was_higher_priority_task_woken);
    *should_context_switch = (*should_context_switch) ||
        (was_higher_priority_task_woken != pdFALSE);
  }
}

// DMA
//////////////////

//! This interrupt fires when the transfer of a line has completed.
static bool prv_write_dma_irq_handler(const SPISlavePort *request, void *context) {
  PROFILER_NODE_START(framebuffer_dma);
  bool should_context_switch = false;
  if (display_busy() || !prv_write_next_line(&should_context_switch)) {
    stop_mode_enable(InhibitorDisplay);
  }
  PROFILER_NODE_STOP(framebuffer_dma);
  return should_context_switch;
}

static void prv_start_dma_transfer(uint8_t *addr, uint32_t length) {
  spi_ll_slave_write_dma_start(ICE40LP->spi_port, addr, length, prv_write_dma_irq_handler, NULL);
}

void display_show_panic_screen(uint32_t error_code) {
  // Lock out display driver from performing further updates
  if (!mutex_lock_with_timeout(s_display_update_mutex, 200)) {
    PBL_LOG(LOG_LEVEL_DEBUG, "Couldn't lock out display driver.");
    return;
  }
  s_panic_screen_lockout = true;

  exti_disable(ICE40LP->cdone_exti);
  // Work around an issue which some boards exhibit where there is about a 50%
  // probability that FPGA malfunctions and draw-scene command doesn't work.
  // This can be detected in software as the FPGA asserts BUSY indefinitely.
  int retries;
  for (retries = 0; retries <= 20; ++retries) {
    if (!display_switch_to_bootloader_mode()) {
      // Probably an unconfigured FPGA. Nothing we can do about that.
      break;
    }
    if (boot_display_show_error_code(error_code)) {
      // Success!
      if (retries > 0) {
        PBL_LOG(LOG_LEVEL_WARNING, "Took %d retries to display panic screen.",
                retries);
      }
      break;
    }
  }
  exti_enable(ICE40LP->cdone_exti);

  mutex_unlock(s_display_update_mutex);
}

void display_show_splash_screen(void) {
  // Assumes that the FPGA is already in bootloader mode but the SPI peripheral
  // and GPIOs are not yet configured; exactly the state that the system is
  // expected to be in before display_init() is called.
  display_start();
  display_spi_configure_default();
  boot_display_show_boot_splash();
}

void display_set_offset(GPoint offset) {
  s_disp_offset = offset;
}

GPoint display_get_offset(void) {
  return s_disp_offset;
}

void analytics_external_collect_display_offset(void) {
  analytics_set(ANALYTICS_DEVICE_METRIC_DISPLAY_OFFSET_X, s_disp_offset.x, AnalyticsClient_System);
  analytics_set(ANALYTICS_DEVICE_METRIC_DISPLAY_OFFSET_Y, s_disp_offset.y, AnalyticsClient_System);
}
