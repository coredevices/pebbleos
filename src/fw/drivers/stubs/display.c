#include "drivers/display/display.h"
#include "drivers/display/sharp_ls013b7dh01/sharp_ls013b7dh01.h"
#include "system/logging.h"
#include "util/reverse.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "bf0_hal.h"

#define SPI_APB_CLOCK 48000000
#define SPI_FREQ 1000000

static SPI_HandleTypeDef spi = {
  .core = CORE_ID_HCPU,
  .Instance = SPI1,
  .State = HAL_SPI_STATE_RESET,
  .Init = {
    .Direction = SPI_DIRECTION_1LINE,
    .Mode = SPI_MODE_MASTER,
    .DataSize = SPI_DATASIZE_8BIT,
    .CLKPhase  = SPI_PHASE_1EDGE,
    .CLKPolarity = SPI_POLARITY_LOW,
    .BaudRatePrescaler = (SPI_APB_CLOCK + SPI_FREQ / 2) / SPI_FREQ,
    .FrameFormat = SPI_FRAME_FORMAT_SPI,
    .SFRMPol = SPI_SFRMPOL_HIGH,
  },
};

static SemaphoreHandle_t s_sem;

#define DISPLAY_SELECT() HAL_GPIO_WritePin(hwp_gpio1, 29, GPIO_PIN_SET)
#define DISPLAY_DESELECT() HAL_GPIO_WritePin(hwp_gpio1, 29, GPIO_PIN_RESET)

uint32_t display_baud_rate_change(uint32_t new_frequency_hz) {
  return new_frequency_hz;
}

void display_init(void) {
  GPIO_InitTypeDef gpio_init;

  HAL_PIN_Set(PAD_PA29, GPIO_A29, PIN_NOPULL, 1);
  HAL_PIN_Set(PAD_PA24, SPI1_DIO, PIN_PULLDOWN, 1);
  HAL_PIN_Set(PAD_PA28, SPI1_CLK, PIN_NOPULL, 1);

  gpio_init.Pin = 29;
  gpio_init.Mode = GPIO_MODE_OUTPUT;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(hwp_gpio1, &gpio_init);

  DISPLAY_DESELECT();

  HAL_RCC_EnableModule(RCC_MOD_SPI1);

  if (HAL_SPI_Init(&spi) != HAL_OK) {
    PBL_LOG(LOG_LEVEL_ERROR, "spi init err!");
    return;
  }

  vSemaphoreCreateBinary(s_sem);

  display_clear();
}

void display_clear(void) {
  HAL_StatusTypeDef ret;
  uint8_t buf[2];

  buf[0] = 0x20U;
  buf[1] = 0x00U;

  DISPLAY_SELECT();
  ret = HAL_SPI_Transmit(&spi, buf, 2, 1000);
  if (ret != HAL_OK) {
    PBL_LOG(LOG_LEVEL_ERROR, "SPI transmit error");
    return;
  }
  DISPLAY_DESELECT();
}

bool display_update_in_progress(void) {
  if (xSemaphoreTake(s_sem, 0) == pdPASS) {
    xSemaphoreGive(s_sem);
    return false;
  }

  return true;
}

void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {
  DisplayRow row;
  HAL_StatusTypeDef ret;
  uint8_t buf[DISP_DMA_BUFFER_SIZE_BYTES];

  xSemaphoreTake(s_sem, portMAX_DELAY);

  DISPLAY_SELECT();

  buf[0] = 0x80U;
  ret = HAL_SPI_Transmit(&spi, buf, 1, 1000);
  if (ret != HAL_OK) {
    PBL_LOG(LOG_LEVEL_ERROR, "SPI transmit error");
    return;
  }

  while (nrcb(&row)) {
    buf[0] = reverse_byte(row.address + 1);
    for (size_t i = 0; i < DISP_LINE_BYTES; i++) {
      buf[i + 1] = reverse_byte(row.data[i]);
    }
    buf[1 + DISP_LINE_BYTES] = 0x00U;

    ret = HAL_SPI_Transmit(&spi, buf, DISP_LINE_BYTES + 2, 1000);
    if (ret != HAL_OK) {
      PBL_LOG(LOG_LEVEL_ERROR, "SPI transmit error");
      return;
    }
  }

  buf[0] = 0x00U;
  ret = HAL_SPI_Transmit(&spi, buf, 1, 1000);
  if (ret != HAL_OK) {
    PBL_LOG(LOG_LEVEL_ERROR, "SPI transmit error");
    return;
  }

  DISPLAY_DESELECT();

  uccb();

  xSemaphoreGive(s_sem);
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
