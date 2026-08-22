/*
 * Pebble Smartwatch - Board Configuration
 *
 * Copyright (c) 2013, 2014 Pebble Technology
 * Ported to QEMU 10.x APIs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_ARM_PEBBLE_H
#define HW_ARM_PEBBLE_H

#include <stdint.h>
#include <stdbool.h>
#include "hw/arm/stm32_common.h"

/* Forward declaration - ARMCPU is typedef'd in target/arm/cpu-qom.h */
struct ArchCPU;
#ifndef ARMCPU
typedef struct ArchCPU ARMCPU;
#endif

typedef enum {
    PBL_BUTTON_ID_NONE = -1,
    PBL_BUTTON_ID_BACK = 0,
    PBL_BUTTON_ID_UP = 1,
    PBL_BUTTON_ID_SELECT = 2,
    PBL_BUTTON_ID_DOWN = 3,
    PBL_NUM_BUTTONS = 4
} PblButtonID;

typedef struct {
    int gpio;
    int pin;
    bool active_high;
} PblButtonMap;

/* Peripheral counts for different STM32 families */
#define STM32F2XX_GPIO_COUNT  9
#define STM32F2XX_UART_COUNT  6
#define STM32F2XX_TIM_COUNT   14
#define STM32F2XX_SPI_COUNT   3

#define STM32F4XX_GPIO_COUNT  11
#define STM32F4XX_UART_COUNT  8
#define STM32F4XX_TIM_COUNT   14
#define STM32F4XX_SPI_COUNT   6

#define STM32F7XX_GPIO_COUNT  11
#define STM32F7XX_UART_COUNT  8
#define STM32F7XX_TIM_COUNT   14
#define STM32F7XX_SPI_COUNT   6

typedef struct {
    int dbgserial_uart_index;
    int pebble_control_uart_index;

    PblButtonMap button_map[PBL_NUM_BUTTONS];
    uint32_t gpio_idr_masks[STM32F4XX_GPIO_COUNT];

    /* memory sizes in KBytes */
    uint32_t flash_size;
    uint32_t ram_size;

    /* screen sizes */
    uint32_t num_rows;
    uint32_t num_cols;
    uint32_t num_border_rows;
    uint32_t num_border_cols;
    bool row_major;
    bool row_inverted;
    bool col_inverted;
    bool round_mask;
} PblBoardConfig;

/* SoC context returned from init functions */
struct stm32f2xx {
    DeviceState *spi_dev[STM32F2XX_SPI_COUNT];
};

struct stm32f4xx {
    DeviceState *spi_dev[STM32F4XX_SPI_COUNT];
    DeviceState *qspi_dev;
};

struct stm32f7xx {
    DeviceState *spi_dev[STM32F7XX_SPI_COUNT];
    DeviceState *qspi_dev;
};

/* SoC init functions */
void stm32f2xx_init(
            ram_addr_t flash_size,
            ram_addr_t ram_size,
            const char *kernel_filename,
            Stm32Gpio **stm32_gpio,
            Stm32Uart **stm32_uart,
            Stm32Timer **stm32_timer,
            DeviceState **stm32_rtc,
            uint32_t osc_freq,
            uint32_t osc32_freq,
            struct stm32f2xx *stm,
            ARMCPU **cpu);

void stm32f4xx_init(
            ram_addr_t flash_size,
            ram_addr_t ram_size,
            const char *kernel_filename,
            Stm32Gpio **stm32_gpio,
            const uint32_t *gpio_idr_masks,
            Stm32Uart **stm32_uart,
            Stm32Timer **stm32_timer,
            DeviceState **stm32_rtc,
            uint32_t osc_freq,
            uint32_t osc32_freq,
            struct stm32f4xx *stm,
            ARMCPU **cpu);

/* Board init functions */
void pebble_32f2_init(MachineState *machine, const PblBoardConfig *board_config);
void pebble_32f412_init(MachineState *machine, const PblBoardConfig *board_config);
void pebble_32f439_init(MachineState *machine, const PblBoardConfig *board_config);
void pebble_32f7xx_init(MachineState *machine, const PblBoardConfig *board_config);

/* Helper functions used across board files */
void pebble_set_button_state(uint32_t button_state);
void pebble_set_qemu_settings(DeviceState *rtc_dev);
void pebble_connect_uarts(Stm32Uart *uart[], const PblBoardConfig *board_config);
void pebble_init_buttons(Stm32Gpio *gpio[], const PblButtonMap *map);
DeviceState *pebble_init_board(Stm32Gpio *gpio[], qemu_irq display_vibe);

/* MX25U flash memory-mapped region accessor */
struct MemoryRegion;
MemoryRegion *mx25u_get_mmap(DeviceState *dev);

/* F7xx UART type forward declarations (stub for now) */
typedef struct Stm32F7xxUart Stm32F7xxUart;

#endif /* HW_ARM_PEBBLE_H */
