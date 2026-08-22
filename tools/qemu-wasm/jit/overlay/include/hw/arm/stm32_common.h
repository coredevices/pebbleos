/*
 * STM32 Microcontroller - Common definitions
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef STM32_COMMON_H
#define STM32_COMMON_H

#include "qemu/osdep.h"
#include "qemu/typedefs.h"
#include "qemu/timer.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "qapi/error.h"
#include "chardev/char-fe.h"


#define ENUM_STRING(x) [x] = #x
#define ARRAY_LENGTH(array) (sizeof((array))/sizeof((array)[0]))

/* COMMON */
#define BYTE_ACCESS_SIZE 1
#define HALFWORD_ACCESS_SIZE 2
#define WORD_ACCESS_SIZE 4

#define STM32_FLASH_ADDR_START (0x08000000)

/* VALUE_BETWEEN is inclusive */
#define VALUE_BETWEEN(value, start, end) ((value >= start) && (value <= end))

#define GET_BIT_MASK(position, value) ((value ? 1 : 0) << position)
#define GET_BIT_MASK_ONE(position) (1 << position)
#define GET_BIT_MASK_ZERO(position) (~(1 << position))
#define GET_BIT_VALUE(value, position) \
                ((value & GET_BIT_MASK_ONE(position)) >> position)
#define IS_BIT_SET(value, position) ((value & GET_BIT_MASK_ONE(position)) != 0)
#define IS_BIT_RESET(value, position) ((value & GET_BIT_MASK_ONE(position)) ==0)
#define SET_BIT(var, position)   var |= GET_BIT_MASK_ONE(position)
#define RESET_BIT(var, position) var &= GET_BIT_MASK_ZERO(position)

/* Can be true, false, 0, or 1 */
#define CHANGE_BIT(var, position, new_value) \
            var = new_value ? \
                    (var | GET_BIT_MASK_ONE(position)) : \
                    (var & GET_BIT_MASK_ZERO(position))
#define CHANGE_BITS(var, start, mask, new_value) \
            var = (var & ~mask) | ((new_value << start) & mask)

void stm32_hw_warn(const char *fmt, ...)
    __attribute__ ((__format__ (__printf__, 1, 2)));

#define stm32_unimp(x...) qemu_log_mask(LOG_UNIMP, x)




/* PERIPHERALS - COMMON */
/* Indexes used for accessing a GPIO array */
#define STM32_GPIOA_INDEX 0
#define STM32_GPIOB_INDEX 1
#define STM32_GPIOC_INDEX 2
#define STM32_GPIOD_INDEX 3
#define STM32_GPIOE_INDEX 4
#define STM32_GPIOF_INDEX 5
#define STM32_GPIOG_INDEX 6
#define STM32_GPIOH_INDEX 7
#define STM32_GPIOI_INDEX 8

/* Indexes used for accessing a UART array */
#define STM32_UART1_INDEX 0
#define STM32_UART2_INDEX 1
#define STM32_UART3_INDEX 2
#define STM32_UART4_INDEX 3
#define STM32_UART5_INDEX 4
#define STM32_UART6_INDEX 5

/* Used for uniquely identifying a peripheral */
typedef int32_t stm32_periph_t;

enum {
    STM32_PERIPH_UNDEFINED = -1,
    STM32_RCC_PERIPH = 0,
    STM32_GPIOA,
    STM32_GPIOB,
    STM32_GPIOC,
    STM32_GPIOD,
    STM32_GPIOE,
    STM32_GPIOF,
    STM32_GPIOG,
    STM32_GPIOH,
    STM32_GPIOI,
    STM32_GPIOJ,
    STM32_GPIOK,
    STM32_SYSCFG,
    STM32_AFIO_PERIPH,
    STM32_UART1,
    STM32_UART2,
    STM32_UART3,
    STM32_UART4,
    STM32_UART5,
    STM32_UART6,
    STM32_UART7,
    STM32_UART8,
    STM32_ADC1,
    STM32_ADC2,
    STM32_ADC3,
    STM32_DAC,
    STM32_TIM1,
    STM32_TIM2,
    STM32_TIM3,
    STM32_TIM4,
    STM32_TIM5,
    STM32_TIM6,
    STM32_TIM7,
    STM32_TIM8,
    STM32_TIM9,
    STM32_TIM10,
    STM32_TIM11,
    STM32_TIM12,
    STM32_TIM13,
    STM32_TIM14,
    STM32_BKP,
    STM32_PWR,
    STM32_I2C1,
    STM32_I2C2,
    STM32_I2C3,
    STM32_I2C4,
    STM32_I2S2,
    STM32_I2S3,
    STM32_WWDG,
    STM32_CAN1,
    STM32_CAN2,
    STM32_CAN,
    STM32_USB,
    STM32_SPI1,
    STM32_SPI2,
    STM32_SPI3,
    STM32_EXTI_PERIPH,
    STM32_SDIO,
    STM32_FSMC,
    STM32_RTC,
    STM32_CRC,
    STM32_DMA1,
    STM32_DMA2,
    STM32_DCMI_PERIPH,
    STM32_CRYP_PERIPH,
    STM32_HASH_PERIPH,
    STM32_RNG_PERIPH,
    STM32_QSPI,
    STM32_LPTIM1,

    STM32_PERIPH_COUNT,
};

const char *stm32_periph_name(stm32_periph_t periph);

/* Convert between a GPIO array index and stm32_periph_t, and vice-versa */
#define STM32_GPIO_INDEX_FROM_PERIPH(gpio_periph) (gpio_periph - STM32_GPIOA)
#define STM32_GPIO_PERIPH_FROM_INDEX(gpio_index) (STM32_GPIOA + gpio_index)




/* REGISTER HELPERS */
/* Macros used for converting a half-word into a word.
 * Assume that memory alignment can be determined by looking at offset
 * i.e. the base address should always be 4 byte aligned.
 * Also assume that odd offsets will never occur
 * i.e. all offsets must be 2 byte aligned.
 */
#define STM32_REG_READH_VALUE(offset, value32) \
          ((offset & 3) ? \
            (value32 & 0xffff0000) >> 16 : \
            value32 & 0x0000ffff)
#define STM32_REG_WRITEH_VALUE(offset, old_value32, new_value32) \
          ((offset & 3) ? \
            (old_value32 & 0x0000ffff) | ((new_value32 & 0x0000ffff) << 16) : \
            (old_value32 & 0xffff0000) | (new_value32 & 0x0000ffff) )

/* Error handlers */
# define STM32_BAD_REG(offset, size)       \
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad register 0x%x - size %u\n", \
                      __FUNCTION__, (int)offset, size)
# define STM32_WARN_RO_REG(offset)        \
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Read-only register 0x%x\n", \
                      __FUNCTION__, (int)offset)
# define STM32_WARN_WO_REG(offset)        \
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Write-only register 0x%x\n", \
                      __FUNCTION__, (int)offset)
# define STM32_NOT_IMPL_REG(offset, size)      \
        qemu_log_mask(LOG_UNIMP, "%s: Not implemented yet 0x%x - size %u\n", \
                      __FUNCTION__, (int)offset, size)



/* IRQs */
#define STM32_PVD_IRQ 1
#define STM32_TAMP_STAMP_IRQ 2
#define STM32_RTC_WKUP_IRQ 3
#define STM32_RCC_IRQ 5
#define STM32_EXTI0_IRQ 6
#define STM32_EXTI1_IRQ 7
#define STM32_EXTI2_IRQ 8
#define STM32_EXTI3_IRQ 9
#define STM32_EXTI4_IRQ 10

#define STM32_DMA1_STREAM0_IRQ 11
#define STM32_DMA1_STREAM1_IRQ 12
#define STM32_DMA1_STREAM2_IRQ 13
#define STM32_DMA1_STREAM3_IRQ 14
#define STM32_DMA1_STREAM4_IRQ 15
#define STM32_DMA1_STREAM5_IRQ 16
#define STM32_DMA1_STREAM6_IRQ 17

#define STM32_EXTI9_5_IRQ 23
#define STM32_TIM1_BRK_TIM9_IRQ 24
#define STM32_TIM1_UP_TIM10_IRQ 25
#define STM32_TIM1_TRG_COM_TIM11_IRQ 26
#define STM32_TIM1_CC_IRQ 27
#define STM32_TIM2_IRQ 28
#define STM32_TIM3_IRQ 29
#define STM32_TIM4_IRQ 30

#define STM32_I2C1_EV_IRQ 31
#define STM32_I2C1_ER_IRQ 32
#define STM32_I2C2_EV_IRQ 33
#define STM32_I2C2_ER_IRQ 34

#define STM32_SPI1_IRQ 35
#define STM32_SPI2_IRQ 36

#define STM32_UART1_IRQ 37
#define STM32_UART2_IRQ 38
#define STM32_UART3_IRQ 39
#define STM32_EXTI15_10_IRQ 40
#define STM32_RTCAlarm_IRQ 41
#define STM32_OTG_FS_WKUP_IRQ 42
#define STM32_TIM8_BRK_TIM12_IRQ 43
#define STM32_TIM8_UP_TIM13_IRQ 44
#define STM32_TIM8_TRG_COMM_TIM14_IRQ 45

#define STM32_DMA1_STREAM7_IRQ 47

#define STM32_TIM5_IRQ 50
#define STM32_SPI3_IRQ 51
#define STM32_UART4_IRQ 52
#define STM32_UART5_IRQ 53
#define STM32_TIM6_IRQ 54
#define STM32_TIM7_IRQ 55

#define STM32_DMA2_STREAM0_IRQ 56
#define STM32_DMA2_STREAM1_IRQ 57
#define STM32_DMA2_STREAM2_IRQ 58
#define STM32_DMA2_STREAM3_IRQ 59
#define STM32_DMA2_STREAM4_IRQ 60

#define STM32_ETH_WKUP_IRQ 62

#define STM32_DMA2_STREAM5_IRQ 68
#define STM32_DMA2_STREAM6_IRQ 69
#define STM32_DMA2_STREAM7_IRQ 70

#define STM32_UART6_IRQ 71

#define STM32_I2C3_EV_IRQ 72
#define STM32_I2C3_ER_IRQ 73

#define STM32_SPI4_IRQ 84
#define STM32_SPI5_IRQ 85
#define STM32_SPI6_IRQ 86

#define STM32_QSPI_IRQ 92
#define STM32_LPTIM1_IRQ 93

#define STM32_I2C4_EV_IRQ 95
#define STM32_I2C4_ER_IRQ 96

#define STM32_MAX_IRQ  127



/* EXTI */
typedef struct Stm32Exti Stm32Exti;

#define TYPE_STM32_EXTI "stm32-exti"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32Exti, STM32_EXTI)

/* Assigns the specified EXTI line to the specified GPIO. */
void stm32_exti_set_gpio(Stm32Exti *s, unsigned exti_line, const uint8_t gpio_index);

/* Unassigns the specified EXTI line from the specified GPIO. */
void stm32_exti_reset_gpio(Stm32Exti *s, unsigned exti_line, const uint8_t gpio_index);



/* RTC */
/* Set the value of one of the QEMU specific extra RTC backup registers.
 * The idx value starts at 0 for the first extra register */
void f2xx_rtc_set_extra_bkup_reg(void *opaque, uint32_t idx, uint32_t value);


/* GPIO */
typedef struct Stm32Gpio Stm32Gpio;

#define TYPE_STM32_GPIO "stm32-gpio"
#define STM32_GPIO(obj) OBJECT_CHECK(Stm32Gpio, (obj), TYPE_STM32_GPIO)

#define STM32_GPIO_COUNT (STM32_GPIOG - STM32_GPIOA + 1)
#define STM32_GPIO_PIN_COUNT 16


/* Sets the EXTI IRQ for the specified pin.  When a change occurs
 * on this pin, an interrupt will be generated on this IRQ.
 */
void stm32_gpio_set_exti_irq(Stm32Gpio *s, unsigned pin, qemu_irq in_irq);

/* GPIO pin mode */
#define STM32_GPIO_MODE_IN 0
#define STM32_GPIO_MODE_OUT_10MHZ 1
#define STM32_GPIO_MODE_OUT_2MHZ 2
#define STM32_GPIO_MODE_OUT_50MHZ 3
uint8_t stm32_gpio_get_mode_bits(Stm32Gpio *s, unsigned pin);

/* GPIO pin config */
#define STM32_GPIO_IN_ANALOG 0
#define STM32_GPIO_IN_FLOAT 1
#define STM32_GPIO_IN_PULLUPDOWN 2
#define STM32_GPIO_OUT_PUSHPULL 0
#define STM32_GPIO_OUT_OPENDRAIN 1
#define STM32_GPIO_OUT_ALT_PUSHPULL 2
#define STM32_GPIO_OUT_ALT_OPEN 3
uint8_t stm32_gpio_get_config_bits(Stm32Gpio *s, unsigned pin);




/* GPIO - f2xx */
typedef struct stm32f2xx_gpio stm32f2xx_gpio;
void f2xx_gpio_exti_set(stm32f2xx_gpio *, unsigned, qemu_irq);
void f2xx_gpio_wake_set(stm32f2xx_gpio *, unsigned, qemu_irq);




/* RCC */
typedef struct Stm32Rcc Stm32Rcc;

#define TYPE_STM32_RCC "stm32-rcc"
#define STM32_RCC(obj) OBJECT_CHECK(Stm32Rcc, (obj), TYPE_STM32_RCC)

/* Checks if the specified peripheral clock is enabled.
 * Generates a hardware error if not.
 */
void stm32_rcc_check_periph_clk(Stm32Rcc *s, stm32_periph_t periph);

/* Sets the IRQ to be called when the specified peripheral clock changes
 * frequency. */
void stm32_rcc_set_periph_clk_irq(
        Stm32Rcc *s,
        stm32_periph_t periph,
        qemu_irq periph_irq);

/* Gets the frequency of the specified peripheral clock. */
uint32_t stm32_rcc_get_periph_freq(
        Stm32Rcc *s,
        stm32_periph_t periph);


/* EXTI - board/SoC wiring (replaces old DEFINE_PROP_PTR) */
void stm32_exti_set_gpio_array(Stm32Exti *s, stm32f2xx_gpio **gpio);

/* SYSCFG */
#define TYPE_STM32F2XX_SYSCFG "stm32f2xx_syscfg"

/* Sets SYSCFG's RCC and EXTI links (board/SoC wiring, replaces old DEFINE_PROP_PTR). */
void stm32_syscfg_set_links(DeviceState *dev, Stm32Rcc *rcc, Stm32Exti *exti);


/* TIM */
typedef struct Stm32Timer Stm32Timer;
#define STM32_TIM_COUNT   14


/* LPTIM */
typedef struct Stm32F7xxLPTimer Stm32F7xxLPTimer;


/* UART */
#define STM32_UART_COUNT 5

typedef struct Stm32Uart Stm32Uart;

#define TYPE_STM32_UART "stm32-uart"
#define STM32_UART(obj) OBJECT_CHECK(Stm32Uart, (obj), TYPE_STM32_UART)

/* Connect a Chardev to this UART (sets up chardev handlers) */
void stm32_uart_connect(Stm32Uart *s, Chardev *chr, uint32_t afio_board_map);

/* Set a custom write handler (used by pebble_control to intercept TX data) */
void stm32_uart_set_write_handler(Stm32Uart *s, void *obj,
        int (*chr_write_handler)(void *chr_write_obj, const uint8_t *buf, int len));

/* Get the UART's receive handlers (used by pebble_control to forward RX data) */
void stm32_uart_get_rcv_handlers(Stm32Uart *s, IOCanReadHandler **can_read,
                                  IOReadHandler **read, IOEventHandler **event);


/* AFIO */
#define TYPE_STM32_AFIO "stm32-afio"
#define STM32_AFIO(obj) OBJECT_CHECK(Stm32Afio, (obj), TYPE_STM32_AFIO)

typedef struct Stm32Afio Stm32Afio;

/* AFIO Peripheral Mapping */
#define STM32_USART1_NO_REMAP 0
#define STM32_USART1_REMAP 1

#define STM32_USART2_NO_REMAP 0
#define STM32_USART2_REMAP 1

#define STM32_USART3_NO_REMAP 0
#define STM32_USART3_PARTIAL_REMAP 1
#define STM32_USART3_FULL_REMAP 3

/* Gets the pin mapping for the specified peripheral.  Will return one
 * of the mapping values defined above. */
uint32_t stm32_afio_get_periph_map(Stm32Afio *s, int32_t periph_num);


/* STM32 PERIPHERALS - GENERAL */
DeviceState *stm32_init_periph(DeviceState *dev, stm32_periph_t periph,
                               hwaddr addr, qemu_irq irq);


/* STM32 MICROCONTROLLER - GENERAL */
typedef struct Stm32 Stm32;

#endif /* STM32_COMMON_H */
