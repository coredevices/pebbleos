/*
 * STM32 Microcontroller - Common utility functions
 *
 * Ported to QEMU 10.x APIs
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/arm/stm32_common.h"
#include "qapi/error.h"
#include "qemu/log.h"

static const char *stm32_periph_name_arr[] = {
    [STM32_RCC_PERIPH] = "STM32_RCC",
    [STM32_GPIOA] = "STM32_GPIOA",
    [STM32_GPIOB] = "STM32_GPIOB",
    [STM32_GPIOC] = "STM32_GPIOC",
    [STM32_GPIOD] = "STM32_GPIOD",
    [STM32_GPIOE] = "STM32_GPIOE",
    [STM32_GPIOF] = "STM32_GPIOF",
    [STM32_GPIOG] = "STM32_GPIOG",
    [STM32_GPIOH] = "STM32_GPIOH",
    [STM32_GPIOI] = "STM32_GPIOI",
    [STM32_GPIOJ] = "STM32_GPIOJ",
    [STM32_GPIOK] = "STM32_GPIOK",
    [STM32_SYSCFG] = "STM32_SYSCFG",
    [STM32_UART1] = "STM32_UART1",
    [STM32_UART2] = "STM32_UART2",
    [STM32_UART3] = "STM32_UART3",
    [STM32_UART4] = "STM32_UART4",
    [STM32_UART5] = "STM32_UART5",
    [STM32_UART6] = "STM32_UART6",
    [STM32_UART7] = "STM32_UART7",
    [STM32_UART8] = "STM32_UART8",
    [STM32_SPI1] = "STM32_SPI1",
    [STM32_SPI2] = "STM32_SPI2",
    [STM32_SPI3] = "STM32_SPI3",
    [STM32_TIM1] = "STM32_TIM1",
    [STM32_TIM2] = "STM32_TIM2",
    [STM32_TIM3] = "STM32_TIM3",
    [STM32_TIM4] = "STM32_TIM4",
    [STM32_TIM5] = "STM32_TIM5",
    [STM32_TIM6] = "STM32_TIM6",
    [STM32_TIM7] = "STM32_TIM7",
    [STM32_TIM8] = "STM32_TIM8",
    [STM32_TIM9] = "STM32_TIM9",
    [STM32_TIM10] = "STM32_TIM10",
    [STM32_TIM11] = "STM32_TIM11",
    [STM32_TIM12] = "STM32_TIM12",
    [STM32_TIM13] = "STM32_TIM13",
    [STM32_TIM14] = "STM32_TIM14",
    [STM32_I2C1] = "STM32_I2C1",
    [STM32_I2C2] = "STM32_I2C2",
    [STM32_I2C3] = "STM32_I2C3",
    [STM32_I2C4] = "STM32_I2C4",
    [STM32_EXTI_PERIPH] = "STM32_EXTI",
    [STM32_RTC] = "STM32_RTC",
    [STM32_CRC] = "STM32_CRC",
    [STM32_DMA1] = "STM32_DMA1",
    [STM32_DMA2] = "STM32_DMA2",
    [STM32_QSPI] = "STM32_QSPI",
    [STM32_LPTIM1] = "STM32_LPTIM1",
    [STM32_ADC1] = "STM32_ADC1",
    [STM32_PWR] = "STM32_PWR",
};

const char *stm32_periph_name(stm32_periph_t periph)
{
    if (periph >= 0 && periph < STM32_PERIPH_COUNT &&
        stm32_periph_name_arr[periph]) {
        return stm32_periph_name_arr[periph];
    }
    return "UNKNOWN";
}

void stm32_hw_warn(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    char buf[512];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    qemu_log_mask(LOG_GUEST_ERROR, "STM32 WARNING: %s\n", buf);
    va_end(ap);
}

/*
 * Modern replacement for the old stm32_init_periph().
 * Realizes the device, maps its MMIO region, and optionally connects an IRQ.
 */
DeviceState *stm32_init_periph(DeviceState *dev, stm32_periph_t periph,
                               hwaddr addr, qemu_irq irq)
{
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);

    if (irq) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    }

    return dev;
}
