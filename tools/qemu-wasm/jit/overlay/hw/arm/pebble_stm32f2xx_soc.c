/*
 * STM32F2xx SoC initialization for Pebble smartwatch (aplite/tintin)
 *
 * Ported from QEMU 2.5.0-pebble8 stm32f2xx.c to QEMU 10.x APIs.
 * Uses ARMv7MState for CPU/NVIC/SysTick and creates Pebble's custom
 * STM32 peripheral models procedurally.
 *
 * Copyright (C) 2010 Andre Beckus
 * Copyright (c) 2013-2016 Pebble Technology
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/boot.h"
#include "hw/arm/stm32_common.h"
#include "hw/arm/pebble.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/misc/unimp.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "hw/ssi/ssi.h"
#include "hw/core/sysbus.h"
#include "hw/block/flash.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "system/system.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/runstate.h"
#include "system/reset.h"
#include "exec/cpu-common.h"
#include "target/arm/cpu-qom.h"
#include "target/arm/cpu.h"

/* SYSCLK frequency (120MHz for STM32F2xx) */
#define SYSCLK_FRQ 120000000ULL

#define FLASH_BASE_ADDRESS   0x08000000
#define SRAM_BASE_ADDRESS    0x20000000

static const char *stm32f2xx_periph_name_arr[] = {
    ENUM_STRING(STM32_UART1),
    ENUM_STRING(STM32_UART2),
    ENUM_STRING(STM32_UART3),
    ENUM_STRING(STM32_UART4),
    ENUM_STRING(STM32_UART5),
    ENUM_STRING(STM32_UART6),
    ENUM_STRING(STM32_SPI1),
    ENUM_STRING(STM32_SPI2),
    ENUM_STRING(STM32_SPI3),
    ENUM_STRING(STM32_I2C1),
    ENUM_STRING(STM32_I2C2),
    ENUM_STRING(STM32_I2C3),
    ENUM_STRING(STM32_TIM1),
    ENUM_STRING(STM32_TIM2),
    ENUM_STRING(STM32_TIM3),
    ENUM_STRING(STM32_TIM4),
    ENUM_STRING(STM32_TIM5),
    ENUM_STRING(STM32_TIM6),
    ENUM_STRING(STM32_TIM7),
    ENUM_STRING(STM32_TIM8),
    ENUM_STRING(STM32_TIM9),
    ENUM_STRING(STM32_TIM10),
    ENUM_STRING(STM32_TIM11),
    ENUM_STRING(STM32_TIM12),
    ENUM_STRING(STM32_TIM13),
    ENUM_STRING(STM32_TIM14),
    ENUM_STRING(STM32_GPIOA),
    ENUM_STRING(STM32_GPIOB),
    ENUM_STRING(STM32_GPIOC),
    ENUM_STRING(STM32_GPIOD),
    ENUM_STRING(STM32_GPIOE),
    ENUM_STRING(STM32_GPIOF),
    ENUM_STRING(STM32_GPIOG),
    ENUM_STRING(STM32_GPIOH),
    ENUM_STRING(STM32_GPIOI),
    ENUM_STRING(STM32_PERIPH_COUNT),
};

static void do_sys_reset(void *opaque, int n, int level)
{
    if (level) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

void stm32f2xx_init(
            ram_addr_t flash_size,        /* in KBytes */
            ram_addr_t ram_size,          /* in KBytes */
            const char *kernel_filename,
            Stm32Gpio **stm32_gpio,
            Stm32Uart **stm32_uart,
            Stm32Timer **stm32_timer,
            DeviceState **stm32_rtc,
            uint32_t osc_freq,
            uint32_t osc32_freq,
            struct stm32f2xx *stm,
            ARMCPU **cpu)
{
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *armv7m_dev;
    int i;
    Error *err = NULL;

    /*
     * Create ARMv7M container (CPU + NVIC + SysTick)
     */
    DeviceState *armv7m_wrapper = qdev_new(TYPE_ARMV7M);
    object_property_add_child(OBJECT(qdev_get_machine()), "armv7m",
                              OBJECT(armv7m_wrapper));

    /* Flash memory region at 0x08000000 */
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    memory_region_init_ram(flash, NULL, "stm32f2xx.flash",
                           flash_size * 1024, &err);
    if (err) {
        error_report_err(err);
        exit(1);
    }
    memory_region_add_subregion(system_memory, FLASH_BASE_ADDRESS, flash);

    /* Flash alias at 0x00000000 */
    MemoryRegion *flash_alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(flash_alias, NULL, "stm32f2xx.flash.alias",
                             flash, 0, flash_size * 1024);
    memory_region_add_subregion(system_memory, 0, flash_alias);

    /* SRAM at 0x20000000 */
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    memory_region_init_ram(sram, NULL, "stm32f2xx.sram",
                           ram_size * 1024, &err);
    if (err) {
        error_report_err(err);
        exit(1);
    }
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, sram);

    /* No CCM on STM32F2xx (unlike F4xx) */

    /* Create sysclk and refclk */
    Clock *sysclk = clock_new(OBJECT(qdev_get_machine()), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    Clock *refclk = clock_new(OBJECT(qdev_get_machine()), "REFCLK");
    clock_set_mul_div(refclk, 8, 1);
    clock_set_source(refclk, sysclk);

    /* Configure ARMv7M — cortex-m3 for STM32F2xx */
    qdev_prop_set_uint32(armv7m_wrapper, "num-irq", STM32_MAX_IRQ);
    qdev_prop_set_uint8(armv7m_wrapper, "num-prio-bits", 4);
    qdev_prop_set_string(armv7m_wrapper, "cpu-type",
                         ARM_CPU_TYPE_NAME("cortex-m3"));
    qdev_prop_set_bit(armv7m_wrapper, "enable-bitband", true);
    qdev_connect_clock_in(armv7m_wrapper, "cpuclk", sysclk);
    qdev_connect_clock_in(armv7m_wrapper, "refclk", refclk);
    object_property_set_link(OBJECT(armv7m_wrapper), "memory",
                             OBJECT(system_memory), &error_abort);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(armv7m_wrapper), &error_fatal);

    armv7m_dev = armv7m_wrapper;

    /* Get ARMCPU for callers that need it */
    *cpu = ARM_CPU(first_cpu);

    /* Connect SYSRESETREQ */
    qdev_connect_gpio_out_named(armv7m_dev, "SYSRESETREQ", 0,
                                qemu_allocate_irq(&do_sys_reset, NULL, 0));

    /* Load firmware into flash memory */
    if (kernel_filename) {
        void *flash_buf = memory_region_get_ram_ptr(flash);
        ssize_t image_size = load_image_size(kernel_filename, flash_buf,
                                              flash_size * 1024);
        if (image_size < 0) {
            error_report("Could not load kernel '%s'", kernel_filename);
            exit(1);
        }
        uint32_t *vt = (uint32_t *)flash_buf;
        fprintf(stderr, "DEBUG: Loaded %zd bytes into flash at %p\n",
                image_size, flash_buf);
        fprintf(stderr, "DEBUG: Vector table: SP=0x%08x PC=0x%08x\n",
                vt[0], vt[1]);
    }

    armv7m_load_kernel(*cpu, kernel_filename,
                       FLASH_BASE_ADDRESS, flash_size * 1024);

    Object *stm32_container = object_new("container");
    object_property_add_child(OBJECT(qdev_get_machine()), "stm32",
                              stm32_container);

    /* === RCC === */
    DeviceState *rcc_dev = qdev_new("stm32f2xx_rcc");
    qdev_prop_set_uint32(rcc_dev, "osc_freq", osc_freq);
    qdev_prop_set_uint32(rcc_dev, "osc32_freq", osc32_freq);
    object_property_add_child(stm32_container, "rcc", OBJECT(rcc_dev));
    stm32_init_periph(rcc_dev, STM32_RCC_PERIPH, 0x40023800,
                      qdev_get_gpio_in(armv7m_dev, STM32_RCC_IRQ));

    /* === GPIOs (9 banks: A-I) === */
    DeviceState **gpio_dev = g_malloc0(sizeof(DeviceState *) *
                                       STM32F2XX_GPIO_COUNT);
    for (i = 0; i < STM32F2XX_GPIO_COUNT; i++) {
        stm32_periph_t periph = STM32_GPIOA + i;
        gpio_dev[i] = qdev_new("stm32f2xx_gpio");
        qdev_prop_set_int32(gpio_dev[i], "periph", periph);
        qdev_prop_set_uint32(gpio_dev[i], "idr-mask", 0);
        stm32_init_periph(gpio_dev[i], periph,
                          0x40020000 + (i * 0x400), NULL);
        stm32_gpio[i] = (Stm32Gpio *)gpio_dev[i];
    }

    /* === EXTI === */
    DeviceState *exti_dev = qdev_new("stm32-exti");
    stm32_exti_set_gpio_array(STM32_EXTI(exti_dev), (stm32f2xx_gpio **)gpio_dev);
    stm32_init_periph(exti_dev, STM32_EXTI_PERIPH, 0x40013C00, NULL);
    SysBusDevice *exti_busdev = SYS_BUS_DEVICE(exti_dev);

    /* EXTI -> NVIC IRQ connections */
    sysbus_connect_irq(exti_busdev, 0,
                       qdev_get_gpio_in(armv7m_dev, STM32_EXTI0_IRQ));
    sysbus_connect_irq(exti_busdev, 1,
                       qdev_get_gpio_in(armv7m_dev, STM32_EXTI1_IRQ));
    sysbus_connect_irq(exti_busdev, 2,
                       qdev_get_gpio_in(armv7m_dev, STM32_EXTI2_IRQ));
    sysbus_connect_irq(exti_busdev, 3,
                       qdev_get_gpio_in(armv7m_dev, STM32_EXTI3_IRQ));
    sysbus_connect_irq(exti_busdev, 4,
                       qdev_get_gpio_in(armv7m_dev, STM32_EXTI4_IRQ));
    sysbus_connect_irq(exti_busdev, 5,
                       qdev_get_gpio_in(armv7m_dev, STM32_EXTI9_5_IRQ));
    sysbus_connect_irq(exti_busdev, 6,
                       qdev_get_gpio_in(armv7m_dev, STM32_EXTI15_10_IRQ));
    sysbus_connect_irq(exti_busdev, 7,
                       qdev_get_gpio_in(armv7m_dev, STM32_PVD_IRQ));
    sysbus_connect_irq(exti_busdev, 8,
                       qdev_get_gpio_in(armv7m_dev, STM32_RTCAlarm_IRQ));
    sysbus_connect_irq(exti_busdev, 9,
                       qdev_get_gpio_in(armv7m_dev, STM32_OTG_FS_WKUP_IRQ));
    sysbus_connect_irq(exti_busdev, 10,
                       qdev_get_gpio_in(armv7m_dev, STM32_ETH_WKUP_IRQ));
    sysbus_connect_irq(exti_busdev, 11,
                       qdev_get_gpio_in(armv7m_dev, STM32_OTG_FS_WKUP_IRQ));
    sysbus_connect_irq(exti_busdev, 12,
                       qdev_get_gpio_in(armv7m_dev, STM32_TAMP_STAMP_IRQ));
    sysbus_connect_irq(exti_busdev, 13,
                       qdev_get_gpio_in(armv7m_dev, STM32_RTC_WKUP_IRQ));

    /* === SYSCFG === */
    DeviceState *syscfg_dev = qdev_new("stm32f2xx_syscfg");
    stm32_syscfg_set_links(syscfg_dev, (Stm32Rcc *)rcc_dev, STM32_EXTI(exti_dev));
    stm32_init_periph(syscfg_dev, STM32_SYSCFG, 0x40013800, NULL);

    /* === UARTs (6 on F2xx) === */
    struct {
        uint32_t addr;
        uint8_t irq_idx;
    } const uart_desc[] = {
        {0x40011000, STM32_UART1_IRQ},
        {0x40004400, STM32_UART2_IRQ},
        {0x40004800, STM32_UART3_IRQ},
        {0x40004C00, STM32_UART4_IRQ},
        {0x40005000, STM32_UART5_IRQ},
        {0x40011400, STM32_UART6_IRQ},
    };
    for (i = 0; i < ARRAY_LENGTH(uart_desc); ++i) {
        assert(i < STM32F2XX_UART_COUNT);
        const stm32_periph_t periph = STM32_UART1 + i;
        DeviceState *uart_dev = qdev_new("stm32-uart");
        qdev_prop_set_int32(uart_dev, "periph", periph);
        stm32_init_periph(uart_dev, periph, uart_desc[i].addr,
                          qdev_get_gpio_in(armv7m_dev, uart_desc[i].irq_idx));
        stm32_uart[i] = (Stm32Uart *)uart_dev;
    }

    /* === SPI (3 on F2xx) === */
    struct {
        uint32_t addr;
        uint8_t irq_idx;
    } const spi_desc[] = {
        {0x40013000, STM32_SPI1_IRQ},
        {0x40003800, STM32_SPI2_IRQ},
        {0x40003C00, STM32_SPI3_IRQ},
    };
    for (i = 0; i < ARRAY_LENGTH(spi_desc); ++i) {
        assert(i < STM32F2XX_SPI_COUNT);
        const stm32_periph_t periph = STM32_SPI1 + i;
        stm->spi_dev[i] = qdev_new("stm32f2xx_spi");
        qdev_prop_set_int32(stm->spi_dev[i], "periph", periph);
        stm32_init_periph(stm->spi_dev[i], periph, spi_desc[i].addr,
                          qdev_get_gpio_in(armv7m_dev, spi_desc[i].irq_idx));
    }

    /* === ADC === */
    DeviceState *adc_dev = qdev_new("stm32f2xx_adc");
    stm32_init_periph(adc_dev, STM32_ADC1, 0x40012000, NULL);

    /* === RTC === */
    DeviceState *rtc_dev = qdev_new("f2xx_rtc");
    *stm32_rtc = rtc_dev;
    stm32_init_periph(rtc_dev, STM32_RTC, 0x40002800, NULL);
    /* Alarm A */
    sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 0,
                       qdev_get_gpio_in(exti_dev, 17));
    /* Alarm B */
    sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 1,
                       qdev_get_gpio_in(exti_dev, 17));
    /* Wake up timer */
    sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 2,
                       qdev_get_gpio_in(exti_dev, 22));

    /* === PWR === */
    DeviceState *pwr_dev = qdev_new("f2xx_pwr");
    stm32_init_periph(pwr_dev, STM32_PWR, 0x40007000, NULL);

    /* === Timers === */
    struct {
        uint8_t timer_num;
        uint32_t addr;
        uint8_t irq_idx;
    } const timer_desc[] = {
        {2,  0x40000000, STM32_TIM2_IRQ},
        {3,  0x40000400, STM32_TIM3_IRQ},
        {4,  0x40000800, STM32_TIM4_IRQ},
        {5,  0x40000C00, STM32_TIM5_IRQ},
        {6,  0x40001000, STM32_TIM6_IRQ},
        {7,  0x40001400, STM32_TIM7_IRQ},
        {9,  0x40014000, STM32_TIM1_BRK_TIM9_IRQ},
        {10, 0x40014400, STM32_TIM1_UP_TIM10_IRQ},
        {11, 0x40014800, STM32_TIM1_TRG_COM_TIM11_IRQ},
        {12, 0x40001800, STM32_TIM8_BRK_TIM12_IRQ},
        {13, 0x40001C00, STM32_TIM8_UP_TIM13_IRQ},
        {14, 0x40002000, STM32_TIM8_TRG_COMM_TIM14_IRQ},
    };
    for (i = 0; i < ARRAY_LENGTH(timer_desc); ++i) {
        assert(i < STM32F2XX_TIM_COUNT);
        const stm32_periph_t periph = STM32_TIM1 + timer_desc[i].timer_num - 1;
        DeviceState *timer = qdev_new("f2xx_tim");
        stm32_init_periph(timer, periph, timer_desc[i].addr,
                          qdev_get_gpio_in(armv7m_dev, timer_desc[i].irq_idx));
        stm32_timer[timer_desc[i].timer_num - 1] = (Stm32Timer *)timer;
    }

    /* === I2C === */
    DeviceState *i2c1 = qdev_new("f2xx_i2c");
    qdev_prop_set_int32(i2c1, "periph", STM32_I2C1);
    stm32_init_periph(i2c1, STM32_I2C1, 0x40005400,
                      qdev_get_gpio_in(armv7m_dev, STM32_I2C1_EV_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(i2c1), 1,
                       qdev_get_gpio_in(armv7m_dev, STM32_I2C1_ER_IRQ));

    DeviceState *i2c2 = qdev_new("f2xx_i2c");
    qdev_prop_set_int32(i2c2, "periph", STM32_I2C2);
    stm32_init_periph(i2c2, STM32_I2C2, 0x40005800,
                      qdev_get_gpio_in(armv7m_dev, STM32_I2C2_EV_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(i2c2), 1,
                       qdev_get_gpio_in(armv7m_dev, STM32_I2C2_ER_IRQ));

    DeviceState *i2c3 = qdev_new("f2xx_i2c");
    qdev_prop_set_int32(i2c3, "periph", STM32_I2C3);
    stm32_init_periph(i2c3, STM32_I2C3, 0x40005C00,
                      qdev_get_gpio_in(armv7m_dev, STM32_I2C3_EV_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(i2c3), 1,
                       qdev_get_gpio_in(armv7m_dev, STM32_I2C3_ER_IRQ));

    /* === CRC === */
    DeviceState *crc = qdev_new("f2xx_crc");
    stm32_init_periph(crc, STM32_CRC, 0x40023000, NULL);

    /* === DMA === */
    static const uint8_t dma1_irqs[8] = {
        STM32_DMA1_STREAM0_IRQ, STM32_DMA1_STREAM1_IRQ,
        STM32_DMA1_STREAM2_IRQ, STM32_DMA1_STREAM3_IRQ,
        STM32_DMA1_STREAM4_IRQ, STM32_DMA1_STREAM5_IRQ,
        STM32_DMA1_STREAM6_IRQ, STM32_DMA1_STREAM7_IRQ,
    };
    DeviceState *dma1 = qdev_new("f2xx_dma");
    stm32_init_periph(dma1, STM32_DMA1, 0x40026000, NULL);
    for (i = 0; i < 8; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dma1), i,
                           qdev_get_gpio_in(armv7m_dev, dma1_irqs[i]));
    }

    static const uint8_t dma2_irqs[8] = {
        STM32_DMA2_STREAM0_IRQ, STM32_DMA2_STREAM1_IRQ,
        STM32_DMA2_STREAM2_IRQ, STM32_DMA2_STREAM3_IRQ,
        STM32_DMA2_STREAM4_IRQ, STM32_DMA2_STREAM5_IRQ,
        STM32_DMA2_STREAM6_IRQ, STM32_DMA2_STREAM7_IRQ,
    };
    DeviceState *dma2 = qdev_new("f2xx_dma");
    stm32_init_periph(dma2, STM32_DMA2, 0x40026400, NULL);
    for (i = 0; i < 8; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dma2), i,
                           qdev_get_gpio_in(armv7m_dev, dma2_irqs[i]));
    }

    /* === Unimplemented stubs === */
    create_unimplemented_device("WWDG",    0x40002C00, 0x400);
    create_unimplemented_device("IWDG",    0x40003000, 0x400);
    create_unimplemented_device("SDIO",    0x40012C00, 0x400);
    create_unimplemented_device("BxCAN1",  0x40006400, 0x400);
    create_unimplemented_device("BxCAN2",  0x40006800, 0x400);
    create_unimplemented_device("DAC",     0x40007400, 0x400);
    create_unimplemented_device("FlashIF", 0x40023C00, 0x400);
    create_unimplemented_device("BKPSRAM", 0x40024000, 0x400);
    create_unimplemented_device("USB_OTG_FS", 0x50000000, 0x31000);
}
