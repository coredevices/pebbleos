/*
 * SPDX-FileCopyrightText: 2026 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

    .ramfunc : {
        . = ALIGN(4);
        __ramfunc_start = .;
        /* The object file extension is the build system's to choose --
           CMake writes .obj -- so the patterns end wild. */
        * (.*l1_ret_text_*)
        * (.*l1_ret_rodata_*)

        *flash_table.*.o*      (.text* .rodata*)
        *bf0_hal_mpi.*.o*      (.text* .rodata*)
        *bf0_hal_mpi_ex.*.o*   (.text* .rodata*)
        *bf0_hal_mpi_psram.*.o* (.text* .rodata*)
        */flash.*.o*            (.text* .rodata*)

        *bf0_hal_rcc.*.o*     (.text* .rodata*)

        *.o*                (.text.SystemPowerOnModeGet)

        *bf0_hal_gpio.*.o*    (.text* .rodata*)

        *bf0_hal_hpaon.*.o*   (.text* .rodata*)
        *bf0_hal.*.o*         (.text.HAL_Init)
        *.o*                (.text.HAL_Delay_us)
        *.o*                (.text.HAL_Delay_us_)
        *.o*                (.text.HAL_Delay_us2_)
        *.o*                (.text.HAL_MspInit)
        *.o*                (.text.HAL_Delay)
        *bf0_hal_pinmux.*.o*  (.text* .rodata*)
        *bf0_pin_const.*.o*   (.text* .rodata*)
        *.o*                (.text.rt_memset)
        *rt_memclr*.*.o*      (.text*)

        *idle.*.o* (.text* .rodata*)

        . = ALIGN(4);
        __ramfunc_end = .;
    } >KERNEL_RAM AT>FLASH
    __ramfunc_load_start = LOADADDR(.ramfunc);
