/*
 * SPDX-FileCopyrightText: 2026 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

    .pbl_kobj_init : {
        . = ALIGN(4);
        __pbl_kobj_init_start__ = .;
        KEEP(*(.pbl_kobj_init))
        __pbl_kobj_init_end__ = .;
    } >FLASH
