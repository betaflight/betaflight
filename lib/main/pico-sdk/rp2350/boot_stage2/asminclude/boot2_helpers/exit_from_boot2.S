/*
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOOT2_HELPER_EXIT_FROM_BOOT2
#define _BOOT2_HELPER_EXIT_FROM_BOOT2

// Always return through saved ra/lr. On RP2350, boot2 is not responsible for entering
// the flash image. It's just a relocatable function that sets up QMI and flash
// for the best possible XIP.

#ifdef __riscv
    jr t0
#else
    pop {pc}
#endif

#endif
