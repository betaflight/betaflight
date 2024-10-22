/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOOT2_HELPER_READ_FLASH_SREG
#define _BOOT2_HELPER_READ_FLASH_SREG

#include "boot2_helpers/wait_qmi_ready.S"

// Pass status read cmd into r0/a0.
// Returns status value in r0/a0.

.global read_flash_sreg
.type read_flash_sreg,%function
#ifdef __riscv
read_flash_sreg:
    // wait_qmi_ready does not clobber t1, so use this rather than stack.
    mv t1, ra
    sw a0, QMI_DIRECT_TX_OFFSET(a3)
    // Dummy byte:
    sw a0, QMI_DIRECT_TX_OFFSET(a3)

    jal wait_qmi_ready
    // Discard first byte, return second
    lw a0, QMI_DIRECT_RX_OFFSET(a3)
    lw a0, QMI_DIRECT_RX_OFFSET(a3)
    jr t1

#else

.thumb_func
read_flash_sreg:
    push {lr}
    str r0, [r3, #QMI_DIRECT_TX_OFFSET]
    // Dummy byte:
    str r0, [r3, #QMI_DIRECT_TX_OFFSET]
    
    bl wait_qmi_ready
    // Discard first byte, return second
    ldr r0, [r3, #QMI_DIRECT_RX_OFFSET]
    ldr r0, [r3, #QMI_DIRECT_RX_OFFSET]

    pop {pc}

#endif

#endif
