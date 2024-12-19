/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOOT2_HELPER_WAIT_SSI_READY
#define _BOOT2_HELPER_WAIT_SSI_READY

// Wait for QMI direct mode to go idle. Clobbers r0/a0 (Arm/RISC-V)

#ifdef __riscv

wait_qmi_ready:
1:
    lw a0, QMI_DIRECT_CSR_OFFSET(a3)
    andi a0, a0, QMI_DIRECT_CSR_BUSY_BITS
    bnez a0, 1b

    ret

#else

wait_qmi_ready:
1:
    ldr r0, [r3, #QMI_DIRECT_CSR_OFFSET]
    lsrs r0, r0, #QMI_DIRECT_CSR_BUSY_LSB + 1
    bcs 1b

    bx lr

#endif

#endif
