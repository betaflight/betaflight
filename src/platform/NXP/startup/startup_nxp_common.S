/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The shared reset path. The family reset handler sets up the FlexRAM split and
 * branches here with interrupts masked and no usable stack; this never returns.
 */

/* CM7 TCM control: bit 0 enable, bit 1 read-modify-write, bit 2 retention. */
    .equ CM7_ITCMCR,             0xe000ef90
    .equ CM7_DTCMCR_OFFSET,      0x4
    .equ CM7_TCMCR_ENABLE,       (1 << 0) | (1 << 1) | (1 << 2)

    .syntax unified
    .cpu cortex-m7
    .fpu fpv5-d16
    .thumb

    .section .text.nxpResetTail, "ax", %progbits
    .align 2
    .thumb_func
    .globl  nxpResetTail
    .type   nxpResetTail, %function
nxpResetTail:
    /* Enable both TCMs on the core side before the stack moves into DTCM. */
    ldr     r0, =CM7_ITCMCR
    ldr     r1, [r0]
    orr     r1, r1, #CM7_TCMCR_ENABLE
    str     r1, [r0]
    ldr     r1, [r0, #CM7_DTCMCR_OFFSET]
    orr     r1, r1, #CM7_TCMCR_ENABLE
    str     r1, [r0, #CM7_DTCMCR_OFFSET]
    dsb
    isb

    ldr     r0, =__stack_top
    mov     sp, r0

    /* Full access to CP10 and CP11 turns the FPU on. */
    ldr     r0, =0xe000ed88             /* SCB->CPACR */
    ldr     r1, [r0]
    orr     r1, r1, #(0xf << 20)
    str     r1, [r0]
    dsb
    isb

    /* .itcm_text: flash -> ITCM (vector table, FAST_CODE, flash writer). */
    ldr     r0, =__itcm_load
    ldr     r1, =__itcm_start
    ldr     r2, =__itcm_end
    bl      copyRange

    /* Instructions were just written through the data path. */
    dsb
    isb

    /* Point VTOR at the vector table in ITCM. */
    ldr     r0, =0xe000ed08             /* SCB->VTOR */
    ldr     r1, =__vector_table
    str     r1, [r0]
    dsb
    isb

    /* .data: flash -> DTCM */
    ldr     r0, =__data_load
    ldr     r1, =__data_start
    ldr     r2, =__data_end
    bl      copyRange

    /* .dtcm_data: flash -> DTCM (DMA buffers with initial values) */
    ldr     r0, =__dtcm_data_load
    ldr     r1, =__dtcm_data_start
    ldr     r2, =__dtcm_data_end
    bl      copyRange

    /* .bss -> 0 */
    ldr     r0, =__bss_start
    ldr     r1, =__bss_end
    movs    r2, #0
1:  cmp     r0, r1
    bhs     2f
    str     r2, [r0], #4
    b       1b
2:

    /* Paint the whole stack for drivers/stack_check.c; this loop must not
       push anything. */
    ldr     r0, =__stack_limit
    ldr     r1, =__stack_top
    ldr     r2, =0xa5a5a5a5
1:  cmp     r0, r1
    bhs     2f
    str     r2, [r0], #4
    b       1b
2:

    bl      nxpClockInit
    bl      nxpMpuInit

    bl      __libc_init_array

    cpsie   i
    bl      main

3:  b       3b
    .size   nxpResetTail, . - nxpResetTail

/* Empty _init and _fini for __libc_init_array, as -nostartfiles drops crti.o. */
    .thumb_func
    .weak   _init
    .type   _init, %function
_init:
    bx      lr
    .size   _init, . - _init

    .thumb_func
    .weak   _fini
    .type   _fini, %function
_fini:
    bx      lr
    .size   _fini, . - _fini

/* Copy words from r0 to [r1, r2); ranges are word aligned and do not overlap. */
    .thumb_func
    .globl  copyRange
    .type   copyRange, %function
copyRange:
1:  cmp     r1, r2
    bhs     2f
    ldr     r3, [r0], #4
    str     r3, [r1], #4
    b       1b
2:  bx      lr
    .size   copyRange, . - copyRange
