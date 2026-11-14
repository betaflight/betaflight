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
 * NavQ95 (i.MX 95 Cortex-M7) startup: vector table and reset handler.
 *
 * The reset handler sets the stack and VTOR, copies .data and .ocram_text,
 * zeroes .bss, then calls SystemInit() and main().
 */

    .syntax unified
    .arch   armv7e-m
    .thumb

/* External IRQs up to LPUART8 (70 entries). */
    .equ    NAVQ95_NUM_EXT_IRQ, 70

    .section .isr_vector, "a"
    .align  2
    .globl  __isr_vector
__isr_vector:
    .long   __StackTop
    .long   Reset_Handler
    .long   NMI_Handler
    .long   HardFault_Handler
    .long   MemManage_Handler
    .long   BusFault_Handler
    .long   UsageFault_Handler
    .long   0
    .long   0
    .long   0
    .long   0
    .long   SVC_Handler
    .long   DebugMon_Handler
    .long   0
    .long   PendSV_Handler
    .long   SysTick_Handler

    /* External IRQs */
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   LPUART2_IRQHandler          /* 20: LPUART2, console */
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   UART5_IRQHandler            /* 66: LPUART5 */
    .long   Default_Handler
    .long   UART7_IRQHandler            /* 68: LPUART7 */
    .long   Default_Handler
    .size   __isr_vector, . - __isr_vector

    .section .text.Reset_Handler
    .thumb_func
    .align  2
    .globl  Reset_Handler
    .type   Reset_Handler, %function
Reset_Handler:
    cpsid   i

    /* The boot loader may not load SP from this vector table. */
    ldr     r0, =__StackTop
    mov     sp, r0

    ldr     r0, =__isr_vector
    ldr     r1, =0xE000ED08             /* SCB->VTOR */
    str     r0, [r1]
    dsb
    isb

    ldr     r0, =_data_load
    ldr     r1, =_data_start
    ldr     r2, =_data_end
    b       .Lcopy_check
.Lcopy_loop:
    ldr     r3, [r0], #4
    str     r3, [r1], #4
.Lcopy_check:
    cmp     r1, r2
    blt     .Lcopy_loop

    ldr     r0, =_ocram_text_load
    ldr     r1, =_ocram_text_start
    ldr     r2, =_ocram_text_end
    b       .Locram_copy_check
.Locram_copy_loop:
    ldr     r3, [r0], #4
    str     r3, [r1], #4
.Locram_copy_check:
    cmp     r1, r2
    blt     .Locram_copy_loop

    ldr     r0, =_bss_start
    ldr     r1, =_bss_end
    movs    r2, #0
    b       .Lzero_check
.Lzero_loop:
    str     r2, [r0], #4
.Lzero_check:
    cmp     r0, r1
    blt     .Lzero_loop

    bl      SystemInit
    cpsie   i
    bl      main

.Lhang:
    b       .Lhang
    .size   Reset_Handler, . - Reset_Handler

    .section .text.Default_Handler, "ax"
    .thumb_func
    .align  2
    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    b       Default_Handler
    .size   Default_Handler, . - Default_Handler

    .weak   NMI_Handler
    .thumb_set NMI_Handler, Default_Handler
    .weak   HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler
    .weak   MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler
    .weak   BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler
    .weak   UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler
    .weak   SVC_Handler
    .thumb_set SVC_Handler, Default_Handler
    .weak   DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler
    .weak   PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler
    .weak   SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler
    .weak   LPUART2_IRQHandler
    .thumb_set LPUART2_IRQHandler, Default_Handler
    .weak   UART5_IRQHandler
    .thumb_set UART5_IRQHandler, Default_Handler
    .weak   UART7_IRQHandler
    .thumb_set UART7_IRQHandler, Default_Handler

    .end
