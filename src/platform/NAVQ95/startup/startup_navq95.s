/*
 * startup_navq95.s — startup for the NavQ95 (NXP i.MX 95, Cortex-M7).
 *
 * Vector table (16 core exceptions + external IRQs up to the LPUART2 console
 * IRQ) and Reset_Handler. External IRQ numbers are the raw NVIC external index;
 * LPUART2 = external IRQ 20 (vector table index 36). External vectors default
 * to Default_Handler (weak) except LPUART2_IRQHandler (provided in C).
 *
 * Reset_Handler copies .data (flash->SRAM), zeroes .bss, sets VTOR to this
 * table, then calls SystemInit and main(). The table is placed by the linker
 * at app_base + 0x800 (after the MCUboot header).
 */

    .syntax unified
    .arch   armv7e-m
    .thumb

/* Emit external IRQ slots up to and past LPUART2 (ext 20). Unused slots point
 * at Default_Handler and are harmless; the CPU only dispatches raised IRQs. */
    .equ    NAVQ95_NUM_EXT_IRQ, 24

/* ── Vector table ───────────────────────────────────────────────────────── */
    .section .isr_vector, "a"
    .align  2
    .globl  __isr_vector
__isr_vector:
    /* 16 core/system exception vectors */
    .long   __StackTop                  /* 0x00: Initial SP (top of SRAM)   */
    .long   Reset_Handler               /* 0x04: Reset                      */
    .long   NMI_Handler                 /* 0x08: NMI                        */
    .long   HardFault_Handler           /* 0x0C: HardFault                  */
    .long   MemManage_Handler           /* 0x10: MemManage                  */
    .long   BusFault_Handler            /* 0x14: BusFault                   */
    .long   UsageFault_Handler          /* 0x18: UsageFault                 */
    .long   0                           /* 0x1C: reserved                   */
    .long   0                           /* 0x20: reserved                   */
    .long   0                           /* 0x24: reserved                   */
    .long   0                           /* 0x28: reserved                   */
    .long   SVC_Handler                 /* 0x2C: SVCall                     */
    .long   DebugMon_Handler            /* 0x30: DebugMonitor               */
    .long   0                           /* 0x34: reserved                   */
    .long   PendSV_Handler              /* 0x38: PendSV                     */
    .long   SysTick_Handler             /* 0x3C: SysTick                    */

    /* External IRQ vectors 0 .. (NAVQ95_NUM_EXT_IRQ-1).
     * Emit Default_Handler for all, then OVERRIDE ext #20 (LPUART2) below by
     * laying the table out explicitly. */
    .long   Default_Handler             /* ext 0  */
    .long   Default_Handler             /* ext 1  */
    .long   Default_Handler             /* ext 2  */
    .long   Default_Handler             /* ext 3  */
    .long   Default_Handler             /* ext 4  */
    .long   Default_Handler             /* ext 5  */
    .long   Default_Handler             /* ext 6  */
    .long   Default_Handler             /* ext 7  */
    .long   Default_Handler             /* ext 8  */
    .long   Default_Handler             /* ext 9  */
    .long   Default_Handler             /* ext 10 */
    .long   Default_Handler             /* ext 11 */
    .long   Default_Handler             /* ext 12 */
    .long   Default_Handler             /* ext 13 */
    .long   Default_Handler             /* ext 14 */
    .long   Default_Handler             /* ext 15 */
    .long   Default_Handler             /* ext 16 */
    .long   Default_Handler             /* ext 17 */
    .long   Default_Handler             /* ext 18 */
    .long   Default_Handler             /* ext 19 (LPUART1) */
    .long   LPUART2_IRQHandler          /* ext 20 (LPUART2 — console)       */
    .long   Default_Handler             /* ext 21 */
    .long   Default_Handler             /* ext 22 */
    .long   Default_Handler             /* ext 23 */
    .size   __isr_vector, . - __isr_vector

/* ── Reset handler ──────────────────────────────────────────────────────── */
    .section .text.Reset_Handler
    .thumb_func
    .align  2
    .globl  Reset_Handler
    .type   Reset_Handler, %function
Reset_Handler:
    cpsid   i

    /* Set SP explicitly (boot ROM/MCUboot may not load it from the vector table) */
    ldr     r0, =__StackTop
    mov     sp, r0

    /* Point VTOR at our vector table so the NVIC dispatches to our handlers,
     * regardless of what MCUboot/the SM left in VTOR. SCB->VTOR = 0xE000ED08. */
    ldr     r0, =__isr_vector
    ldr     r1, =0xE000ED08             /* SCB->VTOR */
    str     r0, [r1]
    dsb
    isb

    /* Copy .data: flash (LMA) -> SRAM (VMA) */
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

    /* Zero .bss */
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

/* ── Default handler ────────────────────────────────────────────────────── */
    .section .text.Default_Handler, "ax"
    .thumb_func
    .align  2
    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    b       Default_Handler
    .size   Default_Handler, . - Default_Handler

/* ── Weak aliases: core-exception + LPUART2 handlers default to Default_Handler
 *    unless a strong symbol (e.g. LPUART2_IRQHandler in platform_stubs.c) is
 *    provided. ──────────────────────────────────────────────────────────── */
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

    .end
