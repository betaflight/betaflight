/*
 * STM32N6 FSBL stub Cortex-M55 startup for the GCC toolchain.
 *
 * The boot ROM loads the entire payload (incl. .text + .data) into AXISRAM2
 * at the address in the FSBL header, so only a .data LMA->VMA copy is
 * required (no flash-to-RAM .text copy).
 *
 * .data copy and .bss zero must run BEFORE SystemInit, otherwise SystemInit
 * writes to HAL globals (uwTick, SystemCoreClock) land in uninitialised
 * memory and HAL_GetTick() reads garbage on the first HAL timeout.
 *
 * The default exception handlers are stub infinite loops. SysTick_Handler
 * is overridden by stm32n6xx_it.c which calls HAL_IncTick.
 */

    .syntax unified
    .arch armv8.1-m.main
    .fpu softvfp
    .thumb

    .global g_pfnVectors
    .global Default_Handler

    /* Linker symbols used below */
    .word _sidata
    .word _sdata
    .word _edata
    .word _sbss
    .word _ebss

    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    ldr   r0, =_sstack
    msr   MSPLIM, r0
    ldr   r0, =_estack
    mov   sp, r0

    /* Copy .data from LMA (end of .text in ROM region) to VMA (RAM). */
    ldr   r0, =_sdata
    ldr   r1, =_edata
    ldr   r2, =_sidata
    movs  r3, #0
    b     LoopCopyDataInit
CopyDataInit:
    ldr   r4, [r2, r3]
    str   r4, [r0, r3]
    adds  r3, r3, #4
LoopCopyDataInit:
    adds  r4, r0, r3
    cmp   r4, r1
    bcc   CopyDataInit

    /* Zero-fill .bss. */
    ldr   r2, =_sbss
    ldr   r4, =_ebss
    movs  r3, #0
    b     LoopFillZerobss
FillZerobss:
    str   r3, [r2]
    adds  r2, r2, #4
LoopFillZerobss:
    cmp   r2, r4
    bcc   FillZerobss

    bl    SystemInit
    bl    main
LoopForever:
    b     LoopForever
    .size Reset_Handler, .-Reset_Handler

    .section .text.Default_Handler, "ax", %progbits
Default_Handler:
    b     Default_Handler
    .size Default_Handler, .-Default_Handler

    /* Cortex-M55 vector table — only the system vectors are populated.
     * IRQ handlers default to Default_Handler via .weak aliases. */
    .section .isr_vector, "a", %progbits
    .type g_pfnVectors, %object
g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word SecureFault_Handler
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word DebugMon_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler
    .size g_pfnVectors, .-g_pfnVectors

    .weak NMI_Handler
    .thumb_set NMI_Handler, Default_Handler
    .weak HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler
    .weak MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler
    .weak BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler
    .weak UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler
    .weak SecureFault_Handler
    .thumb_set SecureFault_Handler, Default_Handler
    .weak SVC_Handler
    .thumb_set SVC_Handler, Default_Handler
    .weak DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler
    .weak PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler
    .weak SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler
