/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"
#include "pico/asm_helper.S"

#include "pico/platform/cpu_regs.h"

#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"
#include "pico/binary_info/defs.h"
#include "boot/picobin.h"
#include "pico/bootrom.h"

#ifdef NDEBUG
#ifndef COLLAPSE_IRQS
#define COLLAPSE_IRQS
#endif
#endif

pico_default_asm_setup

.section .vectors, "ax"
.align 2

.global __vectors, __VECTOR_TABLE
__VECTOR_TABLE:
__vectors:
.word __StackTop
.word _reset_handler
.word isr_nmi
.word isr_hardfault
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_svcall
.word isr_invalid // Reserved, should never fire
.word isr_invalid // Reserved, should never fire
.word isr_pendsv
.word isr_systick
#if PICO_NO_STORED_VECTOR_TABLE && !PICO_NO_FLASH // note in no flash binary, we only have the single RAM vector table anyway
#if PICO_NO_RAM_VECTOR_TABLE
#error Can't specify PICO_NO_STORED_VECTOR_TABLE and PICO_NO_RAM_VECTOR_TABLE
#endif
// we don't include any IRQ vectors; we will initialize them during runtime_init in the RAM vector table
#else

.macro if_irq_word num func
.if \num < NUM_IRQS
.word \func
.endif
.endm

// we include a lot of these to allow for different number of IRQs.
// if_irq_word will only include IRQs that are valid, but we can't
// use a macro loop because isr_irqx MUST appear in the source
// as CMSIS rename exceptions #defines it to another value
if_irq_word 0 isr_irq0
if_irq_word 1 isr_irq1
if_irq_word 2 isr_irq2
if_irq_word 3 isr_irq3
if_irq_word 4 isr_irq4
if_irq_word 5 isr_irq5
if_irq_word 6 isr_irq6
if_irq_word 7 isr_irq7
if_irq_word 8 isr_irq8
if_irq_word 9 isr_irq9
if_irq_word 10 isr_irq10
if_irq_word 11 isr_irq11
if_irq_word 12 isr_irq12
if_irq_word 13 isr_irq13
if_irq_word 14 isr_irq14
if_irq_word 15 isr_irq15
if_irq_word 16 isr_irq16
if_irq_word 17 isr_irq17
if_irq_word 18 isr_irq18
if_irq_word 19 isr_irq19
if_irq_word 20 isr_irq20
if_irq_word 21 isr_irq21
if_irq_word 22 isr_irq22
if_irq_word 23 isr_irq23
if_irq_word 24 isr_irq24
if_irq_word 25 isr_irq25
if_irq_word 26 isr_irq26
if_irq_word 27 isr_irq27
if_irq_word 28 isr_irq28
if_irq_word 29 isr_irq29
if_irq_word 30 isr_irq30
if_irq_word 31 isr_irq31
if_irq_word 32 isr_irq32
if_irq_word 33 isr_irq33
if_irq_word 34 isr_irq34
if_irq_word 35 isr_irq35
if_irq_word 36 isr_irq36
if_irq_word 37 isr_irq37
if_irq_word 38 isr_irq38
if_irq_word 39 isr_irq39
if_irq_word 40 isr_irq40
if_irq_word 41 isr_irq41
if_irq_word 42 isr_irq42
if_irq_word 43 isr_irq43
if_irq_word 44 isr_irq44
if_irq_word 45 isr_irq45
if_irq_word 46 isr_irq46
if_irq_word 47 isr_irq47
if_irq_word 48 isr_irq48
if_irq_word 49 isr_irq49
if_irq_word 50 isr_irq50
if_irq_word 51 isr_irq51
if_irq_word 52 isr_irq52
if_irq_word 53 isr_irq53
if_irq_word 54 isr_irq54
if_irq_word 55 isr_irq55
if_irq_word 56 isr_irq56
if_irq_word 57 isr_irq57
if_irq_word 58 isr_irq58
if_irq_word 59 isr_irq59
if_irq_word 60 isr_irq60
if_irq_word 61 isr_irq61
if_irq_word 62 isr_irq62
if_irq_word 63 isr_irq63
if_irq_word 64 isr_irq64
if_irq_word 65 isr_irq65
if_irq_word 66 isr_irq66
if_irq_word 67 isr_irq67
if_irq_word 68 isr_irq68
if_irq_word 69 isr_irq69
if_irq_word 70 isr_irq70
if_irq_word 71 isr_irq71
if_irq_word 72 isr_irq72
if_irq_word 73 isr_irq73
if_irq_word 74 isr_irq74
if_irq_word 75 isr_irq75
if_irq_word 76 isr_irq76
if_irq_word 77 isr_irq77
if_irq_word 78 isr_irq78
if_irq_word 79 isr_irq79
#if NUM_IRQS > 80
#error more IRQ entries required
#endif
#endif

// all default exception handlers do nothing, and we can check for them being set to our
// default values by seeing if they point to somewhere between __defaults_isrs_start and __default_isrs_end
.global __default_isrs_start
__default_isrs_start:

// Declare a weak symbol for each ISR.
// By default, they will fall through to the undefined IRQ handler below (breakpoint),
// but can be overridden by C functions with correct name.

.macro decl_isr_bkpt name
.weak \name
.type \name,%function
.thumb_func
\name:
    bkpt #0
.endm

// these are separated out for clarity
decl_isr_bkpt isr_invalid
decl_isr_bkpt isr_nmi
decl_isr_bkpt isr_hardfault
decl_isr_bkpt isr_svcall
decl_isr_bkpt isr_pendsv
decl_isr_bkpt isr_systick

.global __default_isrs_end
__default_isrs_end:

.altmacro
.macro decl_isr name
#if !PICO_NO_STORED_VECTOR_TABLE | PICO_NO_FLASH
// We declare a weak label, so user can override
.weak \name
#else
// We declare a strong label, so user can't override (their version would not automatically be used)
#endif
.type \name,%function
.thumb_func
\name:
.endm

.macro if_irq_decl num func
.if \num < NUM_IRQS
decl_isr \func
.endif
.endm

if_irq_decl 0 isr_irq0
if_irq_decl 1 isr_irq1
if_irq_decl 2 isr_irq2
if_irq_decl 3 isr_irq3
if_irq_decl 4 isr_irq4
if_irq_decl 5 isr_irq5
if_irq_decl 6 isr_irq6
if_irq_decl 7 isr_irq7
if_irq_decl 8 isr_irq8
if_irq_decl 9 isr_irq9
if_irq_decl 10 isr_irq10
if_irq_decl 11 isr_irq11
if_irq_decl 12 isr_irq12
if_irq_decl 13 isr_irq13
if_irq_decl 14 isr_irq14
if_irq_decl 15 isr_irq15
if_irq_decl 16 isr_irq16
if_irq_decl 17 isr_irq17
if_irq_decl 18 isr_irq18
if_irq_decl 19 isr_irq19
if_irq_decl 20 isr_irq20
if_irq_decl 21 isr_irq21
if_irq_decl 22 isr_irq22
if_irq_decl 23 isr_irq23
if_irq_decl 24 isr_irq24
if_irq_decl 25 isr_irq25
if_irq_decl 26 isr_irq26
if_irq_decl 27 isr_irq27
if_irq_decl 28 isr_irq28
if_irq_decl 29 isr_irq29
if_irq_decl 30 isr_irq30
if_irq_decl 31 isr_irq31
if_irq_decl 32 isr_irq32
if_irq_decl 33 isr_irq33
if_irq_decl 34 isr_irq34
if_irq_decl 35 isr_irq35
if_irq_decl 36 isr_irq36
if_irq_decl 37 isr_irq37
if_irq_decl 38 isr_irq38
if_irq_decl 39 isr_irq39
if_irq_decl 40 isr_irq40
if_irq_decl 41 isr_irq41
if_irq_decl 42 isr_irq42
if_irq_decl 43 isr_irq43
if_irq_decl 44 isr_irq44
if_irq_decl 45 isr_irq45
if_irq_decl 46 isr_irq46
if_irq_decl 47 isr_irq47
if_irq_decl 48 isr_irq48
if_irq_decl 49 isr_irq49
if_irq_decl 50 isr_irq50
if_irq_decl 51 isr_irq51
if_irq_decl 52 isr_irq52
if_irq_decl 53 isr_irq53
if_irq_decl 54 isr_irq54
if_irq_decl 55 isr_irq55
if_irq_decl 56 isr_irq56
if_irq_decl 57 isr_irq57
if_irq_decl 58 isr_irq58
if_irq_decl 59 isr_irq59
if_irq_decl 60 isr_irq60
if_irq_decl 61 isr_irq61
if_irq_decl 62 isr_irq62
if_irq_decl 63 isr_irq63
if_irq_decl 64 isr_irq64
if_irq_decl 65 isr_irq65
if_irq_decl 66 isr_irq66
if_irq_decl 67 isr_irq67
if_irq_decl 68 isr_irq68
if_irq_decl 69 isr_irq69
if_irq_decl 70 isr_irq70
if_irq_decl 71 isr_irq71
if_irq_decl 72 isr_irq72
if_irq_decl 73 isr_irq73
if_irq_decl 74 isr_irq74
if_irq_decl 75 isr_irq75
if_irq_decl 76 isr_irq76
if_irq_decl 77 isr_irq77
if_irq_decl 78 isr_irq78
if_irq_decl 79 isr_irq79
#if NUM_IRQS > 80
#error more IRQ entries required
#endif

// All unhandled USER IRQs fall through to here
.global __unhandled_user_irq
.thumb_func
__unhandled_user_irq:
    mrs  r0, ipsr
    subs r0, #16
.global unhandled_user_irq_num_in_r0
unhandled_user_irq_num_in_r0:
    bkpt #0

// ----------------------------------------------------------------------------

.section .binary_info_header, "a"

// Header must be in first 256 bytes of main image (i.e. excluding flash boot2).
// For flash builds we put it immediately after vector table; for NO_FLASH the
// vectors are at a +0x100 offset because the bootrom enters RAM images directly
// at their lowest address, so we put the header in the VTOR alignment hole.

#if !PICO_NO_BINARY_INFO
binary_info_header:
.word BINARY_INFO_MARKER_START
.word __binary_info_start
.word __binary_info_end
.word data_cpy_table // we may need to decode pointers that are in RAM at runtime.
.word BINARY_INFO_MARKER_END
#endif

#include "embedded_start_block.inc.S"

// ----------------------------------------------------------------------------

.section .reset, "ax"

// On flash builds, the vector table comes first in the image (conventional).
// On NO_FLASH builds, the reset handler section comes first, as the entry
// point is at offset 0 (fixed due to bootrom), and VTOR is highly-aligned.
// Image is entered in various ways:
//
// - NO_FLASH builds are entered from beginning by UF2 bootloader
//
// - Flash builds vector through the table into _reset_handler from boot2
//
// - Either type can be entered via _entry_point by the debugger, and flash builds
//   must then be sent back round the boot sequence to properly initialise flash

// ELF entry point:
.type _entry_point,%function
.thumb_func
.global _entry_point
_entry_point:

#if PICO_NO_FLASH
    // on the NO_FLASH case, we do not do a rest thru bootrom below, so the RCP may or may not have been initialized:
    //
    // in the normal (e.g. UF2 download etc. case) we will have passed thru bootrom initialization, but if
    // a NO_FLASH binary is loaded by the debugger, and run directly after a reset, then we won't have.
    //
    // we must therefore initialize the RCP if it hasn't already been

#if HAS_REDUNDANCY_COPROCESSOR
    // just enable the RCP which is fine if it already was (we assume no other co-processors are enabled at this point to save space)
    ldr r0, = PPB_BASE + M33_CPACR_OFFSET
    movs r1, #ARM_CPU_PREFIXED(CPACR_CP7_BITS)
    str r1, [r0]
    // only initialize canary seeds if they haven't been (as to do so twice is a fault)
    mrc p7, #1, apsr_nzcv, c0, c0, #0
    bmi 1f
    // i dont think it much matters what we initialized to, as to have gotten here we must have not
    // gone thru the bootrom (which a secure boot would have)
    mcrr p7, #8, r0, r0, c0
    mcrr p7, #8, r0, r0, c1
    sev
1:
#endif
    ldr r0, =__vectors
    // Vector through our own table (SP, VTOR will not have been set up at
    // this point). Same path for debugger entry and bootloader entry.
#else
    // Debugger tried to run code after loading, so SSI is in 03h-only mode.
    // Go back through bootrom + boot2 to properly initialise flash.
    ldr r0, =BOOTROM_VTABLE_OFFSET
#endif

_enter_vtable_in_r0:
    ldr r1, =(PPB_BASE + ARM_CPU_PREFIXED(VTOR_OFFSET))
    str r0, [r1]
    ldmia r0!, {r1, r2}
    msr msp, r1
    bx r2

// Reset handler:
// - initialises .data
// - clears .bss
// - calls runtime_init
// - calls main
// - calls exit (which should eventually hang the processor via _exit)

.type _reset_handler,%function
.thumb_func
_reset_handler:
    // Only core 0 should run the C runtime startup code; core 1 is normally
    // sleeping in the bootrom at this point but check to be sure (e.g. if
    // debugger put core 1 at the ELF entry point for some reason)
    ldr r0, =(SIO_BASE + SIO_CPUID_OFFSET)
    ldr r0, [r0]
#if __ARM_ARCH_6M__
    cmp r0, #0
    beq 1f
#else
    cbz r0, 1f
#endif
hold_non_core0_in_bootrom:
    // Send back to the ROM to wait for core 0 to launch it.
    ldr r0, =BOOTROM_VTABLE_OFFSET
    b _enter_vtable_in_r0
1:

#if !PICO_RP2040 && PICO_EMBED_XIP_SETUP && !PICO_NO_FLASH
    // Execute boot2 on the core 0 stack (it also gets copied into BOOTRAM due
    // to inclusion in the data copy table below). Note the reference
    // to __boot2_entry_point here is what prevents the .boot2 section from
    // being garbage-collected.
_copy_xip_setup:
    ldr r1, =__boot2_entry_point
    mov r3, sp
    add sp, #-256
    mov r2, sp
    bl data_cpy
_call_xip_setup:
    mov r0, sp
    adds r0, #1
    blx r0
    add sp, #256
#endif

    // In a NO_FLASH binary, don't perform .data etc copy, since it's loaded
    // in-place by the SRAM load. Still need to clear .bss
#if !PICO_NO_FLASH
    adr r4, data_cpy_table

    // assume there is at least one entry
1:
    ldmia r4!, {r1-r3}
    cmp r1, #0
    beq 2f
    bl data_cpy
    b 1b
2:
#endif

    // Zero out the BSS
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__
    movs r0, #0
    b bss_fill_test
bss_fill_loop:
    stm r1!, {r0}
bss_fill_test:
    cmp r1, r2
    bne bss_fill_loop

platform_entry: // symbol for stack traces
    // Use 32-bit jumps, in case these symbols are moved out of branch range
    // (e.g. if main is in SRAM and crt0 in flash)
#if !__ARM_ARCH_6M__
    // Make sure stack limit is 0 - the user can set it themselves
    //  todo probably worth adding to the EXE_DEF in the future
    movs r0, #0
    msr msplim, r0
#endif
    ldr r1, =runtime_init
    blx r1
    ldr r1, =main
    blx r1
    ldr r1, =exit
    blx r1
    // exit should not return.  If it does, hang the core.
1: // separate label because _exit can be moved out of branch range
    bkpt #0
    b 1b


#if !PICO_NO_FLASH
data_cpy_loop:
    ldm r1!, {r0}
    stm r2!, {r0}
data_cpy:
    cmp r2, r3
    blo data_cpy_loop
    bx lr
#endif

// Note the data copy table is still included for NO_FLASH builds, even though
// we skip the copy, because it is listed in binary info

.align 2
data_cpy_table:
#if PICO_RP2350 && PICO_EMBED_XIP_SETUP && !PICO_NO_FLASH
.word __boot2_start__
.word BOOTRAM_BASE
.word BOOTRAM_BASE + 256
#endif

#if PICO_COPY_TO_RAM
.word __ram_text_source__
.word __ram_text_start__
.word __ram_text_end__
#endif
.word __etext
.word __data_start__
.word __data_end__

.word __scratch_x_source__
.word __scratch_x_start__
.word __scratch_x_end__

.word __scratch_y_source__
.word __scratch_y_start__
.word __scratch_y_end__

.word 0 // null terminator

// ----------------------------------------------------------------------------
// Provide safe defaults for _exit and runtime_init
// Full implementations usually provided by platform.c

.weak runtime_init
.type runtime_init,%function
.thumb_func
runtime_init:
    bx lr

// ----------------------------------------------------------------------------
// Stack/heap dummies to set size

// Prior to SDK 1.5.1 these were `.section .stack` without the `, "a"`... Clang linker gives a warning about this,
// however setting it explicitly to `, "a"` makes GCC *now* discard the section unless it is also KEEP. This
// seems like very surprising behavior!
//
// Strictly the most correct thing to do (as .stack and .heap are unreferenced) is to mark them as "a", and also KEEP, which
// works correctly for both GCC and Clang, however doing so may break anyone who already has custom linker scripts without
// the KEEP. Therefore we will only add the "a" on Clang, but will also use KEEP to our own linker scripts.

.macro spacer_section name
#if PICO_ASSEMBLER_IS_CLANG
.section \name, "a"
#else
.section \name
#endif
.endm

spacer_section .stack
// align to allow for memory protection (although this alignment is pretty much ignored by linker script)
.p2align 5
    .equ StackSize, PICO_STACK_SIZE
.space StackSize

spacer_section .heap
.p2align 2
    .equ HeapSize, PICO_HEAP_SIZE
.space HeapSize

#include "embedded_end_block.inc.S"
