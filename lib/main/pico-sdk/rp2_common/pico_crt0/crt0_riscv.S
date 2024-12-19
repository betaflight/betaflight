/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"

#include "hardware/regs/addressmap.h"
#include "hardware/regs/rvcsr.h"
#include "pico/binary_info/defs.h"
#include "boot/picobin.h"
#include "pico/bootrom_constants.h"

#ifdef NDEBUG
#ifndef COLLAPSE_IRQS
#define COLLAPSE_IRQS
#endif
#endif

#if !defined(PICO_CRT0_INCLUDE_PICOBIN_IMAGE_TYPE_ITEM)
#define PICO_CRT0_INCLUDE_PICOBIN_IMAGE_TYPE_ITEM 1
#endif

#ifndef PICO_CRT0_INCLUDE_PICOBIN_BLOCK
#define PICO_CRT0_INCLUDE_PICOBIN_BLOCK PICO_CRT0_INCLUDE_PICOBIN_IMAGE_TYPE_ITEM
#endif

#ifndef PICO_CRT0_INCLUDE_PICOBIN_END_BLOCK
#define PICO_CRT0_INCLUDE_PICOBIN_END_BLOCK (PICO_CRT0_INCLUDE_PICOBIN_BLOCK && !PICO_NO_FLASH)
#endif

// If vectors are in RAM, we put them in the .data section, so that they are
// preloaded by _reset_handler (assuming this is not a loaded-in-place
// binary).
#if PICO_NO_RAM_VECTOR_TABLE || PICO_NO_FLASH
.section .vectors, "ax"
#else
.section .data
#endif

.p2align 6
.global __vectors, __VECTOR_TABLE
__VECTOR_TABLE:
__vectors:

// Hardware vector table for standard RISC-V interrupts, indicated by `mtvec`.

.option push
.option norvc
.option norelax
j isr_riscv_machine_exception
.word 0
.word 0
j isr_riscv_machine_soft_irq
.word 0
.word 0
.word 0
j isr_riscv_machine_timer
.word 0
.word 0
.word 0
// j isr_riscv_machine_external_irq -> inlined below
.option pop

// External IRQ dispatch, inlined into the last vector table slot. Note if
// this code is modified, the VTABLE_FIRST_IRQ define in platform_defs.h also
// needs to be modified (it identifies the beginning of the soft vector table)
.global isr_riscv_machine_external_irq
.weak isr_riscv_machine_external_irq
isr_riscv_machine_external_irq:
    // Save caller saves and exception return state whilst IRQs are disabled.
    // We can't be preempted during this time, but if a higher-priority IRQ
    // arrives before we read meinext, that will be the one we enter.
    addi sp, sp, -80
    sw ra,  0(sp)
    sw t0,  4(sp)
    sw t1,  8(sp)
    sw t2, 12(sp)
    sw a0, 16(sp)
    sw a1, 20(sp)
    sw a2, 24(sp)
    sw a3, 28(sp)
    sw a4, 32(sp)
    sw a5, 36(sp)
    sw a6, 40(sp)
    sw a7, 44(sp)
    sw t3, 48(sp)
    sw t4, 52(sp)
    sw t5, 56(sp)
    sw t6, 60(sp)
    csrr a0, mepc
    csrr a1, mstatus
    sw a0, 64(sp)
    sw a1, 68(sp)
save_meicontext:
    // Make sure to set meicontext.clearts to clear+save mie.msie/mtie along
    // with ext IRQ context. We don't let these preempt ext IRQs because they
    // clear meicontext.mreteirq, which breaks __get_current_exception().
    csrrsi a2, RVCSR_MEICONTEXT_OFFSET, RVCSR_MEICONTEXT_CLEARTS_BITS
    sw a2, 72(sp)

get_first_irq:
    // Sample the current highest-priority active IRQ (left-shifted by 2) from
    // meinext, and write 1 to meinext.update to update meicontext with the
    // preemption priority and IRQ number of this IRQ
    csrrsi a0, RVCSR_MEINEXT_OFFSET, RVCSR_MEINEXT_UPDATE_BITS
    // MSB will be set if there is no active IRQ at the current priority level
    bltz a0, no_more_irqs
dispatch_irq:
    // Preemption priority was configured by meinext update, so enable preemption:
    csrsi mstatus, 0x8
    // <- from this point we can be preempted by a higher-priority interrupt.
    // meinext is pre-shifted by 2, so only an add is required to index table
    lui a1, %hi(__soft_vector_table)
    add a1, a1, a0
    lw a1, %lo(__soft_vector_table)(a1)
    jalr ra, a1
    // Disable IRQs on returning so we can sample the next IRQ
    csrci mstatus, 0x8
get_next_irq:
    // Get the next-highest-priority IRQ that is active at this level. If
    // there is such an IRQ, update meicontext with new preemption priority.
    csrrsi a0, RVCSR_MEINEXT_OFFSET, RVCSR_MEINEXT_UPDATE_BITS
    // MSB will be set if there is no active IRQ at the current priority level
    bgez a0, dispatch_irq

no_more_irqs:
    // Restore saved context and return from IRQ
    lw a0, 64(sp)
    lw a1, 68(sp)
    lw a2, 72(sp)
    csrw mepc, a0
    csrw mstatus, a1
    csrw RVCSR_MEICONTEXT_OFFSET, a2
    lw ra,  0(sp)
    lw t0,  4(sp)
    lw t1,  8(sp)
    lw t2, 12(sp)
    // skip a0 for now
    lw a1, 20(sp)
    lw a2, 24(sp)
    lw a3, 28(sp)
    lw a4, 32(sp)
    lw a5, 36(sp)
    lw a6, 40(sp)
    lw a7, 44(sp)
    lw t3, 48(sp)
    lw t4, 52(sp)
    lw t5, 56(sp)
    lw t6, 60(sp)
    // Before popping the stack frame, check if there is a new IRQ, and if so,
    // abandon the mret and take the IRQ. This avoids a worst-case (restore ->
    // mret -> enter -> save) latency. Note since we have already restored
    // meicontext, we will have to re-save it, to re-clear mtie/msie.
check_irq_before_exit:
    csrr a0, RVCSR_MEINEXT_OFFSET
    bgez a0, save_meicontext
    lw a0, 16(sp)
    addi sp, sp, 80
    mret

// Default software vector table for system interrupts, routed through
// mip.meip. Note this is assumed in e.g. hardware_irq to begin exactly 0x34
// words after the hardware vector table indicated by mtvec (defined above).
.p2align 4
.global __soft_vector_table
__soft_vector_table:
.word isr_irq0
.word isr_irq1
.word isr_irq2
.word isr_irq3
.word isr_irq4
.word isr_irq5
.word isr_irq6
.word isr_irq7
.word isr_irq8
.word isr_irq9
.word isr_irq10
.word isr_irq11
.word isr_irq12
.word isr_irq13
.word isr_irq14
.word isr_irq15
.word isr_irq16
.word isr_irq17
.word isr_irq18
.word isr_irq19
.word isr_irq20
.word isr_irq21
.word isr_irq22
.word isr_irq23
.word isr_irq24
.word isr_irq25
.word isr_irq26
.word isr_irq27
.word isr_irq28
.word isr_irq29
.word isr_irq30
.word isr_irq31
.word isr_irq32
.word isr_irq33
.word isr_irq34
.word isr_irq35
.word isr_irq36
.word isr_irq37
.word isr_irq38
.word isr_irq39
.word isr_irq40
.word isr_irq41
.word isr_irq42
.word isr_irq43
.word isr_irq44
.word isr_irq45
.word isr_irq46
.word isr_irq47
.word isr_irq48
.word isr_irq49
.word isr_irq50
.word isr_irq51

// all default trap handlers do nothing, and we can check for them being set to our
// default values by seeing if they point to somewhere between __defaults_isrs_start and __default_isrs_end
.global __default_isrs_start
__default_isrs_start:

// Declare a weak symbol for each ISR.
// By default, they will fall through to the undefined IRQ handler below (breakpoint),
// but can be overridden by C functions with correct name.

.macro decl_isr name
.weak \name
\name:
.endm

.macro decl_isr_bkpt name
.weak \name
\name:
    ebreak
.endm

// hardware_exception on RISC-V defines its own weak handler
#if !PICO_CRT0_NO_ISR_RISCV_MACHINE_EXCEPTION
// Breakpoint will just cause another exception and trash the exception
// state, since there is no double fault lockup on RISC-V. Instead, just
// sleep the core indefinitely, or until debugger connects.
decl_isr isr_riscv_machine_exception
#endif

// Note the mip.mtip is also available as SIO_IRQ_MTIMECMP, and this may be a
// better option, because it plays nicely with interrupt preemption.
decl_isr_bkpt isr_riscv_machine_timer
decl_isr_bkpt isr_riscv_machine_soft_irq

decl_isr isr_irq0
decl_isr isr_irq1
decl_isr isr_irq2
decl_isr isr_irq3
decl_isr isr_irq4
decl_isr isr_irq5
decl_isr isr_irq6
decl_isr isr_irq7
decl_isr isr_irq8
decl_isr isr_irq9
decl_isr isr_irq10
decl_isr isr_irq11
decl_isr isr_irq12
decl_isr isr_irq13
decl_isr isr_irq14
decl_isr isr_irq15
decl_isr isr_irq16
decl_isr isr_irq17
decl_isr isr_irq18
decl_isr isr_irq19
decl_isr isr_irq20
decl_isr isr_irq21
decl_isr isr_irq22
decl_isr isr_irq23
decl_isr isr_irq24
decl_isr isr_irq25
decl_isr isr_irq26
decl_isr isr_irq27
decl_isr isr_irq28
decl_isr isr_irq29
decl_isr isr_irq30
decl_isr isr_irq31
decl_isr isr_irq32
decl_isr isr_irq33
decl_isr isr_irq34
decl_isr isr_irq35
decl_isr isr_irq36
decl_isr isr_irq37
decl_isr isr_irq38
decl_isr isr_irq39
decl_isr isr_irq40
decl_isr isr_irq41
decl_isr isr_irq42
decl_isr isr_irq43
decl_isr isr_irq44
decl_isr isr_irq45
decl_isr isr_irq46
decl_isr isr_irq47
decl_isr isr_irq48
decl_isr isr_irq49
decl_isr isr_irq50
decl_isr isr_irq51
    // fall through

// All unhandled USER IRQs fall through to here. Note there is no way to get
// the "current exception" on RISC-V (as there is no such thing -- the
// hardware does not model the exception lifecycle like on Arm) so instead we
// just pass the IRQ number into the IRQ handler in a0.
.global __unhandled_user_irq
__unhandled_user_irq:
.global unhandled_user_irq_num_in_a0
unhandled_user_irq_num_in_a0:
    // The dispatch code will have left IRQ << 2 in a0 from its meinext read,
    // so just need to shift it back down
    srli a0, a0, 2
    ebreak

.global __default_isrs_end
__default_isrs_end:

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
.global _entry_point
_entry_point:

#if PICO_NO_FLASH
    // Go through our own reset handler. Same path for debugger entry and
    // bootloader entry.
    j _reset_handler
#else
    // Debugger tried to run code after loading, so SSI is in 03h-only mode.
    // Go back through bootrom + boot2 to properly initialise flash.
    j reenter_bootrom
#endif

// Reset handler:
// - initialises .data
// - clears .bss
// - calls runtime_init
// - calls main
// - calls exit (which should eventually hang the processor via _exit)

_reset_handler:
.option push
.option norelax
    la gp, __global_pointer$
.option pop
    la sp, __StackTop
    // Leave interrupts globally disabled for now, we will set that up later
    // in runtime_init_per_core_h3_irq_registers. Still worth installing the vector table:
    la a0, __vectors + 1
    csrw mtvec, a0

    // Only core 0 should run the C runtime startup code; core 1 is normally
    // sleeping in the bootrom at this point but check to be sure
    csrr a0, mhartid
    bnez a0, reenter_bootrom

#if PICO_RP2350 && PICO_EMBED_XIP_SETUP && !PICO_NO_FLASH
    // Execute boot2 on the core 0 stack (it also gets copied into BOOTRAM due
    // to inclusion in the data copy table below)
_copy_xip_setup:
    mv a2, sp
    addi sp, sp, -256
    mv a0, sp
    la a1, __boot2_entry_point
1:
    // Iterate forward, as sequential flash access is faster
    lw a3, (a1)
    sw a3, (a0)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a0, a2, 1b
_call_xip_setup:
    jalr sp
    addi sp, sp, 256
#endif

    // In a NO_FLASH binary, don't perform .data etc copy, since it's loaded
    // in-place by the SRAM load. Still need to clear .bss
#if !PICO_NO_FLASH
    la a4, data_cpy_table

    // assume there is at least one entry
1:
    lw a1, 0(a4)
    beqz a1, 2f
    lw a2, 4(a4)
    lw a3, 8(a4)
    addi a4, a4, 12
    jal data_cpy
    j 1b
2:
#endif

    // Zero out the BSS
    la a1, __bss_start__
    la a2, __bss_end__
    j bss_fill_test
bss_fill_loop:
    sw zero, (a1)
    addi a1, a1, 4
bss_fill_test:
    bne a1, a2, bss_fill_loop

platform_entry: // symbol for stack traces
    // Use `call` pseudo-instruction instead of a bare `jal` so that the
    // linker can use longer sequences if these are out of `jal` range. Will
    // still get relaxed to a `jal` if possible.
    // call runtime_init_per_core_h3_irq_registers (now called in runtime_init)
    call runtime_init
    call main
    call exit
    // exit should not return.  If it does, hang the core.
    // (fall thru into our hang _exit impl
1: // separate label because _exit can be moved out of branch range
    ebreak
    j 1b

data_cpy_loop:
    lw a0, (a1)
    sw a0, (a2)
    addi a1, a1, 4
    addi a2, a2, 4
data_cpy:
    bltu a2, a3, data_cpy_loop
    ret

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
runtime_init:
    ret

// ----------------------------------------------------------------------------
// If core 1 somehow gets into crt0, we need to catch it and send back to the
// sleep-and-launch code in the bootrom. Shouldn't happen (it should sleep in
// the ROM until given an entry point via the cross-core FIFOs) but it's good
// to be defensive.

// Enter through the shared reset handler: on core 1 this should quickly reach
// the wait-for-vector code.
reenter_bootrom:

#ifdef RASPBERRYPI_AMETHYST_FPGA
    // todo remove once 64k bootrom support is not required: vvvvvvvvvvvvvvvvvvvvv
    // Try jumping 32k higher and see if we get a fault :)
    li a0, BOOTROM_ENTRY_OFFSET + 32 * 1024
    la a1, 1f
    csrw mtvec, a1
    jr a0
    // Go here if we trapped:
.p2align 2
1:
#endif

    li a0, BOOTROM_ENTRY_OFFSET
    jr a0

// ----------------------------------------------------------------------------
// IRQ register clearing

// Enable interrupts globally, but disable all interrupt sources.
//
// This is put in the .text section as it is called again on core 1 launch. In
// COPY_TO_RAM binaries, the .reset section is always in flash, whereas .text
// is in SRAM, and we try to avoid running any code from flash after entering
// user code in a COPY_TO_RAM binary. Note because of this we don't call this
// function here in crt0 until after the flash-to-RAM copying is finished.

.section .text
.global runtime_init_per_core_h3_irq_registers
runtime_init_per_core_h3_irq_registers:
    // First clear all IRQ force array bits. Iterate over array registers 0
    // through 3 inclusive, allowing for up to 64 IRQs. Don't clear the
    // enable array as earlier (non-per-core) init stages may have already
    // set up IRQs.
    li a0, 3
1:
    csrw RVCSR_MEIFA_OFFSET, a0
    addi a0, a0, -1
    bgez a0, 1b
    // Setting the global external IRQ enable in mie prepares us to enable
    // IRQs one-by-one later. Also clear the soft IRQ and timer IRQ enables:
    li a0, RVCSR_MIE_MEIE_BITS
    csrw mie, a0
    // Set the global IRQ: we will now take any individual interrupt that is
    // pending && enabled
    csrsi mstatus, RVCSR_MSTATUS_MIE_BITS
    // Take this chance to clear mscratch, which is used to detect nested
    // exceptions in isr_riscv_machine_exception:
    csrw mscratch, zero
    ret

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

