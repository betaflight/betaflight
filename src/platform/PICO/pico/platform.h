/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/** \file platform.h
 *  \defgroup pico_platform pico_platform
 *
 * \brief Macros and definitions (and functions when included by non assembly code) for the RP2 family device / architecture
 * to provide a common abstraction over low level compiler / platform specifics
 *
 * This header may be included by assembly code
 */

#ifndef _PICO_PLATFORM_H
#define _PICO_PLATFORM_H

#ifndef _PICO_H
#error pico/platform.h should not be included directly; include pico.h instead
#endif

#include "pico/platform/compiler.h"
#include "pico/platform/sections.h"
#include "pico/platform/panic.h"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"
#ifdef __riscv
#include "hardware/regs/rvcsr.h"
#endif

// PICO_CONFIG: PICO_RP2350A, Whether the current board has an RP2350 in an A (30 GPIO) package, type=bool, default=Usually provided via board header, group=pico_platform
#if 0 // make tooling checks happy
#define PICO_RP2350A 0
#endif

// PICO_CONFIG: PICO_RP2350_A2_SUPPORTED, Whether to include any specific software support for RP2350 A2 revision, type=bool, default=1, advanced=true, group=pico_platform
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

// PICO_CONFIG: PICO_STACK_SIZE, Minimum amount of stack space reserved in the linker script for each core. See also PICO_CORE1_STACK_SIZE, min=0x100, default=0x800, advanced=true, group=pico_platform
#ifndef PICO_STACK_SIZE
#define PICO_STACK_SIZE _u(0x800)
#endif

// PICO_CONFIG: PICO_HEAP_SIZE, Minimum amount of heap space reserved by the linker script, min=0x100, default=0x800, advanced=true, group=pico_platform
#ifndef PICO_HEAP_SIZE
#define PICO_HEAP_SIZE _u(0x800)
#endif

// PICO_CONFIG: PICO_NO_RAM_VECTOR_TABLE, Enable/disable the RAM vector table, type=bool, default=0, advanced=true, group=pico_platform
#ifndef PICO_NO_RAM_VECTOR_TABLE
#define PICO_NO_RAM_VECTOR_TABLE 0
#endif

#ifndef PICO_RAM_VECTOR_TABLE_SIZE
#define PICO_RAM_VECTOR_TABLE_SIZE (VTABLE_FIRST_IRQ + NUM_IRQS)
#endif

// PICO_CONFIG: PICO_USE_STACK_GUARDS, Enable/disable stack guards, type=bool, default=0, advanced=true, group=pico_platform
#ifndef PICO_USE_STACK_GUARDS
#define PICO_USE_STACK_GUARDS 0
#endif

// PICO_CONFIG: PICO_CLKDIV_ROUND_NEAREST, True if floating point clock divisors should be rounded to the nearest possible clock divisor by default rather than rounding down, type=bool, default=1, group=pico_platform
#ifndef PICO_CLKDIV_ROUND_NEAREST
#define PICO_CLKDIV_ROUND_NEAREST 1
#endif

#ifndef __ASSEMBLER__

/*! \brief No-op function for the body of tight loops
 *  \ingroup pico_platform
 *
 * No-op function intended to be called by any tight hardware polling loop. Using this ubiquitously
 * makes it much easier to find tight loops, but also in the future \#ifdef-ed support for lockup
 * debugging might be added
 */
static __force_inline void tight_loop_contents(void) {}

/*! \brief Helper method to busy-wait for at least the given number of cycles
 *  \ingroup pico_platform
 *
 * This method is useful for introducing very short delays.
 *
 * This method busy-waits in a tight loop for the given number of system clock cycles. The total wait time is only accurate to within 2 cycles,
 * and this method uses a loop counter rather than a hardware timer, so the method will always take longer than expected if an
 * interrupt is handled on the calling core during the busy-wait; you can of course disable interrupts to prevent this.
 *
 * You can use \ref clock_get_hz(clk_sys) to determine the number of clock cycles per second if you want to convert an actual
 * time duration to a number of cycles.
 *
 * \param minimum_cycles the minimum number of system clock cycles to delay for
 */
static inline void busy_wait_at_least_cycles(uint32_t minimum_cycles) {
    pico_default_asm_volatile (
#ifdef __riscv
            // Note the range is halved on RISC-V due to signed comparison (no carry flag)
            ".option push\n"
            ".option norvc\n" // force 32 bit addi, so branch prediction guaranteed
            ".p2align 2\n"
            "1: \n"
            "addi %0, %0, -2 \n"
            "bgez %0, 1b\n"
            ".option pop"
#else
    "1: subs %0, #3\n"
    "bcs 1b\n"
#endif
    : "+r" (minimum_cycles) : : "cc", "memory"
    );
}

// PICO_CONFIG: PICO_NO_FPGA_CHECK, Remove the FPGA platform check for small code size reduction, type=bool, default=1, advanced=true, group=pico_runtime
#ifndef PICO_NO_FPGA_CHECK
#define PICO_NO_FPGA_CHECK 1
#endif

// PICO_CONFIG: PICO_NO_SIM_CHECK, Remove the SIM platform check for small code size reduction, type=bool, default=1, advanced=true, group=pico_runtime
#ifndef PICO_NO_SIM_CHECK
#define PICO_NO_SIM_CHECK 1
#endif

#if PICO_NO_FPGA_CHECK
static inline bool running_on_fpga(void) {return false;}
#else
bool running_on_fpga(void);
#endif
#if PICO_NO_SIM_CHECK
static inline bool running_in_sim(void) {return false;}
#else
bool running_in_sim(void);
#endif

/*! \brief Execute a breakpoint instruction
 *  \ingroup pico_platform
 */
static __force_inline void __breakpoint(void) {
#ifdef __riscv
    __asm ("ebreak");
#else
    pico_default_asm_volatile ("bkpt #0" : : : "memory");
#endif
}

/*! \brief Get the current core number
 *  \ingroup pico_platform
 *
 * \return The core number the call was made from
 */
__force_inline static uint get_core_num(void) {
    return (*(uint32_t *) (SIO_BASE + SIO_CPUID_OFFSET));
}

/*! \brief Get the current exception level on this core
 *  \ingroup pico_platform
 *
 * On Cortex-M this is the exception number defined in the architecture
 * reference, which is equal to VTABLE_FIRST_IRQ + irq num if inside an
 * interrupt handler. (VTABLE_FIRST_IRQ is defined in platform_defs.h).
 *
 * On Hazard3, this function returns VTABLE_FIRST_IRQ + irq num if inside of
 * an external IRQ handler (or a fault from such a handler), and 0 otherwise,
 * generally aligning with the Cortex-M values.
 *
 * \return the exception number if the CPU is handling an exception, or 0 otherwise
 */
static __force_inline uint __get_current_exception(void) {
#ifdef __riscv
    uint32_t meicontext;
    pico_default_asm_volatile (
            "csrr %0, %1\n"
    : "=r" (meicontext) : "i" (RVCSR_MEICONTEXT_OFFSET)
    );
    if (meicontext & RVCSR_MEICONTEXT_NOIRQ_BITS) {
        return 0;
    } else {
        return VTABLE_FIRST_IRQ + (
                (meicontext & RVCSR_MEICONTEXT_IRQ_BITS) >> RVCSR_MEICONTEXT_IRQ_LSB
        );
    }
#else
    uint exception;
    pico_default_asm_volatile (
        "mrs %0, ipsr\n"
        "uxtb %0, %0\n"
        : "=l" (exception)
    );
    return exception;
#endif
}

/*! \brief Return true if executing in the NonSecure state (Arm-only)
 *  \ingroup pico_platform
 *
 * \return True if currently executing in the NonSecure state on an Arm processor
 */
__force_inline static bool pico_processor_state_is_nonsecure(void) {
#ifndef __riscv
    // todo add a define to disable NS checking at all?
    // IDAU-Exempt addresses return S=1 when tested in the Secure state,
    // whereas executing a tt in the NonSecure state will always return S=0.
    uint32_t tt;
    pico_default_asm_volatile (
        "movs %0, #0\n"
        "tt %0, %0\n"
        : "=r" (tt) : : "cc"
    );
    return !(tt & (1u << 22));
#else
    // NonSecure is an Arm concept, there is nothing meaningful to return
    // here. Note it's not possible in general to detect whether you are
    // executing in U-mode as, for example, M-mode is classically
    // virtualisable in U-mode.
    return false;
#endif
}

#define host_safe_hw_ptr(x) ((uintptr_t)(x))
#define native_safe_hw_ptr(x) host_safe_hw_ptr(x)

/*! \brief Returns the RP2350 chip revision number
 *  \ingroup pico_platform
 * @return the RP2350 chip revision number (1 for B0/B1, 2 for B2)
 */
uint8_t rp2350_chip_version(void);

/*! \brief Returns the RP2040 chip revision number for compatibility
 *  \ingroup pico_platform
 * @return 2 RP2040 errata fixed in B2 are fixed in RP2350
 */
static inline uint8_t rp2040_chip_version(void) {
    return 2;
}

/*! \brief Returns the RP2040 rom version number
 *  \ingroup pico_platform
 * @return the RP2040 rom version number (1 for RP2040-B0, 2 for RP2040-B1, 3 for RP2040-B2)
 */
static inline uint8_t rp2040_rom_version(void) {
    GCC_Pragma("GCC diagnostic push")
    GCC_Pragma("GCC diagnostic ignored \"-Warray-bounds\"")
    return *(uint8_t*)0x13;
    GCC_Pragma("GCC diagnostic pop")
}

/*! \brief Multiply two integers using an assembly `MUL` instruction
 *  \ingroup pico_platform
 *
 * This multiplies a by b using multiply instruction using the ARM mul instruction regardless of values (the compiler
 * might otherwise choose to perform shifts/adds), i.e. this is a 1 cycle operation.
 *
 * \param a the first operand
 * \param b the second operand
 * \return a * b
 */
__force_inline static int32_t __mul_instruction(int32_t a, int32_t b) {
#ifdef __riscv
    __asm ("mul %0, %0, %1" : "+r" (a) : "r" (b) : );
#else
    pico_default_asm ("muls %0, %1"     : "+l" (a) : "l" (b) : "cc");
#endif
    return a;
}

/*! \brief multiply two integer values using the fastest method possible
 *  \ingroup pico_platform
 *
 * Efficiently multiplies value a by possibly constant value b.
 *
 * If b is known to be constant and not zero or a power of 2, then a mul instruction is used rather than gcc's default
 * which is often a slow combination of shifts and adds. If b is a power of 2 then a single shift is of course preferable
 * and will be used
 *
 * \param a the first operand
 * \param b the second operand
 * \return a * b
 */
#define __fast_mul(a, b) __builtin_choose_expr(__builtin_constant_p(b) && !__builtin_constant_p(a), \
    (__builtin_popcount(b) >= 2 ? __mul_instruction(a,b) : (a)*(b)), \
    (a)*(b))

#endif // __ASSEMBLER__

#endif

