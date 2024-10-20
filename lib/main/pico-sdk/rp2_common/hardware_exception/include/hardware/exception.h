/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_EXCEPTION_H
#define _HARDWARE_EXCEPTION_H

#include "pico.h"
#include "hardware/address_mapped.h"

/** \file exception.h
 *  \defgroup hardware_exception hardware_exception
 *
 * \brief Methods for setting processor exception handlers
 *
 * Exceptions are identified by a \ref exception_number which is a number from -15 to -1; these are the numbers relative to
 * the index of the first IRQ vector in the vector table. (i.e. vector table index is exception_num plus 16)
 *
 * There is one set of exception handlers per core, so the exception handlers for each core as set by these methods are independent.
 *
 * \note That all exception APIs affect the executing core only (i.e. the core calling the function).
 */

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_EXCEPTION, Enable/disable assertions in the hardware_exception module, type=bool, default=0, group=hardware_exception
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_EXCEPTION
#ifdef PARAM_ASSERTIONS_ENABLED_EXCEPTION // backwards compatibility with SDK < 2.0.0
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_EXCEPTION PARAM_ASSERTIONS_ENABLED_EXCEPTION
#else
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_EXCEPTION 0
#endif
#endif
#ifdef __cplusplus
extern "C" {
#endif

/*! \brief  Exception number definitions
 *
 * On Arm these are vector table indices:
 *
 * Name                 | Value | Exception
 * ---------------------|-------|-----------------------
 * NMI_EXCEPTION        | 2     | Non Maskable Interrupt
 * HARDFAULT_EXCEPTION  | 3     | HardFault
 * SVCALL_EXCEPTION     | 11    | SV Call
 * PENDSV_EXCEPTION     | 14    | Pend SV
 * SYSTICK_EXCEPTION    | 15    | System Tick
 *
 * \if rp2350_specific
 * On RISC-V these are exception cause numbers:
 *
 * Name                    | Value | Exception
 * ------------------------|-------|-----------------------------
 * INSTR_ALIGN_EXCEPTION   | 0     | Instruction fetch misaligned
 * INSTR_FAULT_EXCEPTION   | 1     | Instruction fetch bus fault
 * INSTR_ILLEGAL_EXCEPTION | 2     | Invalid or illegal instruction
 * EBREAK_EXCEPTION        | 3     | ebreak was not caught by an ex
 * LOAD_ALIGN_EXCEPTION    | 4     | Load address not naturally ali
 * LOAD_FAULT_EXCEPTION    | 5     | Load bus fault
 * STORE_ALIGN_EXCEPTION   | 6     | Store or AMO address not natur
 * STORE_FAULT_EXCEPTION   | 7     | Store or AMO bus fault
 * ECALL_UMODE_EXCEPTION   | 8     | ecall was executed in U-mode
 * ECALL_SMODE_EXCEPTION   | 9     | ecall was executed in S-mode
 * ECALL_MMODE_EXCEPTION   | 11    | ecall was executed in M-mode
 * \endif
 *
 * \ingroup hardware_exception
 */
#ifdef __riscv
enum exception_number {
    // Assigned to non-IRQ xcause values
    MIN_EXCEPTION_NUM       = 0,
    INSTR_ALIGN_EXCEPTION   = 0,  ///< Instruction fetch misaligned (never fires if C/Zca is present)
    INSTR_FAULT_EXCEPTION   = 1,  ///< Instruction fetch bus fault
    INSTR_ILLEGAL_EXCEPTION = 2,  ///< Invalid or illegal instruction
    EBREAK_EXCEPTION        = 3,  ///< ebreak was not caught by an external debugger
    LOAD_ALIGN_EXCEPTION    = 4,  ///< Load address not naturally aligned
    LOAD_FAULT_EXCEPTION    = 5,  ///< Load bus fault
    STORE_ALIGN_EXCEPTION   = 6,  ///< Store or AMO address not naturally aligned
    STORE_FAULT_EXCEPTION   = 7,  ///< Store or AMO bus fault
    ECALL_UMODE_EXCEPTION   = 8,  ///< ecall was executed in U-mode
    ECALL_SMODE_EXCEPTION   = 9,  ///< ecall was executed in S-mode
    ECALL_MMODE_EXCEPTION   = 11, ///< ecall was executed in M-mode
    MAX_EXCEPTION_NUM       = 11
};
#else
enum exception_number {
    // Assigned to VTOR indices
    MIN_EXCEPTION_NUM = 2,
    NMI_EXCEPTION = 2,        ///< Non Maskable Interrupt
    HARDFAULT_EXCEPTION = 3,  ///< HardFault Interrupt
    SVCALL_EXCEPTION = 11,    ///< SV Call Interrupt
    PENDSV_EXCEPTION = 14,    ///< Pend SV Interrupt
    SYSTICK_EXCEPTION = 15,   ///< System Tick Interrupt
    MAX_EXCEPTION_NUM = 15
};
#endif

#define PICO_LOWEST_EXCEPTION_PRIORITY 0xff
#define PICO_HIGHEST_EXCEPTION_PRIORITY 0x00


/*! \brief Exception handler function type
 *  \ingroup hardware_exception
 *
 * All exception handlers should be of this type, and follow normal ARM EABI register saving conventions
 */
typedef void (*exception_handler_t)(void);

/*! \brief  Set the exception handler for an exception on the executing core.
 *  \ingroup hardware_exception
 *
 * This method will assert if an exception handler has been set for this exception number on this core via
 * this method, without an intervening restore via exception_restore_handler.
 *
 * \note this method may not be used to override an exception handler that was specified at link time by
 * providing a strong replacement for the weakly defined stub exception handlers. It will assert in this case too.
 *
 * \param num Exception number
 * \param handler The handler to set
 * \see exception_number
 */
exception_handler_t exception_set_exclusive_handler(enum exception_number num, exception_handler_t handler);

/*! \brief Restore the original exception handler for an exception on this core
 *  \ingroup hardware_exception
 *
 * This method may be used to restore the exception handler for an exception on this core to the state
 * prior to the call to exception_set_exclusive_handler(), so that exception_set_exclusive_handler()
 * may be called again in the future.
 *
 * \param num Exception number \ref exception_number
 * \param original_handler The original handler returned from \ref exception_set_exclusive_handler
 * \see exception_set_exclusive_handler()
 */
void exception_restore_handler(enum exception_number num, exception_handler_t original_handler);

/*! \brief Get the current exception handler for the specified exception from the currently installed vector table
 * of the execution core
 *  \ingroup hardware_exception
 *
 * \param num Exception number
 * \return the address stored in the VTABLE for the given exception number
 */
exception_handler_t exception_get_vtable_handler(enum exception_number num);

#ifndef __riscv
/*! \brief Set specified exception's priority
 *  \ingroup hardware_exception
 *
 * \param num Exception number \ref exception_number
 * \param hardware_priority Priority to set.
 *
 * Numerically-lower values indicate a higher priority. Hardware priorities
 * range from 0 (highest priority) to 255 (lowest priority).
 *
 * \if rp2040_specific
 * Only the top 2 bits are significant on ARM Cortex-M0+ on RP2040.
 * \endif
 *
 * \if rp2350_specific
 * Only the top 4 bits are significant on ARM Cortex-M33 on RP2350, and exception priorities
 * are not supported on RISC-V
 * \endif
 */
bool exception_set_priority(uint num, uint8_t hardware_priority);

/*! \brief Get specified exception's priority
 *  \ingroup hardware_exception
 *
 * Numerically-lower values indicate a higher priority. Hardware priorities
 * range from 0 (highest priority) to 255 (lowest priority).
 *
 * \if rp2040_specific
 * Only the top 2 bits are significant on ARM Cortex-M0+ on RP2040.
 * \endif
 *
 * \if rp2350_specific
 * Only the top 4 bits are significant on ARM Cortex-M33 on RP2350, and exception priorities
 * are not supported on RISC-V
 * \endif
 *
 * \param num Exception number \ref exception_number
 * \return the exception priority
 */
uint exception_get_priority(uint num);
#endif

#ifdef __cplusplus
}
#endif

#endif
