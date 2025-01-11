/*
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NRFX_GLUE_H__
#define NRFX_GLUE_H__

// THIS IS A TEMPLATE FILE.
// It should be copied to a suitable location within the host environment into
// which nrfx is integrated, and the following macros should be provided with
// appropriate implementations.
// And this comment should be removed from the customized file.

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_glue nrfx_glue.h
 * @{
 * @ingroup nrfx
 *
 * @brief This file contains macros that should be implemented according to
 *        the needs of the host environment into which @em nrfx is integrated.
 */

// Uncomment this line to use the standard MDK way of binding IRQ handlers
// at linking time.
#include <soc/nrfx_irqs.h>

//------------------------------------------------------------------------------

/**
 * @brief Macro for placing a runtime assertion.
 *
 * @param expression  Expression to evaluate.
 */
#define NRFX_ASSERT(expression)

/**
 * @brief Macro for placing a compile time assertion.
 *
 * @param expression  Expression to evaluate.
 */
#define NRFX_STATIC_ASSERT(expression)

//------------------------------------------------------------------------------

/**
 * @brief Macro for setting the priority of a specific IRQ.
 *
 * @param irq_number  IRQ number.
 * @param priority    Priority to set.
 */
#define NRFX_IRQ_PRIORITY_SET(irq_number, priority) _NRFX_IRQ_PRIORITY_SET(irq_number, priority)
static inline void _NRFX_IRQ_PRIORITY_SET(IRQn_Type irq_number,
                                          uint8_t   priority)
{
    NRFX_ASSERT(INTERRUPT_PRIORITY_IS_VALID(priority));
    NVIC_SetPriority(irq_number, priority);
}

/**
 * @brief Macro for enabling a specific IRQ.
 *
 * @param irq_number  IRQ number.
 */
#define NRFX_IRQ_ENABLE(irq_number)  _NRFX_IRQ_ENABLE(irq_number)
static inline void _NRFX_IRQ_ENABLE(IRQn_Type irq_number)
{
    NVIC_ClearPendingIRQ(irq_number);
    NVIC_EnableIRQ(irq_number);
}

/**
 * @brief Macro for checking if a specific IRQ is enabled.
 *
 * @param irq_number  IRQ number.
 *
 * @retval true  If the IRQ is enabled.
 * @retval false Otherwise.
 */
#define NRFX_IRQ_IS_ENABLED(irq_number)  _NRFX_IRQ_IS_ENABLED(irq_number)
static inline bool _NRFX_IRQ_IS_ENABLED(IRQn_Type irq_number)
{
    return 0 != (NVIC->ISER[irq_number / 32] & (1UL << (irq_number % 32)));
}

/**
 * @brief Macro for disabling a specific IRQ.
 *
 * @param irq_number  IRQ number.
 */
#define NRFX_IRQ_DISABLE(irq_number)  _NRFX_IRQ_DISABLE(irq_number)
static inline void _NRFX_IRQ_DISABLE(IRQn_Type irq_number)
{
    NVIC_DisableIRQ(irq_number);
}

/**
 * @brief Macro for setting a specific IRQ as pending.
 *
 * @param irq_number  IRQ number.
 */
#define NRFX_IRQ_PENDING_SET(irq_number) _NRFX_IRQ_PENDING_SET(irq_number)
static inline void _NRFX_IRQ_PENDING_SET(IRQn_Type irq_number)
{
    NVIC_SetPendingIRQ(irq_number);
}

/**
 * @brief Macro for clearing the pending status of a specific IRQ.
 *
 * @param irq_number  IRQ number.
 */
#define NRFX_IRQ_PENDING_CLEAR(irq_number) _NRFX_IRQ_PENDING_CLEAR(irq_number)
static inline void _NRFX_IRQ_PENDING_CLEAR(IRQn_Type irq_number)
{
    NVIC_ClearPendingIRQ(irq_number);
}

/**
 * @brief Macro for checking the pending status of a specific IRQ.
 *
 * @retval true  If the IRQ is pending.
 * @retval false Otherwise.
 */
#define NRFX_IRQ_IS_PENDING(irq_number) _NRFX_IRQ_IS_PENDING(irq_number)
static inline bool _NRFX_IRQ_IS_PENDING(IRQn_Type irq_number)
{
    return (NVIC_GetPendingIRQ(irq_number) == 1);
}

/**
 * @brief Macro for entering into a critical section.
 */
#define NRFX_CRITICAL_SECTION_ENTER()

/**
 * @brief Macro for exiting from a critical section.
 */
#define NRFX_CRITICAL_SECTION_EXIT()

//------------------------------------------------------------------------------

/**
 * @brief When set to a non-zero value, this macro specifies that
 *        @ref nrfx_coredep_delay_us uses a precise DWT-based solution.
 *        A compilation error is generated if the DWT unit is not present
 *        in the SoC used.
 */
#define NRFX_DELAY_DWT_BASED    0

/**
 * @brief Macro for delaying the code execution for at least the specified time.
 *
 * @param us_time Number of microseconds to wait.
 */
#include <soc/nrfx_coredep.h>
#define NRFX_DELAY_US(us_time) nrfx_coredep_delay_us(us_time)

//------------------------------------------------------------------------------

/**
 * @brief When set to a non-zero value, this macro specifies that the
 *        @ref nrfx_error_codes and the @ref nrfx_err_t type itself are defined
 *        in a customized way and the default definitions from @c <nrfx_error.h>
 *        should not be used.
 */
#define NRFX_CUSTOM_ERROR_CODES 0

//------------------------------------------------------------------------------

/**
 * @brief Bitmask defining PPI channels reserved to be used outside of nrfx.
 */
#define NRFX_PPI_CHANNELS_USED  0

/**
 * @brief Bitmask defining PPI groups reserved to be used outside of nrfx.
 */
#define NRFX_PPI_GROUPS_USED    0

/**
 * @brief Bitmask defining SWI instances reserved to be used outside of nrfx.
 */
#define NRFX_SWI_USED           0

/**
 * @brief Bitmask defining TIMER instances reserved to be used outside of nrfx.
 */
#define NRFX_TIMERS_USED        0

/** @} */

//------------------------------------------------------------------------------

#include <soc/nrfx_atomic.h>

/**
 * @brief Atomic 32 bit unsigned type.
 */
#define nrfx_atomic_t               nrfx_atomic_u32_t

/**
 * @brief Stores value to an atomic object and returns previously stored value.
 *
 * @param[in] p_data  Atomic memory pointer.
 * @param[in] value   Value to store.
 *
 * @return Old value stored into atomic object.
 */
#define NRFX_ATOMIC_FETCH_STORE(p_data, value) nrfx_atomic_u32_fetch_store(p_data, value)

/**
 * @brief Performs logical OR operation on an atomic object and returns previously stored value.
 *
 * @param[in] p_data  Atomic memory pointer.
 * @param[in] value   Value of second operand of OR operation.
 *
 * @return Old value stored into atomic object.
 */
#define NRFX_ATOMIC_FETCH_OR(p_data, value)   nrfx_atomic_u32_fetch_or(p_data, value)

/**
 * @brief Performs logical AND operation on an atomic object and returns previously stored value.
 *
 * @param[in] p_data  Atomic memory pointer.
 * @param[in] value   Value of second operand of AND operation.
 *
 * @return Old value stored into atomic object.
 */
#define NRFX_ATOMIC_FETCH_AND(p_data, value)   nrfx_atomic_u32_fetch_and(p_data, value)

/**
 * @brief Performs logical XOR operation on an atomic object and returns previously stored value.
 *
 * @param[in] p_data  Atomic memory pointer.
 * @param[in] value   Value of second operand of XOR operation.
 *
 * @return Old value stored into atomic object.
 */
#define NRFX_ATOMIC_FETCH_XOR(p_data, value)   nrfx_atomic_u32_fetch_xor(p_data, value)

/**
 * @brief Performs logical ADD operation on an atomic object and returns previously stored value.
 *
 * @param[in] p_data  Atomic memory pointer.
 * @param[in] value   Value of second operand of ADD operation.
 *
 * @return Old value stored into atomic object.
 */
#define NRFX_ATOMIC_FETCH_ADD(p_data, value)   nrfx_atomic_u32_fetch_add(p_data, value)

/**
 * @brief Performs logical SUB operation on an atomic object and returns previously stored value.
 *
 * @param[in] p_data  Atomic memory pointer.
 * @param[in] value   Value of second operand of SUB operation.
 *
 * @return Old value stored into atomic object.
 */
#define NRFX_ATOMIC_FETCH_SUB(p_data, value)   nrfx_atomic_u32_fetch_sub(p_data, value)

#ifdef __cplusplus
}
#endif

#endif // NRFX_GLUE_H__
