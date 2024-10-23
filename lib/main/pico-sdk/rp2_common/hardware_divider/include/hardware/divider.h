/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_DIVIDER_H
#define _HARDWARE_DIVIDER_H

#include "pico.h"

/** \file hardware/divider.h
 *  \defgroup hardware_divider hardware_divider
 *
 * \brief RP2040 Low Low-level hardware-divider API. Non-RP2040 platforms provide software versions of all the functions
 *
 * The SIO contains an 8-cycle signed/unsigned divide/modulo circuit, per core. Calculation is started by writing a dividend
 * and divisor to the two argument registers, DIVIDEND and DIVISOR. The divider calculates the quotient / and remainder % of
 * this division over the next 8 cycles, and on the 9th cycle the results can be read from the two result registers
 * DIV_QUOTIENT and DIV_REMAINDER. A 'ready' bit in register DIV_CSR can be polled to wait for the calculation to
 * complete, or software can insert a fixed 8-cycle delay
 *
 * This header provides low level macros and inline functions for accessing the hardware dividers directly,
 * and perhaps most usefully performing asynchronous divides. These functions however do not follow the regular
 * SDK conventions for saving/restoring the divider state, so are not generally safe to call from interrupt handlers
 *
 * The pico_divider library provides a more user friendly set of APIs over the divider (and support for
 * 64 bit divides), and of course by default regular C language integer divisions are redirected through that library, meaning
 * you can just use C level `/` and `%` operators and gain the benefits of the fast hardware divider.
 *
 * \if rp2350_specific
 * On RP2350 there is no hardware divider, and the functions are implemented in software
 * \endif
 *
 * @see pico_divider
 *
 * \subsection divider_example Example
 * \addtogroup hardware_divider
 * \include hello_divider.c
 */

#if HAS_SIO_DIVIDER
#include "hardware/structs/sio.h"
#else
#define PICO_EMULATE_DIVIDER 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef uint64_t divmod_result_t;

#if PICO_EMULATE_DIVIDER
extern divmod_result_t hw_divider_results[NUM_CORES];

static inline int __sign_of(int32_t v) {
    return v > 0 ? 1 : (v < 0 ? -1 : 0);
}
#endif

/*! \brief Do a signed HW divide and wait for result
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return result as a pair of 32-bit quotient/remainder values.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Results of divide as a pair of 32-bit quotient/remainder values.
 */
#if !PICO_EMULATE_DIVIDER
divmod_result_t hw_divider_divmod_s32(int32_t a, int32_t b);
#else
static inline divmod_result_t hw_divider_divmod_s32(int32_t a, int32_t b) {
    if (!b) return (((uint64_t)a)<<32u) | (uint32_t)(-__sign_of(a));
    return (((uint64_t)(a%b))<<32u) | (uint32_t)(a/b);
}
#endif

/*! \brief Do an unsigned HW divide and wait for result
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return result as a pair of 32-bit quotient/remainder values.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Results of divide as a pair of 32-bit quotient/remainder values.
 */
#if !PICO_EMULATE_DIVIDER
divmod_result_t hw_divider_divmod_u32(uint32_t a, uint32_t b);
#else
static inline divmod_result_t hw_divider_divmod_u32(uint32_t a, uint32_t b)  {
    if (!b) return (((uint64_t)a)<<32u) | (uint32_t)(-1); // todo check this
    return (((uint64_t)(a%b))<<32u) | (a/b);
}
#endif

/*! \brief Start a signed asynchronous divide
 *  \ingroup hardware_divider
 *
 * Start a divide of the specified signed parameters. You should wait for 8 cycles (__div_pause()) or wait for the ready bit to be set
 * (hw_divider_wait_ready()) prior to reading the results.
 *
 * \param a The dividend
 * \param b The divisor
 */
static inline void hw_divider_divmod_s32_start(int32_t a, int32_t b) {
#if !PICO_EMULATE_DIVIDER
    check_hw_layout( sio_hw_t, div_sdividend, SIO_DIV_SDIVIDEND_OFFSET);
    sio_hw->div_sdividend = (uint32_t)a;
    sio_hw->div_sdivisor = (uint32_t)b;
#else
    hw_divider_divmod_s32(a, b);
#endif
}

/*! \brief Start an unsigned asynchronous divide
 *  \ingroup hardware_divider
 *
 * Start a divide of the specified unsigned parameters. You should wait for 8 cycles (__div_pause()) or wait for the ready bit to be set
 * (hw_divider_wait_ready()) prior to reading the results.
 *
 * \param a The dividend
 * \param b The divisor
 */
static inline void hw_divider_divmod_u32_start(uint32_t a, uint32_t b) {
#if !PICO_EMULATE_DIVIDER
    check_hw_layout(
            sio_hw_t, div_udividend, SIO_DIV_UDIVIDEND_OFFSET);
    sio_hw->div_udividend = a;
    sio_hw->div_udivisor = b;
#else
    hw_divider_divmod_u32(a, b);
#endif
}

/*! \brief Wait for a divide to complete
 *  \ingroup hardware_divider
 *
 * Wait for a divide to complete
 */
static inline void hw_divider_wait_ready(void) {
#if !PICO_EMULATE_DIVIDER
    // this is #1 in lsr below
    static_assert(SIO_DIV_CSR_READY_BITS == 1, "");

    // we use one less register and instruction than gcc which uses a TST instruction

    uint32_t tmp; // allow compiler to pick scratch register
    pico_default_asm_volatile (
    "hw_divider_result_loop_%=:"
    "ldr %0, [%1, %2]\n\t"
    "lsrs %0, %0, #1\n\t"
    "bcc hw_divider_result_loop_%=\n\t"
    : "=&l" (tmp)
    : "l" (sio_hw), "I" (SIO_DIV_CSR_OFFSET)
    : "cc"
    );
#endif
}

/*! \brief Return result of HW divide, nowait
 *  \ingroup hardware_divider
 *
 * \note This is UNSAFE in that the calculation may not have been completed.
 *
 * \return Current result. Most significant 32 bits are the remainder, lower 32 bits are the quotient.
 */
static inline divmod_result_t hw_divider_result_nowait(void) {
#if !PICO_EMULATE_DIVIDER
    // as ugly as this looks it is actually quite efficient
    divmod_result_t rc = ((divmod_result_t) sio_hw->div_remainder) << 32u;
    rc |= sio_hw->div_quotient;
    return rc;
#else
    return hw_divider_results[get_core_num()];
#endif
}

/*! \brief Return result of last asynchronous HW divide
 *  \ingroup hardware_divider
 *
 * This function waits for the result to be ready by calling hw_divider_wait_ready().
 *
 * \return Current result. Most significant 32 bits are the remainder, lower 32 bits are the quotient.
 */
static inline divmod_result_t hw_divider_result_wait(void) {
    hw_divider_wait_ready();
    return hw_divider_result_nowait();
}

/*! \brief Efficient extraction of unsigned quotient from 32p32 fixed point
 *  \ingroup hardware_divider
 *
 * \param r A pair of 32-bit quotient/remainder values.
 * \return Unsigned quotient
 */
inline static uint32_t to_quotient_u32(divmod_result_t r) {
    return (uint32_t) r;
}

/*! \brief Efficient extraction of signed quotient from 32p32 fixed point
 *  \ingroup hardware_divider
 *
 * \param r A pair of 32-bit quotient/remainder values.
 * \return Unsigned quotient
 */
inline static int32_t to_quotient_s32(divmod_result_t r) {
    return (int32_t)(uint32_t)r;
}

/*! \brief Efficient extraction of unsigned remainder from 32p32 fixed point
 *  \ingroup hardware_divider
 *
 * \param r A pair of 32-bit quotient/remainder values.
 * \return Unsigned remainder
 *
 * \note On Arm this is just a 32 bit register move or a nop
 */
inline static uint32_t to_remainder_u32(divmod_result_t r) {
    return (uint32_t)(r >> 32u);
}

/*! \brief Efficient extraction of signed remainder from 32p32 fixed point
 *  \ingroup hardware_divider
 *
 * \param r A pair of 32-bit quotient/remainder values.
 * \return Signed remainder
 *
 * \note On arm this is just a 32 bit register move or a nop
 */
inline static int32_t to_remainder_s32(divmod_result_t r) {
    return (int32_t)(r >> 32u);
}


/*! \brief Return result of last asynchronous HW divide, unsigned quotient only
 *  \ingroup hardware_divider
 *
 * This function waits for the result to be ready by calling hw_divider_wait_ready().
 *
 * \return Current unsigned quotient result.
 */
static inline uint32_t hw_divider_u32_quotient_wait(void) {
#if !PICO_EMULATE_DIVIDER
    hw_divider_wait_ready();
    return sio_hw->div_quotient;
#else
    return to_quotient_u32(hw_divider_result_wait());
#endif
}

/*! \brief Return result of last asynchronous HW divide, signed quotient only
 *  \ingroup hardware_divider
 *
 * This function waits for the result to be ready by calling hw_divider_wait_ready().
 *
 * \return Current signed quotient result.
 */
static inline int32_t hw_divider_s32_quotient_wait(void) {
#if !PICO_EMULATE_DIVIDER
    hw_divider_wait_ready();
    return (int32_t)sio_hw->div_quotient;
#else
    return to_quotient_s32(hw_divider_result_wait());
#endif
}

/*! \brief Return result of last asynchronous HW divide, unsigned remainder only
 *  \ingroup hardware_divider
 *
 * This function waits for the result to be ready by calling hw_divider_wait_ready().
 *
 * \return Current unsigned remainder result.
 */
static inline uint32_t hw_divider_u32_remainder_wait(void) {
#if !PICO_EMULATE_DIVIDER
    hw_divider_wait_ready();
    uint32_t rc = sio_hw->div_remainder;
    sio_hw->div_quotient; // must read quotient to cooperate with other SDK code
    return rc;
#else
    return to_remainder_u32(hw_divider_result_wait());
#endif
}

/*! \brief Return result of last asynchronous HW divide, signed remainder only
 *  \ingroup hardware_divider
 *
 * This function waits for the result to be ready by calling hw_divider_wait_ready().
 *
 * \return Current remainder results.
 */
static inline int32_t hw_divider_s32_remainder_wait(void) {
#if !PICO_EMULATE_DIVIDER
    hw_divider_wait_ready();
    int32_t rc = (int32_t)sio_hw->div_remainder;
    sio_hw->div_quotient; // must read quotient to cooperate with other SDK code
    return rc;
#else
    return to_remainder_s32(hw_divider_result_wait());
#endif
}

/*! \brief Do an unsigned HW divide, wait for result, return quotient
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return quotient.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Quotient results of the divide
 */
static inline uint32_t hw_divider_u32_quotient(uint32_t a, uint32_t b) {
#if !PICO_EMULATE_DIVIDER
    return to_quotient_u32(hw_divider_divmod_u32(a, b));
#else
    return b ? (a / b) : (uint32_t)(-1);
#endif
}

/*! \brief Do an unsigned HW divide, wait for result, return remainder
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return remainder.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Remainder results of the divide
 */
static inline uint32_t hw_divider_u32_remainder(uint32_t a, uint32_t b) {
#if !PICO_EMULATE_DIVIDER
    return to_remainder_u32(hw_divider_divmod_u32(a, b));
#else
    return b ? (a % b) : a;
#endif
}

/*! \brief Do a signed HW divide, wait for result, return quotient
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return quotient.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Quotient results of the divide
 */
static inline int32_t hw_divider_quotient_s32(int32_t a, int32_t b) {
#if !PICO_EMULATE_DIVIDER
    return to_quotient_s32(hw_divider_divmod_s32(a, b));
#else
    return b ? (a / b) : -1;
#endif
}

/*! \brief Do a signed HW divide, wait for result, return remainder
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return remainder.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Remainder results of the divide
 */
static inline int32_t hw_divider_remainder_s32(int32_t a, int32_t b) {
#if !PICO_EMULATE_DIVIDER
    return to_remainder_s32(hw_divider_divmod_s32(a, b));
#else
    return b ? (a % b) : a;
#endif
}

/*! \brief Pause for exact amount of time needed for a asynchronous divide to complete
 *  \ingroup hardware_divider
 */
static inline void hw_divider_pause(void) {
#if !PICO_EMULATE_DIVIDER
    pico_default_asm_volatile(
    "b _1_%=\n"
    "_1_%=:\n"
    "b _2_%=\n"
    "_2_%=:\n"
    "b _3_%=\n"
    "_3_%=:\n"
    "b _4_%=\n"
    "_4_%=:\n"
    :::);
#endif
}

/*! \brief Do a hardware unsigned HW divide, wait for result, return quotient
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return quotient.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Quotient result of the divide
 */
static inline uint32_t hw_divider_u32_quotient_inlined(uint32_t a, uint32_t b) {
#if !PICO_EMULATE_DIVIDER
    hw_divider_divmod_u32_start(a, b);
    hw_divider_pause();
    return sio_hw->div_quotient;
#else
    return hw_divider_u32_quotient(a,b);
#endif
}

/*! \brief Do a hardware unsigned HW divide, wait for result, return remainder
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return remainder.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Remainder result of the divide
 */
static inline uint32_t hw_divider_u32_remainder_inlined(uint32_t a, uint32_t b) {
#if !PICO_EMULATE_DIVIDER
    hw_divider_divmod_u32_start(a, b);
    hw_divider_pause();
    uint32_t rc = sio_hw->div_remainder;
    sio_hw->div_quotient; // must read quotient to cooperate with other SDK code
    return rc;
#else
    return hw_divider_u32_remainder(a,b);
#endif
}

/*! \brief Do a hardware signed HW divide, wait for result, return quotient
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return quotient.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Quotient result of the divide
 */
static inline int32_t hw_divider_s32_quotient_inlined(int32_t a, int32_t b) {
#if !PICO_EMULATE_DIVIDER
    hw_divider_divmod_s32_start(a, b);
    hw_divider_pause();
    return (int32_t)sio_hw->div_quotient;
#else
    return hw_divider_quotient_s32(a,b);
#endif
}

/*! \brief Do a hardware signed HW divide, wait for result, return remainder
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return remainder.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Remainder result of the divide
 */
static inline int32_t hw_divider_s32_remainder_inlined(int32_t a, int32_t b) {
#if !PICO_EMULATE_DIVIDER
    hw_divider_divmod_s32_start(a, b);
    hw_divider_pause();
    int32_t rc = (int32_t)sio_hw->div_remainder;
    sio_hw->div_quotient; // must read quotient to cooperate with other SDK code
    return rc;
#else
    return hw_divider_remainder_s32(a,b);
#endif
}

#if !PICO_EMULATE_DIVIDER
typedef struct {
    uint32_t values[4];
} hw_divider_state_t;
#else
typedef uint64_t hw_divider_state_t;
#endif

/*! \brief Save the calling cores hardware divider state
 *  \ingroup hardware_divider
 *
 * Copy the current core's hardware divider state into the provided structure. This method
 * waits for the divider results to be stable, then copies them to memory.
 * They can be restored via hw_divider_restore_state()
 *
 * \param dest the location to store the divider state
 */
#if !PICO_EMULATE_DIVIDER
void hw_divider_save_state(hw_divider_state_t *dest);
#else
static inline void hw_divider_save_state(hw_divider_state_t *dest) {
    *dest = hw_divider_results[get_core_num()];
}
#endif

/*! \brief Load a saved hardware divider state into the current core's hardware divider
 *  \ingroup hardware_divider
 *
 * Copy the passed hardware divider state into the hardware divider.
 *
 * \param src the location to load the divider state from
 */
#if !PICO_EMULATE_DIVIDER
void hw_divider_restore_state(hw_divider_state_t *src);
#else
static inline void hw_divider_restore_state(hw_divider_state_t *src) {
    hw_divider_results[get_core_num()] = *src;
}
#endif

#ifdef __cplusplus
}
#endif

#endif // _HARDWARE_DIVIDER_H
