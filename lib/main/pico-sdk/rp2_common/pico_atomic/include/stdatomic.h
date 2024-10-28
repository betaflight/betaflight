/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2024 Stephen Street (stephen@redrocketcomputing.com).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __STDATOMIC_H
#define __STDATOMIC_H

#ifdef __cplusplus
extern "C" {
#endif

/** \file stdatomic.h
 * \defgroup pico_atomic pico_atomic
 *
 * \brief Helper implementations for C11 atomics
 *
 * \if rp2040_specific
 * On RP2040 a spin lock is used as protection for all atomic operations, since there is no C library support.
 * \endif
 *
 * \if rp2350_specific
 * On RP2350 the C-library provides implementations for all 1-byte, 2-byte and 4-byte atomics using processor
 * exclusive operations. This library provides a spin-lock protected version for arbitrary-sized atomics (including 64-bit).
 * \endif
*/
#include <stdint.h>
#include_next <stdatomic.h>

// needed for PICO_C_COMPILER_IS_GNU
#include "pico.h"

#if PICO_RP2040 && PICO_C_COMPILER_IS_GNU
// on GNU without exclusive instructions these don't get routed thru _1 functions for some reason
#undef atomic_flag_test_and_set
#undef atomic_flag_test_and_set_explicit

extern _Bool __atomic_test_and_set_c(volatile void *mem, int model);

#define atomic_flag_test_and_set(PTR) __atomic_test_and_set_c((PTR), __ATOMIC_SEQ_CST)
#define atomic_flag_test_and_set_explicit(PTR, MO) __atomic_test_and_set_c((PTR), (MO))
#endif

#ifdef __cplusplus
}
#endif
#endif
