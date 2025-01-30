/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_RUNTIME_H
#define _PICO_RUNTIME_H

#include "pico.h"

/** \file runtime.h
* \defgroup pico_runtime pico_runtime
* \brief Basic runtime support for running pre-main initializers provided by other libraries
*
* This library aggregates the following other libraries (if available):
*
* * \ref hardware_uart
* * \ref pico_bit_ops
* * \ref pico_divider
* * \ref pico_double
* * \ref pico_int64_ops
* * \ref pico_float
* * \ref pico_malloc
* * \ref pico_mem_ops
* * \ref pico_atomic
* * \ref pico_cxx_options
* * \ref pico_standard_binary_info
* * \ref pico_standard_link
* * \ref pico_sync
* * \ref pico_printf
* * \ref pico_crt0
* * \ref pico_clib_interface
* * \ref pico_stdio
*/

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __ASSEMBLER__
/*! \brief Run all the initializations that are usually called by crt0.S before entering main
 *  \ingroup pico_runtime
 *
 * This method is useful to set up the runtime after performing a watchdog or powman reboot
 * via scratch vector.
 */
void runtime_init(void);

void runtime_run_initializers(void);
void runtime_run_per_core_initializers(void);

#ifndef PICO_RUNTIME_INIT_FUNC
#define PICO_RUNTIME_INIT_FUNC(func, priority_string) uintptr_t __used __attribute__((section(".preinit_array." priority_string))) __pre_init_ ## func = (uintptr_t)(void (*)(void)) (func)
#endif
#else
#ifndef PICO_RUNTIME_INIT_FUNC
#define PICO_RUNTIME_INIT_FUNC(func, priority_string) __pre_init func, priority_string
#endif
#endif
#define PICO_RUNTIME_INIT_FUNC_HW(func, priority_string) PICO_RUNTIME_INIT_FUNC(func, priority_string)
#define PICO_RUNTIME_INIT_FUNC_RUNTIME(func, priority_string) PICO_RUNTIME_INIT_FUNC(func, priority_string)
// priority strings are of the form 00000->99999; we want the per core stuff all at the end, so prefix with ZZZZZ which is clearly after 99999
#define PICO_RUNTIME_INIT_FUNC_PER_CORE(func, priority_string) PICO_RUNTIME_INIT_FUNC(func, "ZZZZZ." priority_string)

#ifdef __cplusplus
}
#endif

#endif