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
*/

#ifdef __cplusplus
extern "C" {
#endif

#define PICO_RUNTIME_INIT_TYPE_HW                 0
#define PICO_RUNTIME_INIT_TYPE_RUNTIME            1
#define PICO_RUNTIME_INIT_TYPE_PER_CORE           2

#ifndef PICO_RUNTIME_INIT_FUNC_FLAGS
#define PICO_RUNTIME_INIT_FUNC_FLAGS(func, flags, priority_string1, priority_string2)
#endif
#define PICO_RUNTIME_INIT_FUNC_HW(func, priority_string1, priority_string2) PICO_RUNTIME_INIT_FUNC_FLAGS(func, PICO_RUNTIME_INIT_TYPE_HW, priority_string1, priority_string2)
#define PICO_RUNTIME_INIT_FUNC_RUNTIME(func, priority_string1, priority_string2) PICO_RUNTIME_INIT_FUNC_FLAGS(func, PICO_RUNTIME_INIT_TYPE_RUNTIME, priority_string1, priority_string2)
#define PICO_RUNTIME_INIT_FUNC_PER_CORE(func, priority_string1, priority_string2) PICO_RUNTIME_INIT_FUNC_FLAGS(func, PICO_RUNTIME_INIT_TYPE_PER_CORE, priority_string1, priority_string2)

#ifdef __cplusplus
}
#endif

#endif