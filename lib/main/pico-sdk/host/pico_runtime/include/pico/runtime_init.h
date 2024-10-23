/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_RUNTIME_INIT_H
#define _PICO_RUNTIME_INIT_H

#include "pico.h"
#include "pico/runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * The runtime initialization is registration based.
 *
 * For each step of the initialization there is a 5 digit ordinal which indicates
 * the ordering (alphabetic increasing sort of the 5 digits) of the steps.
 *
 * e.g. for the step "bootrom_reset", there is:
 *
 * \code
 * #ifndef PICO_RUNTIME_INIT_BOOTROM_RESET
 * #define PICO_RUNTIME_INIT_BOOTROM_RESET   "00050"
 * #endif
 * \endcode
 *
 * The user can override the order if they wish, by redefining PICO_RUNTIME_INIT_BOOTROM_RESET
 *
 * For each step, the automatic initialization may be skipped by defining (in this case)
 * PICO_RUNTIME_SKIP_INIT_BOOTROM_RESET = 1. The user can then choose to either omit the step
 * completely or register their own replacement initialization.
 *
 * The default method used to perform the initialization is provided, in case the user
 * wishes to call it manually; in this case:
 *
 * \code
 * void runtime_init_bootrom_reset(void);
 * \endcode
 *
 * If PICO_RUNTIME_NO_INIT_BOOTOROM_RESET define is set (NO vs SKIP above), then the function
 * is not defined, allowing the user to provide a replacement (and also avoiding
 * cases where the default implementation won't compile due to missing dependencies)
 */

// must have no dependency on any other initialization code
#define PICO_RUNTIME_INIT_EARLIEST              "00001"

#define PICO_RUNTIME_INIT_LATEST                "99999"

// not supported on host at, (needs custom section)
#define PICO_RUNTIME_NO_INIT_MUTEX 1

#ifdef __cplusplus
}
#endif

#endif