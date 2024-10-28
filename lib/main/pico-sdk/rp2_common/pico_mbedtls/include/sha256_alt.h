/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PICO_MBEDTLS_SHA256_ALT_H
#define PICO_MBEDTLS_SHA256_ALT_H

#if LIB_PICO_SHA256
#include "pico/sha256.h"

typedef struct pico_sha256_state mbedtls_sha256_context;
#endif // PICO_RP2350

#endif
