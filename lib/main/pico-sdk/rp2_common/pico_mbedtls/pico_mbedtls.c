/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include "pico.h"
#include "pico/rand.h"
#include "mbedtls/sha256.h"
#include "common.h"

/* Function to feed mbedtls entropy. */
int mbedtls_hardware_poll(void *data __unused, unsigned char *output, size_t len, size_t *olen) {
    *olen = 0;
    while(*olen < len) {
        uint64_t rand_data = get_rand_64();
        size_t to_copy = MIN(len, sizeof(rand_data));
        memcpy(output + *olen, &rand_data, to_copy);
        *olen += to_copy;
    }
    return 0;
}

#ifdef MBEDTLS_SHA256_ALT
#if !LIB_PICO_SHA256
#error SHA256 hardware acceleration not supported
#endif

// PICO_CONFIG: PICO_MBEDTLS_SHA256_ALT_USE_DMA, Whether to use DMA for writing to hardware for the mbedtls SHA-256 hardware acceleration, type=int, default=1, group=pico_stdlib
#ifndef PICO_MBEDTLS_SHA256_ALT_USE_DMA
#define PICO_MBEDTLS_SHA256_ALT_USE_DMA 1
#endif

void mbedtls_sha256_init(__unused mbedtls_sha256_context *ctx) {
}

void mbedtls_sha256_free(__unused mbedtls_sha256_context *ctx) {
}

int mbedtls_sha256_starts_ret(mbedtls_sha256_context *ctx, int is224) {
    hard_assert(!is224); // that's annoying
    return pico_sha256_start_blocking(ctx, SHA256_BIG_ENDIAN, PICO_MBEDTLS_SHA256_ALT_USE_DMA);
}

int mbedtls_sha256_update_ret(mbedtls_sha256_context *ctx, const unsigned char *input, size_t ilen) {
    pico_sha256_update_blocking(ctx, input, ilen);
    return 0;
}

int mbedtls_sha256_finish_ret( mbedtls_sha256_context *ctx, unsigned char output[32]) {
    sha256_result_t result;
    pico_sha256_finish(ctx, &result);
    memcpy(output, result.bytes, 32);
    return 0;
}
#endif // MBEDTLS_SHA256_ALT
