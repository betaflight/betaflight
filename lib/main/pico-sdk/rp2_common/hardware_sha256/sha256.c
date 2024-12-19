/**
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/sha256.h"

void sha256_get_result(sha256_result_t *out, enum sha256_endianness endianness) {
    for (uint i = 0; i < count_of(out->words); i++) {
        uint32_t data = sha256_hw->sum[i];
        if (endianness == SHA256_BIG_ENDIAN) data = __builtin_bswap32(data);
        out->words[i] = data;
    }
}
