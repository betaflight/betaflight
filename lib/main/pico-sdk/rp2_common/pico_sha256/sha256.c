/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "hardware/sha256.h"
#include "pico/bootrom/lock.h"
#include "pico/sha256.h"
#include "pico/time.h"

// We add one 0x80 byte, then 8 bytes for the size
#define SHA256_PADDING_DATA_BYTES 9
#define SHA256_BLOCK_SIZE_BYTES 64

bool __weak pico_sha256_lock(pico_sha256_state_t *state) {
    if (!bootrom_try_acquire_lock(BOOTROM_LOCK_SHA_256))
        return false;
    state->locked = true;
    return true;
}

void __weak pico_sha256_unlock(pico_sha256_state_t *state) {
    assert(state->locked);
    bootrom_release_lock(BOOTROM_LOCK_SHA_256);
    state->locked = false;
}

int pico_sha256_try_start(pico_sha256_state_t *state, enum sha256_endianness endianness, bool use_dma) {
    memset(state, 0, sizeof(*state));
    if (!pico_sha256_lock(state)) return PICO_ERROR_RESOURCE_IN_USE;
    state->endianness = endianness;
    if (use_dma) {
        state->channel = (int8_t)dma_claim_unused_channel(false);
        if (state->channel < 0) {
            pico_sha256_unlock(state);
            return PICO_ERROR_INSUFFICIENT_RESOURCES;
        }
        state->config = dma_channel_get_default_config(state->channel);
        channel_config_set_transfer_data_size(&state->config, DMA_SIZE_8);
        channel_config_set_read_increment(&state->config, true);
        channel_config_set_write_increment(&state->config, false);
        channel_config_set_dreq(&state->config, DREQ_SHA256);
        sha256_set_dma_size(1);
    } else {
        state->channel = -1;
    }
    sha256_err_not_ready_clear();
    sha256_set_bswap(endianness == SHA256_BIG_ENDIAN);
    sha256_start();
    state->total_data_size = 0;
    return PICO_OK;
}

int pico_sha256_start_blocking_until(pico_sha256_state_t *state, enum sha256_endianness endianness, bool use_dma, absolute_time_t until) {
    int rc;
    do {
        rc = pico_sha256_try_start(state, endianness, use_dma);
        if (rc != PICO_ERROR_RESOURCE_IN_USE) break;
        if (time_reached(until)) {
            rc = PICO_ERROR_TIMEOUT;
            break;
        }
    } while (true);
    return rc;
}

static void write_to_hardware(pico_sha256_state_t *state, const uint8_t *data, size_t data_size_bytes) {
    if (state->channel >= 0) {
        dma_channel_wait_for_finish_blocking(state->channel);
        assert(!sha256_err_not_ready());
        sha256_wait_ready_blocking();
        dma_channel_configure(
            state->channel,
            &state->config,
            sha256_get_write_addr(),
            data,
            data_size_bytes,
            true
        );
    } else {
        if (!state->cache_used && !(((uintptr_t)data)&3u)) {
            GCC_Like_Pragma("GCC diagnostic ignored \"-Wcast-align\"")
            const uint32_t *data32 = (const uint32_t *)data;
            // aligned writes
            while (data_size_bytes >= 4) {
                // write a whole word
                sha256_wait_ready_blocking();
                sha256_put_word(*data32++);
                data_size_bytes -= 4;
            }
            data = (const uint8_t *)data32;
        }
        while (data_size_bytes--) {
            state->cache.bytes[state->cache_used++] = *data++;
            if (state->cache_used == 4) {
                state->cache_used = 0;
                sha256_wait_ready_blocking();
                sha256_put_word(state->cache.word);
            }
        }
    }
}

static void update_internal(pico_sha256_state_t *state, const uint8_t *data, size_t data_size_bytes) {
    assert(state->locked);
    // must finish off the last 64 byte block first or else sha256_err_not_ready will be true
    size_t bytes_left = ((state->total_data_size + (SHA256_BLOCK_SIZE_BYTES - 1)) & ~(SHA256_BLOCK_SIZE_BYTES - 1)) - state->total_data_size;
    if (bytes_left > data_size_bytes) bytes_left = data_size_bytes;
    if (bytes_left > 0) {
        write_to_hardware(state, data, bytes_left);
        state->total_data_size += bytes_left;
        data_size_bytes -= bytes_left;
        data += bytes_left;
    }
    // Write the rest of the data
    if (data_size_bytes > 0) {
        write_to_hardware(state, data, data_size_bytes);
        state->total_data_size += data_size_bytes;
    }
}

static void add_zero_bytes(pico_sha256_state_t *state, size_t data_size_bytes) {
    uint32_t zero = 0;
    // todo: can be done a bit more efficiently with dma?
    assert(data_size_bytes < INT32_MAX);
    while((int32_t)data_size_bytes > 0) {
        update_internal(state, (uint8_t *)&zero, MIN(4, data_size_bytes));
        data_size_bytes -= 4;
    }
}

void pico_sha256_update(pico_sha256_state_t *state, const uint8_t *data, size_t data_size_bytes) {
    update_internal(state, data, data_size_bytes);
}

void pico_sha256_update_blocking(pico_sha256_state_t *state, const uint8_t *data, size_t data_size_bytes) {
    update_internal(state, data, data_size_bytes);
    if (state->channel >= 0) {
        dma_channel_wait_for_finish_blocking(state->channel);
    }
}

// write the SHA-256 padding to hardware
static void write_padding(pico_sha256_state_t *state) {
    // Has to be a multiple of 64 bytes
    uint64_t size = (state->total_data_size + SHA256_PADDING_DATA_BYTES + (SHA256_BLOCK_SIZE_BYTES - 1)) & ~(SHA256_BLOCK_SIZE_BYTES - 1);
    const size_t user_data_size = state->total_data_size;
    const size_t padding_size_bytes = size - state->total_data_size;

    // append a single '1' bit
    const uint8_t one_bit = 0x80;
    update_internal(state, &one_bit, 1);

    // Zero unused padding
    add_zero_bytes(state, padding_size_bytes - SHA256_PADDING_DATA_BYTES);

    // Add size in bits, big endian
    size = __builtin_bswap64(user_data_size * 8);
    update_internal(state, (uint8_t*)&size, sizeof(uint64_t)); // last write
}

void pico_sha256_finish(pico_sha256_state_t *state, sha256_result_t *out) {
    assert(state->locked);
    // pass NULL to abandon the current hash in case of an error
    if (out) {
        write_padding(state);
        if (state->channel >= 0) {
            dma_channel_wait_for_finish_blocking(state->channel);
            assert(!sha256_err_not_ready());
        }
        sha256_wait_valid_blocking();
        sha256_get_result(out, state->endianness);
    }
    if (state->channel >= 0) {
        dma_channel_cleanup(state->channel);
        dma_channel_unclaim(state->channel);
        state->channel  = -1;
    }
    pico_sha256_unlock(state);
}
