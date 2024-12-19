/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_SHA256_H
#define _PICO_SHA256_H

#include "pico/time.h"
#include "hardware/dma.h"
#include "hardware/sha256.h"

/** \file pico/sha256.h
 *  \defgroup pico_sha256 pico_sha256
 *
 * \brief SHA-256 Hardware Accelerated implementation
 *
 * RP2350 is equipped with a hardware accelerated implementation of the SHA-256 hash algorithm.
 * This should be much quicker than performing a SHA-256 checksum in software.
 *
 * \code
 * pico_sha256_state_t state;
 * if (pico_sha256_try_start(&state, SHA256_BIG_ENDIAN, true) == PICO_OK) {
 *     sha256_result_t result;
 *     pico_sha256_update(&state, some_data, sizeof(some_data));
 *     pico_sha256_update(&state, some_more_data, sizeof(some_more_data));
 *     pico_sha256_finish(&state, &result);
 *     for (int i = 0; i < SHA256_RESULT_BYTES; i++) {
 *         printf("%02x", result.bytes[i]);
 *     }
 * }
 * \endcode
 *
 * \subsection sha256_example Example
 * \addtogroup pico_sha256
 *
 * \include hello_sha256.c
 */

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief SHA-256 state used by the API
 *  \ingroup pico_sha256
 */
typedef struct pico_sha256_state {
    enum sha256_endianness endianness;
    int8_t channel;
    bool locked;
    uint8_t cache_used;
    union {
        uint32_t word;
        uint8_t bytes[4];
    } cache;
    dma_channel_config config;
    size_t total_data_size;
} pico_sha256_state_t;

/*! \brief Start a SHA-256 calculation returning immediately with an error if the SHA-256 hardware is not available
 *  \ingroup pico_sha256
 *
 * Initialises the hardware and state ready to start a new SHA-256 calculation.
 * Only one instance can be started at any time.
 *
 * @param state A pointer to a pico_sha256_state_t instance
 * @param endianness SHA256_BIG_ENDIAN or SHA256_LITTLE_ENDIAN for data in and data out
 * @param use_dma Set to true to use DMA internally to copy data to hardware. This is quicker at the expense of hardware DMA resources.
 * @return Returns PICO_OK if the hardware was available for use and the sha256 calculation could be started, otherwise an error is returned
 */
int pico_sha256_try_start(pico_sha256_state_t *state, enum sha256_endianness endianness, bool use_dma);

/*! \brief Start a SHA-256 calculation waiting for a defined period for the SHA-256 hardware to be available
 *  \ingroup pico_sha256
 *
 * Initialises the hardware and state ready to start a new SHA-256 calculation.
 * Only one instance can be started at any time.
 *
 * @param state A pointer to a pico_sha256_state_t instance
 * @param endianness SHA256_BIG_ENDIAN or SHA256_LITTLE_ENDIAN for data in and data out
 * @param use_dma Set to true to use DMA internally to copy data to hardware. This is quicker at the expense of hardware DMA resources.
 * @param until How long to wait for the SHA hardware to be available
 * @return Returns PICO_OK if the hardware was available for use and the sha256 calculation could be started in time, otherwise an error is returned
 */
int pico_sha256_start_blocking_until(pico_sha256_state_t *state, enum sha256_endianness endianness, bool use_dma, absolute_time_t until);

/*! \brief Start a SHA-256 calculation, blocking forever waiting until the SHA-256 hardware is available
 *  \ingroup pico_sha256
 *
 * Initialises the hardware and state ready to start a new SHA-256 calculation.
 * Only one instance can be started at any time.
 *
 * @param state A pointer to a pico_sha256_state_t instance
 * @param endianness SHA256_BIG_ENDIAN or SHA256_LITTLE_ENDIAN for data in and data out
 * @param use_dma Set to true to use DMA internally to copy data to hardware. This is quicker at the expense of hardware DMA resources.
 * @return Returns PICO_OK if the hardware was available for use and the sha256 calculation could be started, otherwise an error is returned
 */
static inline int pico_sha256_start_blocking(pico_sha256_state_t *state, enum sha256_endianness endianness, bool use_dma) {
    return pico_sha256_start_blocking_until(state, endianness, use_dma, at_the_end_of_time);
}

/*! \brief Add byte data to be SHA-256 calculation
 *  \ingroup pico_sha256
 *
 * Add byte data to be SHA-256 calculation
 * You may call this as many times as required to add all the data needed.
 * You must have called pico_sha256_try_start (or equivalent) already.
 *
 * @param state A pointer to a pico_sha256_state_t instance
 * @param data Pointer to the data to be added to the calculation
 * @param data_size_bytes Amount of data to add
 *
 * @note This function may return before the copy has completed in which case the data passed to the function must remain valid and
 * unchanged until a further call to pico_sha256_update or pico_sha256_finish. If this is not done, corrupt data may be used for the
 * SHA-256 calculation giving an unexpected result.
 */
void pico_sha256_update(pico_sha256_state_t *state, const uint8_t *data, size_t data_size_bytes);

/*! \brief Add byte data to be SHA-256 calculation
 *  \ingroup pico_sha256
 *
 * Add byte data to be SHA-256 calculation
 * You may call this as many times as required to add all the data needed.
 * You must have called pico_sha256_try_start already.
 *
 * @param state A pointer to a pico_sha256_state_t instance
 * @param data Pointer to the data to be added to the calculation
 * @param data_size_bytes Amount of data to add
 *
 * @note This function will only return when the data passed in is no longer required, so it can be freed or changed on return.
 */
void pico_sha256_update_blocking(pico_sha256_state_t *state, const uint8_t *data, size_t data_size_bytes);

/*! \brief Finish the SHA-256 calculation and return the result
 *  \ingroup pico_sha256
 *
 * Ends the SHA-256 calculation freeing the hardware for use by another caller.
 * You must have called pico_sha256_try_start already.
 *
 * @param state A pointer to a pico_sha256_state_t instance
 * @param out The SHA-256 checksum
 */
void pico_sha256_finish(pico_sha256_state_t *state, sha256_result_t *out);

#ifdef __cplusplus
}
#endif

#endif