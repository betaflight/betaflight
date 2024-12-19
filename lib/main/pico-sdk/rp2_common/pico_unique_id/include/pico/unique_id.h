/*
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_UNIQUE_ID_H
#define _PICO_UNIQUE_ID_H

#include "pico.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \file pico/unique_id.h
 *  \defgroup pico_unique_id pico_unique_id
 *
 * \brief Unique device ID access API
 *
 * \if rp2040_specific
 * RP2040 does not have an on-board unique identifier (all instances of RP2040
 * silicon are identical and have no persistent state). However, RP2040 boots
 * from serial NOR flash devices which have at least a 64-bit unique ID as a standard
 * feature, and there is a 1:1 association between RP2040 and flash, so this
 * is suitable for use as a unique identifier for an RP2040-based board.
 *
 * This library injects a call to the flash_get_unique_id function from the
 * hardware_flash library, to run before main, and stores the result in a
 * static location which can safely be accessed at any time via
 * pico_get_unique_id().
 *
 * This avoids some pitfalls of the hardware_flash API, which requires any
 * flash-resident interrupt routines to be disabled when called into.
 * \endif
 *
 * \if rp2350_specific
 * On boards using RP2350, the unique identifier is read from OTP memory on boot.
 * \endif
 */

#define PICO_UNIQUE_BOARD_ID_SIZE_BYTES 8

/**
 * \brief Unique board identifier
 * \ingroup pico_unique_id
 *
 * This structure contains an array of PICO_UNIQUE_BOARD_ID_SIZE_BYTES identifier bytes suitable for
 * holding the unique identifier for the device.
 *
 * \if rp2040_specific
 * On an RP2040-based board, the unique identifier is retrieved from the external NOR flash device at boot,
 * or for PICO_NO_FLASH builds the unique identifier is set to all 0xEE.
 * \endif
 *
 * \if rp2350_specific
 * On an RP2350-based board, the unique identifier is retrieved from OTP memory at boot.
 * \endif
 *
 */
typedef struct {
	uint8_t id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES];
} pico_unique_board_id_t;

/*! \brief Get unique ID
 *  \ingroup pico_unique_id
 *
 * Get the unique 64-bit device identifier.
 *
 * \if rp2040_specific
 * On an RP2040-based board, the unique identifier is retrieved from the external NOR flash device at boot,
 * or for PICO_NO_FLASH builds the unique identifier is set to all 0xEE.
 * \endif
 *
 * \if rp2350_specific
 * On an RP2350-based board, the unique identifier is retrieved from OTP memory at boot.
 * \endif
 *
 * \param id_out a pointer to a pico_unique_board_id_t struct, to which the identifier will be written
 */
void pico_get_unique_board_id(pico_unique_board_id_t *id_out);

/*! \brief Get unique ID in string format
 *  \ingroup pico_unique_id
 *
 * Get the unique 64-bit device identifier formatted as a 0-terminated ASCII hex string.
 *
 * \if rp2040_specific
 * On an RP2040-based board, the unique identifier is retrieved from the external NOR flash device at boot,
 * or for PICO_NO_FLASH builds the unique identifier is set to all 0xEE.
 * \endif
 *
 * \if rp2350_specific
 * On an RP2350-based board, the unique identifier is retrieved from OTP memory at boot.
 * \endif
 *
 * \param id_out a pointer to a char buffer of size len, to which the identifier will be written
 * \param len the size of id_out. For full serial, len >= 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1
 */
void pico_get_unique_board_id_string(char *id_out, uint len);


#ifdef __cplusplus
}
#endif

#endif
