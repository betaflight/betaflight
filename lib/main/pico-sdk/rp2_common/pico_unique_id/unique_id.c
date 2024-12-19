/*
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/flash.h"
#include "pico/bootrom.h"
#include "pico/unique_id.h"

static_assert(PICO_UNIQUE_BOARD_ID_SIZE_BYTES <= FLASH_UNIQUE_ID_SIZE_BYTES, "Board ID size must at least be the size of flash ID");

static pico_unique_board_id_t retrieved_id;

static void __attribute__((constructor)) _retrieve_unique_id_on_boot(void) {
#if PICO_RP2040
    #if PICO_NO_FLASH
        // The hardware_flash call will panic() if called directly on a NO_FLASH
        // build. Since this constructor is pre-main it would be annoying to
        // debug, so just produce something well-defined and obviously wrong.
        for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++)
            retrieved_id.id[i] = 0xee;
    #elif (PICO_UNIQUE_BOARD_ID_SIZE_BYTES == FLASH_UNIQUE_ID_SIZE_BYTES)
        flash_get_unique_id(retrieved_id.id);
    #elif (PICO_UNIQUE_BOARD_ID_SIZE_BYTES < FLASH_UNIQUE_ID_SIZE_BYTES)
        // The flash ID is >8 bytes (e.g. IS25LP016D) but we want to keep the
        // pico unique board ID as 8 bytes, just use the last 8 bytes which are likely to change
        uint8_t flash_id[FLASH_UNIQUE_ID_SIZE_BYTES];
        flash_get_unique_id(flash_id);
        memcpy(retrieved_id.id, flash_id + FLASH_UNIQUE_ID_SIZE_BYTES - PICO_UNIQUE_BOARD_ID_SIZE_BYTES, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
    #else
        #error unique board ID size is greater than flash unique ID size
    #endif
#else
    rom_get_sys_info_fn func = (rom_get_sys_info_fn) rom_func_lookup(ROM_FUNC_GET_SYS_INFO);
    union {
        uint32_t words[9];
        uint8_t bytes[9 * 4];
    } out;
    __unused int rc = func(out.words, 9, SYS_INFO_CHIP_INFO);
    assert(rc == 4);
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        retrieved_id.id[i] = out.bytes[PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 1 + 2 * 4 - i];
    }
#endif
}

void pico_get_unique_board_id(pico_unique_board_id_t *id_out) {
    *id_out = retrieved_id;
}

void pico_get_unique_board_id_string(char *id_out, uint len) {
    assert(len > 0);
    size_t i;
    // Generate hex one nibble at a time
    for (i = 0; (i < len - 1) && (i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2); i++) {
        int nibble = (retrieved_id.id[i/2] >> (4 - 4 * (i&1))) & 0xf;
        id_out[i] = (char)(nibble < 10 ? nibble + '0' : nibble + 'A' - 10);
    }
    id_out[i] = 0;
}
