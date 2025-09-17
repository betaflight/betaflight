/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

// Streams data out to the EEPROM, padding to the write size as
// needed, and updating the checksum as it goes.

#if defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_MEMORY_MAPPED_FLASH)
#define CONFIG_STREAMER_BUFFER_SIZE 8 // Must not be greater than the smallest flash page size of all compiled-in flash devices.
#define CONFIG_BUFFER_TYPE    uint32_t
#elif defined(CONFIG_IN_RAM) || defined(CONFIG_IN_SDCARD)
#define CONFIG_STREAMER_BUFFER_SIZE 32
#define CONFIG_BUFFER_TYPE    uint64_t
#elif defined(CONFIG_IN_FILE)
#define CONFIG_BUFFER_TYPE    uint32_t
#elif defined(CONFIG_IN_FLASH)
#if defined(FLASH_CONFIG_STREAMER_BUFFER_SIZE)
#define CONFIG_STREAMER_BUFFER_SIZE FLASH_CONFIG_STREAMER_BUFFER_SIZE
#endif
#define CONFIG_BUFFER_TYPE          FLASH_CONFIG_BUFFER_TYPE
#endif

#if !defined(CONFIG_BUFFER_TYPE)
#error "No config buffer alignment set"
#endif

#if !defined(CONFIG_STREAMER_BUFFER_SIZE)
#define CONFIG_STREAMER_BUFFER_SIZE sizeof(CONFIG_BUFFER_TYPE)
typedef uint64_t config_streamer_buffer_align_type_t;
#elif defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H735xx) 
#define CONFIG_STREAMER_BUFFER_SIZE 32  // Flash word = 256-bits
typedef uint64_t config_streamer_buffer_align_type_t;
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define CONFIG_STREAMER_BUFFER_SIZE 16  // Flash word = 128-bits
typedef uint64_t config_streamer_buffer_align_type_t;
#elif defined(STM32G4)
#define CONFIG_STREAMER_BUFFER_SIZE 8   // Flash word = 64-bits
typedef uint64_t config_streamer_buffer_align_type_t;
#else
#define CONFIG_STREAMER_BUFFER_SIZE 4
typedef uint32_t config_streamer_buffer_align_type_t;
#endif

typedef enum {
    CONFIG_RESULT_TIMEOUT           = -4,
    CONFIG_RESULT_ADDRESS_INVALID   = -3,
    CONFIG_RESULT_INCOMPLETE        = -2,
    CONFIG_RESULT_FAILURE           = -1,
    CONFIG_RESULT_SUCCESS           =  0
} configStreamerResult_e;

typedef CONFIG_BUFFER_TYPE config_streamer_buffer_type_t;
STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE % sizeof(config_streamer_buffer_type_t) == 0,  "CONFIG_STREAMER_BUFFER_SIZE does not fit to FLASH WORD size");

typedef struct config_streamer_s {
    uintptr_t address;
    int size;
    union {
        uint8_t b[CONFIG_STREAMER_BUFFER_SIZE];
        config_streamer_buffer_type_t w[CONFIG_STREAMER_BUFFER_SIZE / sizeof(config_streamer_buffer_type_t)];
    } buffer;
    int at;
    configStreamerResult_e err;
    bool unlocked;
} config_streamer_t;

void config_streamer_init(config_streamer_t *c);

void config_streamer_start(config_streamer_t *c, uintptr_t base, size_t size);
configStreamerResult_e config_streamer_write(config_streamer_t *c, const uint8_t *p, size_t size);
configStreamerResult_e config_streamer_flush(config_streamer_t *c);

configStreamerResult_e config_streamer_finish(config_streamer_t *c);
configStreamerResult_e config_streamer_status(config_streamer_t *c);

