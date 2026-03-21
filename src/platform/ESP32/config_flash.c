/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>

#include "platform.h"
#include "drivers/system.h"
#include "config/config_streamer.h"

#if defined(CONFIG_IN_FLASH)

#include "esp_rom_spiflash.h"

// Flash base address for memory-mapped access
#define ESP32_FLASH_BASE  0x42000000

// Convert memory-mapped address to flash offset
#define FLASH_ADDR_TO_OFFSET(addr)  ((addr) - ESP32_FLASH_BASE)

static bool sectorErased = false;
static uint32_t lastErasedSector = UINT32_MAX;

void configUnlock(void)
{
    sectorErased = false;
    lastErasedSector = UINT32_MAX;
}

void configLock(void)
{
    // NOOP - no locking needed for ROM flash API
}

void configClearFlags(void)
{
    sectorErased = false;
    lastErasedSector = UINT32_MAX;
}

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    uint32_t flashOffset = FLASH_ADDR_TO_OFFSET(address);

    // Erase sector if needed (4KB sectors)
    uint32_t sector = flashOffset / 4096;
    if (sector != lastErasedSector) {
        if (esp_rom_spiflash_erase_sector(sector) != 0) {
            return CONFIG_RESULT_FAILURE;
        }
        lastErasedSector = sector;
        sectorErased = true;
    }

    // Write data (CONFIG_STREAMER_BUFFER_SIZE bytes)
    if (esp_rom_spiflash_write(flashOffset, (const uint32_t *)buffer, CONFIG_STREAMER_BUFFER_SIZE) != 0) {
        return CONFIG_RESULT_FAILURE;
    }

    return CONFIG_RESULT_SUCCESS;
}

#endif // CONFIG_IN_FLASH
