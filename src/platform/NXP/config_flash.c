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

// Configuration storage in the memory mapped boot flash; writes go through flexspi_nor_imxrt.c.

#include <stdint.h>

#include "platform.h"

#include "config/config_streamer.h"
#include "config/config_streamer_impl.h"
#include "drivers/system.h"

#include "flexspi_nor_imxrt.h"

#if defined(CONFIG_IN_FLASH)

// False when configUnlock() could not prepare the flash, so the save writes nothing.
static bool configWritable;

void configUnlock(void)
{
    // Retries the flash driver setup if systemInit() failed it.
    configWritable = flexspiNorInit();
}

void configLock(void)
{
    // NOOP - the ROM flash API needs no locking.
}

void configClearFlags(void)
{
    // NOOP
}

STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == FLEXSPI_NOR_PAGE_SIZE,
              "config streamer buffer must be one NOR page");

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    if (!configWritable) {
        return CONFIG_RESULT_FAILURE;
    }

    // The streamer writes in order, so erase each sector as it is reached.
    if ((address % FLEXSPI_NOR_SECTOR_SIZE) == 0) {
        if (!flexspiNorEraseSector(address)) {
            return CONFIG_RESULT_FAILURE;
        }
    }

    if (!flexspiNorProgramPage(address, buffer)) {
        return CONFIG_RESULT_FAILURE;
    }

    return CONFIG_RESULT_SUCCESS;
}

#endif // CONFIG_IN_FLASH
