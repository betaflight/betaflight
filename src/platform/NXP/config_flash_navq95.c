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

/*
 * CONFIG_IN_FLASH back-end for the FlexSPI boot NOR. The config region
 * [__config_start, __config_end) is read through the XIP window; each
 * configWriteWord() call programs one 256-byte page and erases the 4 KB sector
 * first when the page starts one.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(CONFIG_IN_FLASH)

#include "config/config_streamer.h"
#include "config/config_streamer_impl.h"

#include "flexspi_nor_imxrt.h"

extern struct linker_symbol __config_start;
extern struct linker_symbol __config_end;

#define NAVQ95_CFG_PAGE   FLEXSPI_NOR_PAGE_SIZE
#define NAVQ95_CFG_SECTOR FLEXSPI_NOR_SECTOR_SIZE

STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == NAVQ95_CFG_PAGE,
              navq95_config_streamer_buffer_must_equal_nor_page);

static bool norInited;

static bool navq95ConfigNorReady(void)
{
    if (!norInited) {
        norInited = flexspiNorInit();
    }
    return norInited;
}

void configUnlock(void)
{
    navq95ConfigNorReady();
}

void configLock(void)
{
    // Drop the AHB read buffers so the XIP window returns the new data.
    flexspiNorFlush();
}

void configClearFlags(void)
{
}

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    if (!navq95ConfigNorReady()) {
        return CONFIG_RESULT_FAILURE;
    }

    const uint32_t addr = (uint32_t)address;

    if ((addr % NAVQ95_CFG_SECTOR) == 0u) {
        if (!flexspiNorEraseSector(addr)) {
            return CONFIG_RESULT_FAILURE;
        }
    }

    if (!flexspiNorProgramPage(addr, (const uint32_t *)buffer)) {
        return CONFIG_RESULT_ADDRESS_INVALID;
    }

    return CONFIG_RESULT_SUCCESS;
}

#endif /* CONFIG_IN_FLASH */
