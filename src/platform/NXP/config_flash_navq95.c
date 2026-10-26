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
 * NavQ95 (i.MX 95 M7) CONFIG_IN_FLASH platform back-end.
 *
 * Implements the config_streamer_impl.h seam (configUnlock / configLock /
 * configClearFlags / configWriteWord) entirely inside src/platform/NXP (no
 * core edit). Config lives in the FlexSPI boot NOR at [__config_start,
 * __config_end) (navq95_flexspi.ld); erase/program go through the FlexSPI NOR
 * driver (flexspi_nor_imx95.c) and reads come from the XIP-mapped NOR. The core
 * streamer calls configWriteWord() once per FLASH_CONFIG_STREAMER_BUFFER_SIZE
 * (= 256 B, one NOR page) — each call programs one page, erasing the 4 KB
 * sector when the address is sector-aligned.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(CONFIG_IN_FLASH)

#include "config/config_streamer.h"
#include "config/config_streamer_impl.h"

#include "flexspi_nor_imxrt.h"   /* flexspiNorInit / EraseSector / ProgramPage / Flush */

/* The linker places the config region in the NOR; common_post.h declares these
 * as struct linker_symbol under CONFIG_IN_FLASH (take their address). */
extern struct linker_symbol __config_start;
extern struct linker_symbol __config_end;

/* config_streamer writes this many bytes per configWriteWord() call. We size it
 * to one NOR page in target.h so a call maps to exactly one page program. */
#define NAVQ95_CFG_PAGE   FLEXSPI_NOR_PAGE_SIZE     /* 256 */
#define NAVQ95_CFG_SECTOR FLEXSPI_NOR_SECTOR_SIZE   /* 4096 */

STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == NAVQ95_CFG_PAGE,
              navq95_config_streamer_buffer_must_equal_nor_page);

static bool norInited;

/* Ensure the FlexSPI NOR write path is set up (borrows the mcuboot-configured
 * controller + installs the write LUT slots). Idempotent. */
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
    /* Nothing to lock on the FlexSPI NOR; a flush drops the AHB read buffers so
     * the just-written config is read back correctly from the XIP window. */
    flexspiNorFlush();
}

void configClearFlags(void)
{
    /* No sticky controller error flags to clear on this path. */
}

/* Program one streamer word (= one NOR page) at a memory-mapped NOR address,
 * erasing the enclosing 4 KB sector first when the address is sector-aligned.
 * Returns a configStreamerResult_e. */
configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    if (!navq95ConfigNorReady()) {
        return CONFIG_RESULT_FAILURE;
    }

    const uint32_t addr = (uint32_t)address;

    /* Erase the sector when we reach its first page. */
    if ((addr % NAVQ95_CFG_SECTOR) == 0u) {
        if (!flexspiNorEraseSector(addr)) {
            return CONFIG_RESULT_FAILURE;
        }
    }

    /* Program one page. buffer is CONFIG_STREAMER_BUFFER_SIZE (= page) bytes of
     * uint32_t words; flexspiNorProgramPage wants a 32-bit-aligned uint32_t*. */
    if (!flexspiNorProgramPage(addr, (const uint32_t *)buffer)) {
        return CONFIG_RESULT_ADDRESS_INVALID;
    }

    return CONFIG_RESULT_SUCCESS;
}

#endif /* CONFIG_IN_FLASH */
