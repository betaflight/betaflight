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
#include <string.h>

#include "platform.h"

#include "config/config_eeprom.h"
#include "config/config_streamer.h"
#include "config/config_streamer_impl.h"
#include "drivers/system.h"

#include "flexspi_nor_imxrt.h"

#if defined(CONFIG_IN_FLASH)

// False when configUnlock() could not prepare the flash, so the save writes nothing.
static bool configWritable;

#if defined(ENABLE_CONFIG_FLASH_INIT)

// A backup of the same size follows the configuration region, so a save cut short leaves a record to boot from.
#define CONFIG_START    ((uintptr_t)&__config_start)
#define CONFIG_SIZE     ((uintptr_t)&__config_end - CONFIG_START)
#define CONFIG_BACKUP   ((uintptr_t)&__config_end)

// Second header byte of every record config_eeprom.c writes.
#define CONFIG_MAGIC_BE 0xBE

static bool configCopy(uintptr_t to, uintptr_t from)
{
    // The writer may not read the NOR it writes, so each page goes through RAM.
    uint32_t page[FLEXSPI_NOR_PAGE_SIZE / sizeof(uint32_t)];
    for (uintptr_t at = 0; at < CONFIG_SIZE; at += FLEXSPI_NOR_PAGE_SIZE) {
        if ((at % FLEXSPI_NOR_SECTOR_SIZE) == 0 && !flexspiNorEraseSector(to + at)) {
            return false;
        }
        memcpy(page, (const void *)(from + at), sizeof(page));
        if (!flexspiNorProgramPage(to + at, page)) {
            return false;
        }
    }
    return true;
}

static bool configBackupDiffers(void)
{
    return memcmp((const void *)CONFIG_START, (const void *)CONFIG_BACKUP, CONFIG_SIZE) != 0;
}

void configFlashInit(void)
{
    const uint8_t *backup = (const uint8_t *)CONFIG_BACKUP;

    // Only a backup with this firmware's header replaces a record that does not load.
    if ((!isEEPROMVersionValid() || !isEEPROMStructureValid())
        && backup[0] == EEPROM_CONF_VERSION && backup[1] == CONFIG_MAGIC_BE && configBackupDiffers()
        && flexspiNorInit()) {
        // A failed copy leaves a record that does not load, so the defaults load as before.
        (void)configCopy(CONFIG_START, CONFIG_BACKUP);
        (void)flexspiNorFlush();
    }
}

#endif

void configUnlock(void)
{
    // Retries the flash driver setup if systemInit() failed it.
    configWritable = flexspiNorInit();

#if defined(ENABLE_CONFIG_FLASH_INIT)
    // Only a valid record is backed up, so the retry after a failed save keeps the backup.
    if (configWritable && isEEPROMVersionValid() && isEEPROMStructureValid() && configBackupDiffers()) {
        // The record is the only good copy until the backup is complete.
        configWritable = configCopy(CONFIG_BACKUP, CONFIG_START);
    }
#endif
}

void configLock(void)
{
#if defined(ENABLE_CONFIG_FLASH_INIT)
    // The streamer and the backup copy write without reading back, so one flush here covers the whole save.
    (void)flexspiNorFlush();
#else
    // NOOP - the ROM flash API needs no locking.
#endif
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
