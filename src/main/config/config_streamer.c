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

#include <string.h>

#include "platform.h"

#include "drivers/system.h"
#include "drivers/flash/flash.h"

#include "config/config_eeprom.h"
#include "config/config_streamer.h"
#include "config/config_streamer_impl.h"

#if !defined(CONFIG_IN_FLASH)
#if defined(CONFIG_IN_RAM) && defined(PERSISTENT)
PERSISTENT uint8_t eepromData[EEPROM_SIZE];
#else
uint8_t eepromData[EEPROM_SIZE];
#endif
#endif

void config_streamer_init(config_streamer_t *c)
{
    memset(c, 0, sizeof(*c));
}

void config_streamer_start(config_streamer_t *c, uintptr_t base, size_t size)
{
    // base must start at FLASH_PAGE_SIZE boundary when using embedded flash.
    c->address = base;
    c->size = size;
    if (!c->unlocked) {
#if defined(CONFIG_IN_RAM) || defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_SDCARD)
        // NOP
#elif defined(CONFIG_IN_FILE) || defined(CONFIG_IN_FLASH)
        configUnlock();
#endif
        c->unlocked = true;
    }

#if defined(CONFIG_IN_RAM) || defined(CONFIG_IN_FILE) || defined(CONFIG_IN_EXTERNAL_FLASH)
    // NOP
#elif defined(CONFIG_IN_FLASH)
    configClearFlags();
#endif
    c->err = CONFIG_RESULT_SUCCESS;
}

static int write_word(config_streamer_t *c, config_streamer_buffer_type_t *buffer)
{
    if (c->err != CONFIG_RESULT_SUCCESS) {
        return c->err;
    }
#if defined(CONFIG_IN_EXTERNAL_FLASH)

    uint32_t dataOffset = (uint32_t)(c->address - (uintptr_t)&eepromData[0]);

    const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_CONFIG);
    const flashGeometry_t *flashGeometry = flashGetGeometry();

    uint32_t flashStartAddress = flashPartition->startSector * flashGeometry->sectorSize;
    uint32_t flashOverflowAddress = ((flashPartition->endSector + 1) * flashGeometry->sectorSize); // +1 to sector for inclusive

    uint32_t flashAddress = flashStartAddress + dataOffset;
    if (flashAddress + CONFIG_STREAMER_BUFFER_SIZE > flashOverflowAddress) {
        return CONFIG_RESULT_ADDRESS_INVALID; // address is past end of partition
    }

    uint32_t flashSectorSize = flashGeometry->sectorSize;
    uint32_t flashPageSize = flashGeometry->pageSize;
    const uint8_t *buffers[1];
    uint32_t bufferSizes[1];

    bool onPageBoundary = (flashAddress % flashPageSize == 0);
    if (onPageBoundary) {

        bool firstPage = (flashAddress == flashStartAddress);
        if (!firstPage) {
            flashPageProgramFinish();
        }

        if (flashAddress % flashSectorSize == 0) {
            flashEraseSector(flashAddress);
        }

        flashPageProgramBegin(flashAddress, NULL);
    }

    buffers[0] = (uint8_t*)buffer;
    bufferSizes[0] = CONFIG_STREAMER_BUFFER_SIZE;

    flashPageProgramContinue(buffers, bufferSizes, 1);

#elif defined(CONFIG_IN_RAM) || defined(CONFIG_IN_SDCARD) || defined(CONFIG_IN_MEMORY_MAPPED_FLASH)
    if (c->address == (uintptr_t)&eepromData[0]) {
        memset(eepromData, 0, sizeof(eepromData));
    }

    uint64_t *dest_addr = (uint64_t*)c->address;
    uint64_t *src_addr = (uint64_t*)buffer;
    uint8_t row_index = CONFIG_STREAMER_BUFFER_SIZE / sizeof(uint64_t);
    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE % sizeof(uint64_t) == 0, "CONFIG_STREAMER_BUFFER_SIZE does not match written size");
    /* copy the 256 bits flash word */
    do {
      *dest_addr++ = *src_addr++;
    } while (--row_index != 0);

#elif defined(CONFIG_IN_FLASH) || defined(CONFIG_IN_FILE)

    configStreamerResult_e result = configWriteWord(c->address, buffer);
    if (result != CONFIG_RESULT_SUCCESS) {
        return result;
    }

#endif
    c->address += CONFIG_STREAMER_BUFFER_SIZE;
    return CONFIG_RESULT_SUCCESS;
}

configStreamerResult_e config_streamer_write(config_streamer_t *c, const uint8_t *p, size_t size)
{
    for (const uint8_t *pat = p; pat != (uint8_t*)p + size; pat++) {
        c->buffer.b[c->at++] = *pat;

        if (c->at == sizeof(c->buffer)) {
            c->err = write_word(c, c->buffer.w);
            c->at = 0;
        }
    }
    return c->err;
}

configStreamerResult_e config_streamer_status(config_streamer_t *c)
{
    return c->err;
}

configStreamerResult_e config_streamer_flush(config_streamer_t *c)
{
    if (c->at != 0) {
        memset(c->buffer.b + c->at, 0, sizeof(c->buffer) - c->at);
        c->err = write_word(c, c->buffer.w);
        c->at = 0;
    }
    return c->err;
}

configStreamerResult_e config_streamer_finish(config_streamer_t *c)
{
    if (c->unlocked) {
#if defined(CONFIG_IN_SDCARD)
        saveEEPROMToSDCard();
#elif defined(CONFIG_IN_EXTERNAL_FLASH)
        flashFlush();
#elif defined(CONFIG_IN_MEMORY_MAPPED_FLASH)
        saveEEPROMToMemoryMappedFlash();
#elif defined(CONFIG_IN_RAM)
        // NOP
#elif defined(CONFIG_IN_FILE) || defined(CONFIG_IN_FLASH)
        configLock();
#endif
        c->unlocked = false;
    }
    return c->err;
}
