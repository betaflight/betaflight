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
#include "drivers/flash.h"

#include "config/config_streamer.h"

#if !defined(EEPROM_IN_FLASH)
#if defined(EEPROM_IN_RAM) && defined(PERSISTENT)
PERSISTENT uint8_t eepromData[EEPROM_SIZE];
#else
uint8_t eepromData[EEPROM_SIZE];
#endif
#endif


#if defined(STM32H750xx) && !(defined(EEPROM_IN_EXTERNAL_FLASH) || defined(EEPROM_IN_RAM) || defined(EEPROM_IN_SDCARD))
#error "STM32750xx only has one flash page which contains the bootloader, no spare flash pages available, use external storage for persistent config or ram for target testing"
#endif
// @todo this is not strictly correct for F4/F7, where sector sizes are variable
#if !defined(FLASH_PAGE_SIZE)
// F1
# if defined(STM32F10X_MD)
#  define FLASH_PAGE_SIZE                 (0x400)
# elif defined(STM32F10X_HD)
#  define FLASH_PAGE_SIZE                 (0x800)
// F3
# elif defined(STM32F303xC)
#  define FLASH_PAGE_SIZE                 (0x800)
// F4
# elif defined(STM32F40_41xxx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000) // 16K sectors
# elif defined (STM32F411xE)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
# elif defined(STM32F427_437xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
# elif defined (STM32F446xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
// F7
#elif defined(STM32F722xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000) // 16K sectors
# elif defined(STM32F745xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000) // 32K sectors
# elif defined(STM32F746xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000)
# elif defined(STM32F765xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000)
# elif defined(UNIT_TEST)
#  define FLASH_PAGE_SIZE                 (0x400)
// H7
# elif defined(STM32H743xx) || defined(STM32H750xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x20000) // 128K sectors
// SIMULATOR
# elif defined(SIMULATOR_BUILD)
#  define FLASH_PAGE_SIZE                 (0x400)
# else
#  error "Flash page size not defined for target."
# endif
#endif

void config_streamer_init(config_streamer_t *c)
{
    memset(c, 0, sizeof(*c));
}

void config_streamer_start(config_streamer_t *c, uintptr_t base, int size)
{
    // base must start at FLASH_PAGE_SIZE boundary when using embedded flash.
    c->address = base;
    c->size = size;
    if (!c->unlocked) {
#if defined(EEPROM_IN_RAM) || defined(EEPROM_IN_EXTERNAL_FLASH) || defined(EEPROM_IN_SDCARD)
        // NOP
#elif defined(EEPROM_IN_FLASH) || defined(EEPROM_IN_FILE)
#if defined(STM32F7) || defined(STM32H7)
        HAL_FLASH_Unlock();
#else
        FLASH_Unlock();
#endif
#endif
        c->unlocked = true;
    }

#if defined(EEPROM_IN_RAM) || defined(EEPROM_IN_FILE) || defined(EEPROM_IN_EXTERNAL_FLASH)
    // NOP
#elif defined(EEPROM_IN_FLASH)
#if defined(STM32F10X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#elif defined(STM32F303)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F4)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32F7)
    // NOP
#elif defined(STM32H7)
    // NOP
#elif defined(UNIT_TEST) || defined(SIMULATOR_BUILD)
    // NOP
#else
# error "Unsupported CPU"
#endif
#endif
    c->err = 0;
}

#if defined(EEPROM_IN_RAM) || defined(EEPROM_IN_EXTERNAL_FLASH) || defined(EEPROM_IN_SDCARD)
// No flash sector method required.
#elif defined(EEPROM_IN_FLASH)
#if defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx)
/*
Sector 0    0x08000000 - 0x08007FFF 32 Kbytes
Sector 1    0x08008000 - 0x0800FFFF 32 Kbytes
Sector 2    0x08010000 - 0x08017FFF 32 Kbytes
Sector 3    0x08018000 - 0x0801FFFF 32 Kbytes
Sector 4    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 5    0x08040000 - 0x0807FFFF 256 Kbytes
Sector 6    0x08080000 - 0x080BFFFF 256 Kbytes
Sector 7    0x080C0000 - 0x080FFFFF 256 Kbytes

F7X5XI device with 2M flash
Sector 8    0x08100000 - 0x0813FFFF 256 Kbytes
Sector 9    0x08140000 - 0x0817FFFF 256 Kbytes
Sector 10   0x08180000 - 0x081BFFFF 256 Kbytes
Sector 11   0x081C0000 - 0x081FFFFF 256 Kbytes
*/

static uint32_t getFLASHSectorForEEPROM(void)
{
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_SECTOR_0;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_SECTOR_1;
    if ((uint32_t)&__config_start <= 0x08017FFF)
        return FLASH_SECTOR_2;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_SECTOR_3;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_SECTOR_4;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_SECTOR_5;
    if ((uint32_t)&__config_start <= 0x080BFFFF)
        return FLASH_SECTOR_6;
    if ((uint32_t)&__config_start <= 0x080FFFFF)
        return FLASH_SECTOR_7;
#if defined(STM32F765xx)
    if ((uint32_t)&__config_start <= 0x0813FFFF)
        return FLASH_SECTOR_8;
    if ((uint32_t)&__config_start <= 0x0817FFFF)
        return FLASH_SECTOR_9;
    if ((uint32_t)&__config_start <= 0x081BFFFF)
        return FLASH_SECTOR_10;
    if ((uint32_t)&__config_start <= 0x081FFFFF)
        return FLASH_SECTOR_11;
#endif

    // Not good
    while (1) {
        failureMode(FAILURE_CONFIG_STORE_FAILURE);
    }
}

#elif defined(STM32F722xx)
/*
Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
*/

static uint32_t getFLASHSectorForEEPROM(void)
{
    if ((uint32_t)&__config_start <= 0x08003FFF)
        return FLASH_SECTOR_0;
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_SECTOR_1;
    if ((uint32_t)&__config_start <= 0x0800BFFF)
        return FLASH_SECTOR_2;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_SECTOR_3;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_SECTOR_4;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_SECTOR_5;
    if ((uint32_t)&__config_start <= 0x0805FFFF)
        return FLASH_SECTOR_6;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_SECTOR_7;

    // Not good
    while (1) {
        failureMode(FAILURE_CONFIG_STORE_FAILURE);
    }
}

#elif defined(STM32F4)
/*
Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
Sector 8    0x08080000 - 0x0809FFFF 128 Kbytes
Sector 9    0x080A0000 - 0x080BFFFF 128 Kbytes
Sector 10   0x080C0000 - 0x080DFFFF 128 Kbytes
Sector 11   0x080E0000 - 0x080FFFFF 128 Kbytes
*/

static uint32_t getFLASHSectorForEEPROM(void)
{
    if ((uint32_t)&__config_start <= 0x08003FFF)
        return FLASH_Sector_0;
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_Sector_1;
    if ((uint32_t)&__config_start <= 0x0800BFFF)
        return FLASH_Sector_2;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_Sector_3;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_Sector_4;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_Sector_5;
    if ((uint32_t)&__config_start <= 0x0805FFFF)
        return FLASH_Sector_6;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_Sector_7;
    if ((uint32_t)&__config_start <= 0x0809FFFF)
        return FLASH_Sector_8;
    if ((uint32_t)&__config_start <= 0x080DFFFF)
        return FLASH_Sector_9;
    if ((uint32_t)&__config_start <= 0x080BFFFF)
        return FLASH_Sector_10;
    if ((uint32_t)&__config_start <= 0x080FFFFF)
        return FLASH_Sector_11;

    // Not good
    while (1) {
        failureMode(FAILURE_CONFIG_STORE_FAILURE);
    }
}

#elif defined(STM32H743xx)
/*
There are two banks of 8 of 128K sectors (up to 2MB flash)

Bank 1
Sector 0    0x08000000 - 0x0801FFFF 128 Kbytes
Sector 1    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 2    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 3    0x08060000 - 0x0807FFFF 128 Kbytes
Sector 4    0x08080000 - 0x0809FFFF 128 Kbytes
Sector 5    0x080A0000 - 0x080BFFFF 128 Kbytes
Sector 6    0x080C0000 - 0x080DFFFF 128 Kbytes
Sector 7    0x080E0000 - 0x080FFFFF 128 Kbytes

Bank 2
Sector 0    0x08100000 - 0x0811FFFF 128 Kbytes
Sector 1    0x08120000 - 0x0813FFFF 128 Kbytes
Sector 2    0x08140000 - 0x0815FFFF 128 Kbytes
Sector 3    0x08160000 - 0x0817FFFF 128 Kbytes
Sector 4    0x08180000 - 0x0819FFFF 128 Kbytes
Sector 5    0x081A0000 - 0x081BFFFF 128 Kbytes
Sector 6    0x081C0000 - 0x081DFFFF 128 Kbytes
Sector 7    0x081E0000 - 0x081FFFFF 128 Kbytes

*/

static void getFLASHSectorForEEPROM(uint32_t *bank, uint32_t *sector)
{
    uint32_t start = (uint32_t)&__config_start;

    if (start >= FLASH_BANK1_BASE && start < FLASH_BANK2_BASE) {
        *bank = FLASH_BANK_1;
    } else if (start >= FLASH_BANK2_BASE && start < FLASH_BANK2_BASE + 0x100000) {
        *bank = FLASH_BANK_2;
        start -= 0x100000;
    } else {
        // Not good
        while (1) {
            failureMode(FAILURE_CONFIG_STORE_FAILURE);
        }
    }

    if (start <= 0x0801FFFF)
        *sector = FLASH_SECTOR_0;
    else if (start <= 0x0803FFFF)
        *sector = FLASH_SECTOR_1;
    else if (start <= 0x0805FFFF)
        *sector = FLASH_SECTOR_2;
    else if (start <= 0x0807FFFF)
        *sector = FLASH_SECTOR_3;
    else if (start <= 0x0809FFFF)
        *sector = FLASH_SECTOR_4;
    else if (start <= 0x080BFFFF)
        *sector = FLASH_SECTOR_5;
    else if (start <= 0x080DFFFF)
        *sector = FLASH_SECTOR_6;
    else if (start <= 0x080FFFFF)
        *sector = FLASH_SECTOR_7;
}
#elif defined(STM32H750xx)
/*
The memory map supports 2 banks of 8 128k sectors like the H743xx, but there is only one 128K sector so we save some code
space by using a smaller function.

Bank 1
Sector 0    0x08000000 - 0x0801FFFF 128 Kbytes

*/

static void getFLASHSectorForEEPROM(uint32_t *bank, uint32_t *sector)
{

    uint32_t start = (uint32_t)&__config_start;

    if (start == FLASH_BANK1_BASE) {
        *sector = FLASH_SECTOR_0;
        *bank = FLASH_BANK_1;
    } else {
        // Not good
        while (1) {
            failureMode(FAILURE_CONFIG_STORE_FAILURE);
        }
    }
}
#endif
#endif

// FIXME the return values are currently magic numbers
static int write_word(config_streamer_t *c, config_streamer_buffer_align_type_t *buffer)
{
    if (c->err != 0) {
        return c->err;
    }
#if defined(EEPROM_IN_EXTERNAL_FLASH)

    uint32_t dataOffset = (uint32_t)(c->address - (uintptr_t)&eepromData[0]);

    const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_CONFIG);
    const flashGeometry_t *flashGeometry = flashGetGeometry();

    uint32_t flashStartAddress = flashPartition->startSector * flashGeometry->sectorSize;
    uint32_t flashOverflowAddress = ((flashPartition->endSector + 1) * flashGeometry->sectorSize); // +1 to sector for inclusive

    uint32_t flashAddress = flashStartAddress + dataOffset;
    if (flashAddress + CONFIG_STREAMER_BUFFER_SIZE > flashOverflowAddress) {
        return -3; // address is past end of partition
    }

    uint32_t flashSectorSize = flashGeometry->sectorSize;
    uint32_t flashPageSize = flashGeometry->pageSize;

    bool onPageBoundary = (flashAddress % flashPageSize == 0);
    if (onPageBoundary) {

        bool firstPage = (flashAddress == flashStartAddress);
        if (!firstPage) {
            flashPageProgramFinish();
        }

        if (flashAddress % flashSectorSize == 0) {
            flashEraseSector(flashAddress);
        }

        flashPageProgramBegin(flashAddress);
    }

    flashPageProgramContinue((uint8_t *)buffer, CONFIG_STREAMER_BUFFER_SIZE);

#elif defined(EEPROM_IN_RAM) || defined(EEPROM_IN_SDCARD)
    if (c->address == (uintptr_t)&eepromData[0]) {
        memset(eepromData, 0, sizeof(eepromData));
    }

    uint64_t *dest_addr = (uint64_t *)c->address;
    uint64_t *src_addr = (uint64_t*)buffer;
    uint8_t row_index = 4;
    /* copy the 256 bits flash word */
    do
    {
      *dest_addr++ = *src_addr++;
    } while (--row_index != 0);

#elif defined(EEPROM_IN_FILE)

    if (c->address % FLASH_PAGE_SIZE == 0) {
        const FLASH_Status status = FLASH_ErasePage(c->address);
        if (status != FLASH_COMPLETE) {
            return -1;
        }
    }
    const FLASH_Status status = FLASH_ProgramWord(c->address, *buffer);
    if (status != FLASH_COMPLETE) {
        return -2;
    }

#elif defined(EEPROM_IN_FLASH)

#if defined(STM32H7)
    if (c->address % FLASH_PAGE_SIZE == 0) {
        FLASH_EraseInitTypeDef EraseInitStruct = {
            .TypeErase     = FLASH_TYPEERASE_SECTORS,
            .VoltageRange  = FLASH_VOLTAGE_RANGE_3, // 2.7-3.6V
            .NbSectors     = 1
        };
        getFLASHSectorForEEPROM(&EraseInitStruct.Banks, &EraseInitStruct.Sector);
        uint32_t SECTORError;
        const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
        if (status != HAL_OK) {
            return -1;
        }
    }

    // For H7
    // HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t DataAddress);
    const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, c->address, (uint64_t)(uint32_t)buffer);
    if (status != HAL_OK) {
        return -2;
    }
#elif defined(STM32F7)
    if (c->address % FLASH_PAGE_SIZE == 0) {
        FLASH_EraseInitTypeDef EraseInitStruct = {
            .TypeErase     = FLASH_TYPEERASE_SECTORS,
            .VoltageRange  = FLASH_VOLTAGE_RANGE_3, // 2.7-3.6V
            .NbSectors     = 1
        };
        EraseInitStruct.Sector = getFLASHSectorForEEPROM();
        uint32_t SECTORError;
        const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
        if (status != HAL_OK) {
            return -1;
        }
    }

    // For F7
    // HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
    const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, c->address, (uint64_t)*buffer);
    if (status != HAL_OK) {
        return -2;
    }
#else // !STM32H7 && !STM32F7
    if (c->address % FLASH_PAGE_SIZE == 0) {
#if defined(STM32F4)
        const FLASH_Status status = FLASH_EraseSector(getFLASHSectorForEEPROM(), VoltageRange_3); //0x08080000 to 0x080A0000
#else // STM32F3, STM32F1
        const FLASH_Status status = FLASH_ErasePage(c->address);
#endif
        if (status != FLASH_COMPLETE) {
            return -1;
        }
    }
    const FLASH_Status status = FLASH_ProgramWord(c->address, *buffer);
    if (status != FLASH_COMPLETE) {
        return -2;
    }
#endif
#endif
    c->address += CONFIG_STREAMER_BUFFER_SIZE;
    return 0;
}

int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size)
{
    for (const uint8_t *pat = p; pat != (uint8_t*)p + size; pat++) {
        c->buffer.b[c->at++] = *pat;

        if (c->at == sizeof(c->buffer)) {
            c->err = write_word(c, &c->buffer.w);
            c->at = 0;
        }
    }
    return c->err;
}

int config_streamer_status(config_streamer_t *c)
{
    return c->err;
}

int config_streamer_flush(config_streamer_t *c)
{
    if (c->at != 0) {
        memset(c->buffer.b + c->at, 0, sizeof(c->buffer) - c->at);
        c->err = write_word(c, &c->buffer.w);
        c->at = 0;
    }
    return c-> err;
}

int config_streamer_finish(config_streamer_t *c)
{
    if (c->unlocked) {
#if defined(EEPROM_IN_SDCARD)
        bool saveEEPROMToSDCard(void); // XXX forward declaration to avoid circular dependency between config_streamer / config_eeprom
        saveEEPROMToSDCard();
        // TODO overwrite the data in the file on the SD card.
#elif defined(EEPROM_IN_EXTERNAL_FLASH)
        flashFlush();
#elif defined(EEPROM_IN_RAM)
        // NOP
#elif defined(EEPROM_IN_FILE)
        FLASH_Lock();
#elif defined(EEPROM_IN_FLASH)
#if defined(STM32F7) || defined(STM32H7)
        HAL_FLASH_Lock();
#else
        FLASH_Lock();
#endif
#endif
        c->unlocked = false;
    }
    return c->err;
}
