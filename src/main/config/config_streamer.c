/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>

#include "platform.h"

#include "drivers/system.h"

#include "config/config_streamer.h"

extern uint8_t __config_start;   // configured via linker script when building binaries.
extern uint8_t __config_end;

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
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
# elif defined (STM32F411xE)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
# elif defined(STM32F427_437xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x20000) // 128K sectors
// F7
#elif defined(STM32F722xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000) // 16K sectors
# elif defined(STM32F745xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000) // 32K sectors
# elif defined(STM32F746xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000)
# elif defined(UNIT_TEST)
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
    // base must start at FLASH_PAGE_SIZE boundary
    c->address = base;
    c->size = size;
    if (!c->unlocked) {
#if defined(STM32F7)
        HAL_FLASH_Unlock();
#else
        FLASH_Unlock();
#endif
        c->unlocked = true;
    }

#if defined(STM32F10X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#elif defined(STM32F303)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F4)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32F7)
    // NOP
#elif defined(UNIT_TEST)
    // NOP
#else
# error "Unsupported CPU"
#endif
    c->err = 0;
}

#if defined(STM32F745xx) || defined(STM32F746xx)
/*
Sector 0    0x08000000 - 0x08007FFF 32 Kbytes
Sector 1    0x08008000 - 0x0800FFFF 32 Kbytes
Sector 2    0x08010000 - 0x08017FFF 32 Kbytes
Sector 3    0x08018000 - 0x0801FFFF 32 Kbytes
Sector 4    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 5    0x08040000 - 0x0807FFFF 256 Kbytes
Sector 6    0x08080000 - 0x080BFFFF 256 Kbytes
Sector 7    0x080C0000 - 0x080FFFFF 256 Kbytes
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

    // Not good
    while (1) {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
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
        failureMode(FAILURE_FLASH_WRITE_FAILED);
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
        failureMode(FAILURE_FLASH_WRITE_FAILED);
    }
}
#endif

static int write_word(config_streamer_t *c, uint32_t value)
{
    if (c->err != 0) {
        return c->err;
    }
#if defined(STM32F7)
    if (c->address % FLASH_PAGE_SIZE == 0) {
        FLASH_EraseInitTypeDef EraseInitStruct = {
            .TypeErase     = FLASH_TYPEERASE_SECTORS,
            .VoltageRange  = FLASH_VOLTAGE_RANGE_3, // 2.7-3.6V
            .NbSectors     = 1
        };
        EraseInitStruct.Sector = getFLASHSectorForEEPROM();
        uint32_t SECTORError;
        const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
        if (status != HAL_OK){
            return -1;
        }
    }
    const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, c->address, value);
    if (status != HAL_OK) {
        return -2;
    }
#else
    if (c->address % FLASH_PAGE_SIZE == 0) {
#if defined(STM32F4)
        const FLASH_Status status = FLASH_EraseSector(getFLASHSectorForEEPROM(), VoltageRange_3); //0x08080000 to 0x080A0000
#else
        const FLASH_Status status = FLASH_ErasePage(c->address);
#endif
        if (status != FLASH_COMPLETE) {
            return -1;
        }
    }
    const FLASH_Status status = FLASH_ProgramWord(c->address, value);
    if (status != FLASH_COMPLETE) {
        return -2;
    }
#endif
    c->address += sizeof(value);
    return 0;
}

int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size)
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

int config_streamer_status(config_streamer_t *c)
{
    return c->err;
}

int config_streamer_flush(config_streamer_t *c)
{
    if (c->at != 0) {
        memset(c->buffer.b + c->at, 0, sizeof(c->buffer) - c->at);
        c->err = write_word(c, c->buffer.w);
        c->at = 0;
    }
    return c-> err;
}

int config_streamer_finish(config_streamer_t *c)
{
    if (c->unlocked) {
#if defined(STM32F7)
        HAL_FLASH_Lock();
#else
        FLASH_Lock();
#endif
        c->unlocked = false;
    }
    return c->err;
}
