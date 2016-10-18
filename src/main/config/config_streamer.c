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

#include "config_streamer.h"

#include "platform.h"

#include <string.h>

#if !defined(FLASH_PAGE_SIZE)
# if defined(STM32F303xC)
#  define FLASH_PAGE_SIZE                 (0x800)
# elif defined(STM32F10X_MD)
#  define FLASH_PAGE_SIZE                 (0x400)
# elif defined(STM32F10X_HD)
#  define FLASH_PAGE_SIZE                 (0x800)
<<<<<<< 5f03cdcdb9f4724480a48877425d8e841408c772
# elif defined(STM32F40_41xxx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
# elif defined (STM32F411xE)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
# elif defined(STM32F745xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x40000)
=======
>>>>>>> Initial commit
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

#if defined(STM32F4)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32F303)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F10X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#elif defined(STM32F7)
    // NOP
#elif defined(UNIT_TEST)
    // NOP
#else
# error "Unsupported CPU"
#endif
    c->err = 0;
}

#if defined(STM32F7)
static int write_word(config_streamer_t *c, uint32_t value)
{
    if (c->err != 0) {
        return c->err;
    }

    HAL_StatusTypeDef status;
    if (c->address % FLASH_PAGE_SIZE == 0) {
        FLASH_EraseInitTypeDef EraseInitStruct = {0};
        EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3; // 2.7-3.6V
        EraseInitStruct.Sector        = (FLASH_SECTOR_TOTAL-1);
        EraseInitStruct.NbSectors     = 1;
        uint32_t SECTORError;
        status = HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
        if (status != HAL_OK){
            return -1;
        }
    }
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, c->address, value);
    if (status != HAL_OK) {
        return -2;
    }
    c->address += sizeof(value);
    return 0;
}
#else
static int write_word(config_streamer_t *c, uint32_t value)
{
    if (c->err != 0) {
        return c->err;
    }

    FLASH_Status status;

    if (c->address % FLASH_PAGE_SIZE == 0) {
#if defined(STM32F40_41xxx)
        status = FLASH_EraseSector(FLASH_Sector_8, VoltageRange_3); //0x08080000 to 0x080A0000
#elif defined (STM32F411xE)
        status = FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3); //0x08060000 to 0x08080000
#else
        status = FLASH_ErasePage(c->address);
#endif
        if (status != FLASH_COMPLETE) {
            return -1;
        }
    }
    status = FLASH_ProgramWord(c->address, value);
    if (status != FLASH_COMPLETE) {
        return -2;
    }
    c->address += sizeof(value);
    return 0;
}
#endif

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
