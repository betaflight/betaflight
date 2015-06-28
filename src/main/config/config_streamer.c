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

#if defined(FLASH_PAGE_SIZE)
#elif defined(STM32F303xC)
#define FLASH_PAGE_SIZE                 (0x800)
#elif defined(STM32F10X_MD)
#define FLASH_PAGE_SIZE                 (0x400)
#elif defined(STM32F10X_HD)
#define FLASH_PAGE_SIZE                 (0x800)
#elif defined(UNIT_TEST)
#define FLASH_PAGE_SIZE                 (0x400)
#else
#error "Flash page size not defined for target."
#endif

void config_streamer_init(config_streamer_t *c)
{
    memset(c, 0, sizeof(*c));
}

void config_streamer_start(config_streamer_t *c, uintptr_t base)
{
    c->address = base;

    if (!c->unlocked) {
        FLASH_Unlock();
        c->unlocked = true;
    }

#if defined(STM32F303)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F10X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#elif defined(UNIT_TEST)
#else
    #error
#endif
    c->err = 0;
}

int write_word(config_streamer_t *c, uint32_t value)
{
    if (c->err != 0) {
        return c->err;
    }

    FLASH_Status status;

    if (c->address % FLASH_PAGE_SIZE == 0) {
        status = FLASH_ErasePage(c->address);
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

int config_streamer_write(config_streamer_t *c, const void *p, uint32_t size)
{
    const uint8_t *pat = p;
    const uint8_t *pend = pat + size;

    for (; pat != pend; pat++) {
        c->buffer.b[c->at++] = *pat;
        c->chk ^= *pat;

        if (c->at == sizeof(c->buffer)) {
            c->err = write_word(c, c->buffer.w);
            c->at = 0;
        }
    }
    return c->err;
}

uint8_t config_streamer_chk(config_streamer_t *c)
{
    return c->chk;
}

int config_streamer_finish(config_streamer_t *c)
{
    if (c->at != 0) {
        // Flush what's left.  Don't worry about the garbage as it
        // shouldn't be read.
        c->err = write_word(c, c->buffer.w);
        c->at = 0;
    }
    if (c->unlocked) {
        FLASH_Lock();
        c->unlocked = false;
    }
    return c->err;
}
