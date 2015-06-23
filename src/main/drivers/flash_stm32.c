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

#include "platform.h"
#include "drivers/flash_stm32.h"

#include <string.h>

#if defined(FLASH_PAGE_SIZE)
#elif defined(STM32F303xC)
#define FLASH_PAGE_SIZE                 (0x800)
#elif defined(STM32F10X_MD)
#define FLASH_PAGE_SIZE                 (0x400)
#elif defined(STM32F10X_HD)
#define FLASH_PAGE_SIZE                 (0x800)
#else
#error "Flash page size not defined for target."
#endif

void flash_stm32_init(flash_stm32_writer_t *f)
{
    memset(f, 0, sizeof(*f));
}

int flash_stm32_start(flash_stm32_writer_t *f, uintptr_t base)
{
    if (!f->unlocked) {
        FLASH_Unlock();
        f->unlocked = true;
    }

#if defined(STM32F303)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F10X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#else
    #error
#endif
    f->address = base;
    f->err = 0;

    return f->err;
}

int flash_stm32_write(flash_stm32_writer_t *f, const void *p, uint32_t size)
{
    if (f->err != 0) {
        return f->err;
    }

    uint32_t *at = (uint32_t *)p;

    for (uint32_t offset = 0; offset < size; offset += sizeof(uint32_t)) {
        uint32_t address = f->address + offset;
        FLASH_Status status;
        
        if (address % FLASH_PAGE_SIZE == 0) {
            status = FLASH_ErasePage(address);
            if (status != FLASH_COMPLETE) {
                f->err = -1;
                break;
            }
        }
        status = FLASH_ProgramWord(address, *at++);
        if (status != FLASH_COMPLETE) {
            f->err = -2;
            break;
        }
    }
    f->address += size;

    return f->err;
}

int flash_stm32_finish(flash_stm32_writer_t *f)
{
    if (f->unlocked) {
        FLASH_Lock();
        f->unlocked = false;
    }
    return f->err;
}
