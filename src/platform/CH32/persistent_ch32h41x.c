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
 * An implementation of persistent data storage utilizing reserved SRAM.
 * CH32H41x has no RTC backup data registers used here; the reserved 1 KB tail
 * of SRAM retains values across software resets and bootloader handoffs.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24) | ('e' << 16) | ('f' << 8) | ('1' << 0))
#define BKP_DATA_ADDR                 0x2017FC00U
#define BKP_DATA_SIZE_BYTES           1024U

STATIC_ASSERT(PERSISTENT_OBJECT_COUNT * sizeof(uint32_t) <= BKP_DATA_SIZE_BYTES, persistent_storage_overflow);

static uint32_t simBkpDataRead(uint32_t id)
{
    return *(volatile uint32_t *)(BKP_DATA_ADDR + id * sizeof(uint32_t));
}

static void simBkpDataWrite(uint32_t id, uint32_t value)
{
    *(volatile uint32_t *)(BKP_DATA_ADDR + id * sizeof(uint32_t)) = value;
    __asm__ volatile ("fence rw, rw" ::: "memory");
}

static bool isSoftResetFlagSet(void)
{
    return ((RCC->RSTSCKR & RCC_SFTRSTF) == RCC_SFTRSTF);
}

uint32_t persistentObjectRead(persistentObjectId_e id)
{
    if ((uint32_t)id >= PERSISTENT_OBJECT_COUNT) {
        return 0;
    }
    return simBkpDataRead((uint32_t)id);
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    if ((uint32_t)id >= PERSISTENT_OBJECT_COUNT) {
        return;
    }
    simBkpDataWrite((uint32_t)id, value);
}

void persistentObjectInit(void)
{
    const bool wasSoftReset = isSoftResetFlagSet();

    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
        for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
    }
}
