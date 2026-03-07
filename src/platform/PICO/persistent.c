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

/*
 * An implementation of persistent data storage utilizing RTC backup data register.
 * Retains values written across software resets and some boot loader activities.
 * - won't retain if FLASH is wiped.
 */

#include <stdint.h>
#include <string.h>
#include "platform.h"

#include "drivers/persistent.h"

// Use a no-init SRAM section so contents survive soft resets (but not power loss)
#define PERSIST_NOINIT __attribute__((section(".uninitialized_data.persistent")))

#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24)|('e' << 16)|('f' << 8)|('1' << 0))

typedef struct {
    uint32_t values[PERSISTENT_OBJECT_COUNT];
} picoPersistentStore_t;

static PERSIST_NOINIT picoPersistentStore_t picoPersistentStore;

void persistentObjectValidate(void)
{
    // Ensure the persistent store is valid
    if (picoPersistentStore.values[PERSISTENT_OBJECT_MAGIC] != PERSISTENT_OBJECT_MAGIC_VALUE) {
        memset(&picoPersistentStore, 0, sizeof(picoPersistentStore));
        picoPersistentStore.values[PERSISTENT_OBJECT_MAGIC] = PERSISTENT_OBJECT_MAGIC_VALUE;
    }
}
uint32_t persistentObjectRead(persistentObjectId_e id)
{
    persistentObjectValidate();

    if ((unsigned)id < PERSISTENT_OBJECT_COUNT) {
        return picoPersistentStore.values[id];
    }

    return 0;
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    persistentObjectValidate();

    if ((unsigned)id < PERSISTENT_OBJECT_COUNT) {
        picoPersistentStore.values[id] = value;
    }
}
