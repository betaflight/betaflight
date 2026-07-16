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
 * Persistent data storage utilizing no-init SRAM section.
 * Retains values across software resets but not power loss.
 */

#include <stdint.h>
#include <string.h>
#include "platform.h"

#include "drivers/persistent.h"

#define PERSIST_NOINIT __attribute__((section(".noinit")))

#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24)|('e' << 16)|('f' << 8)|('1' << 0))

typedef struct {
    uint32_t values[PERSISTENT_OBJECT_COUNT];
} esp32PersistentStore_t;

static PERSIST_NOINIT esp32PersistentStore_t esp32PersistentStore;

void persistentObjectValidate(void)
{
    if (esp32PersistentStore.values[PERSISTENT_OBJECT_MAGIC] != PERSISTENT_OBJECT_MAGIC_VALUE) {
        memset(&esp32PersistentStore, 0, sizeof(esp32PersistentStore));
        esp32PersistentStore.values[PERSISTENT_OBJECT_MAGIC] = PERSISTENT_OBJECT_MAGIC_VALUE;
    }
}

uint32_t persistentObjectRead(persistentObjectId_e id)
{
    persistentObjectValidate();

    if ((unsigned)id < PERSISTENT_OBJECT_COUNT) {
        return esp32PersistentStore.values[id];
    }

    return 0;
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    persistentObjectValidate();

    if ((unsigned)id < PERSISTENT_OBJECT_COUNT) {
        esp32PersistentStore.values[id] = value;
    }
}
