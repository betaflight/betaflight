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
 * Retains values written across software resets and boot loader activities.
 */

#include <stdint.h>
#include "platform.h"

#include "bgpr_hpmicro.h"
#include "drivers/persistent.h"
#include "drivers/system.h"
#include "hpm_bgpr_drv.h"

#define PERSISTENT_OBJECT_MAGIC_VALUE ('B' << 24 | 'e' << 16 | 'f' << 8 | '1' << 0)

uint32_t persistentObjectRead(persistentObjectId_e id)
{
    /* BGPR word 0 belongs to the boot ROM DFU trigger; persistent objects
     * start at BGPR_PERSISTENT_WORD_OFFSET (see bgpr_hpmicro.h). */
    uint32_t value = 0;
    (void) bgpr_read32(HPM_BGPR, BGPR_PERSISTENT_WORD_OFFSET + id, &value);
    return value;
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    (void) bgpr_write32(HPM_BGPR, BGPR_PERSISTENT_WORD_OFFSET + id, value);
}

void persistentObjectInit(void)
{

    uint32_t wasSoftReset;

    // Use the startup snapshot because RESET_FLAG is W1C and may be cleared
    // after systemInit() captures the reset cause.
    wasSoftReset = isMPUSoftReset();

    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
        for (int i = 0; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
    }
}
