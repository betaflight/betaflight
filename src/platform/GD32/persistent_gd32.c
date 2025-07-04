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
 * An implementation of persistent data storage utilizing RTC backup data register.
 * Retains values written across software resets and boot loader activities.
 */

#include <stdint.h>
#include "platform.h"

#include "drivers/persistent.h"
#include "drivers/system.h"

#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24)|('e' << 16)|('f' << 8)|('1' << 0))

uint32_t persistentObjectRead(persistentObjectId_e id)
{
    uint32_t value = REG32((RTC) + 0x50U + (id * 4U));

    return value;
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    REG32(RTC + 0x50U + (id * 4U)) = value;

    RTC_WPK = RTC_LOCK_KEY;
}

void persistentObjectRTCEnable(void)
{
    /* Enable access to the backup domain */
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();
    rcu_periph_clock_enable(RCU_RTC); 

    rtc_register_sync_wait();

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;
}

void persistentObjectInit(void)
{
    // Enable RTC and backup register access
    persistentObjectRTCEnable();

    // Check if the system was a software reset
    uint32_t wasSoftReset = RCU_RSTSCK & RCU_RSTSCK_SWRSTF;

    // Check if the magic value is present in the backup register
    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
        // Clear all persistent objects
        for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
        // Write the magic value to indicate valid data
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
    }
}
