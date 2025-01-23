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
    uint32_t value = ertc_bpr_data_read((ertc_dt_type)id);
    return value;
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    ertc_write_protect_disable();
    ertc_bpr_data_write((ertc_dt_type)id, value);
    ertc_write_protect_enable();
}

void persistentObjectRTCEnable(void)
{
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_battery_powered_domain_access(TRUE);
    ertc_write_protect_enable();
    ertc_write_protect_disable();
}

void persistentObjectInit(void)
{
    persistentObjectRTCEnable();
    uint32_t wasSoftReset;

    wasSoftReset = crm_flag_get(CRM_SW_RESET_FLAG);

    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
        for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
    }
}
