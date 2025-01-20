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
    RTC_HandleTypeDef rtcHandle = { .Instance = RTC };

    uint32_t value = DAL_RTCEx_BKUPRead(&rtcHandle, id);

    return value;
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    RTC_HandleTypeDef rtcHandle = { .Instance = RTC };

    DAL_RTCEx_BKUPWrite(&rtcHandle, id, value);

#ifdef USE_SPRACING_PERSISTENT_RTC_WORKAROUND
    // Also write the persistent location used by the bootloader to support DFU etc.
    if (id == PERSISTENT_OBJECT_RESET_REASON) {
        // SPRACING firmware sometimes enters DFU mode when MSC mode is requested
        if (value == RESET_MSC_REQUEST) {
            value = RESET_NONE;
        }
        DAL_RTCEx_BKUPWrite(&rtcHandle, PERSISTENT_OBJECT_RESET_REASON_FWONLY, value);
    }
#endif
}

static void persistentObjectRTCEnable(void)
{
    RTC_HandleTypeDef rtcHandle = { .Instance = RTC };

    __DAL_RCM_PMU_CLK_ENABLE(); // Enable Access to PMU

    DAL_PMU_EnableBkUpAccess(); // Disable backup domain protection

    // We don't need a clock source for RTC itself. Skip it.

    __DAL_RTC_WRITEPROTECTION_ENABLE(&rtcHandle);  // Reset sequence
    __DAL_RTC_WRITEPROTECTION_DISABLE(&rtcHandle); // Apply sequence
}

void persistentObjectInit(void)
{
    // Configure and enable RTC for backup register access

    persistentObjectRTCEnable();

    // XXX Magic value checking may be sufficient

    uint32_t wasSoftReset;

    wasSoftReset = RCM->CSTS & RCM_CSTS_SWRSTFLG;

    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
        for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
    }
}
