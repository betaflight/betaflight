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
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/persistent.h"

#include "platform/rcc.h"


#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24) | ('e' << 16) | ('f' << 8) | ('1' << 0))

static bool persistentObjectIsValidId(persistentObjectId_e id)
{
    return (unsigned)id < PERSISTENT_OBJECT_COUNT;
}

static uint8_t persistentObjectToBackupRegister(persistentObjectId_e id)
{
    return (uint8_t)id + 1;
}

uint32_t persistentObjectRead(persistentObjectId_e id)
{
    if (!persistentObjectIsValidId(id)) {
        return 0;
    }

    return RTC_BKUPRgRead(persistentObjectToBackupRegister(id));
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    if (!persistentObjectIsValidId(id)) {
        return;
    }

    RTC_BKUPRgWrite(persistentObjectToBackupRegister(id), value);
}

static void persistentObjectRTCEnable(void)
{
    RCC_ClockCmd(RCC_APB5_2(RTCPCLK), ENABLE);
    PWR_BackupAccessEnable(ENABLE);
}

void persistentObjectInit(void)
{
    persistentObjectRTCEnable();

    const bool wasSoftReset = RCC_GetFlagStatus(RCC_FLAG_CM7SFTRST) == SET;

    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
        for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
    }
}
