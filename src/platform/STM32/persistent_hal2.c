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
 * Persistent data storage via TAMP backup registers for HAL2-based STM32 families.
 * HAL2 moves backup register access from RTC to TAMP peripheral.
 */

#include <stdint.h>
#include "platform.h"

#include "drivers/persistent.h"
#include "drivers/system.h"

#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24)|('e' << 16)|('f' << 8)|('1' << 0))

uint32_t persistentObjectRead(persistentObjectId_e id)
{
    return HAL_TAMP_ReadBackupRegisterValue((hal_tamp_backup_register_idx_t)id);
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    HAL_TAMP_WriteBackupRegisterValue((hal_tamp_backup_register_idx_t)id, value);
}

void persistentObjectRTCEnable(void)
{
    // HAL2: TAMP backup registers are accessible without explicit
    // RTC/PWR clock enable or write-protection dance on C5.
    // The TAMP peripheral clock is enabled by default after reset.
}

void persistentObjectInit(void)
{
    persistentObjectRTCEnable();

    uint32_t wasSoftReset = RCC->RSR & RCC_RSR_SFTRSTF;

    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
        for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
    }
}
