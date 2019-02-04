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

#pragma once

// Available RTC backup registers (4-byte words) per MCU type
// F4: 20 words
// F7: 32 words
// H7: 32 words

typedef enum {
    PERSISTENT_OBJECT_MAGIC = 0,
    PERSISTENT_OBJECT_HSE_VALUE,
    PERSISTENT_OBJECT_OVERCLOCK_LEVEL,
    PERSISTENT_OBJECT_BOOTMODE_REQUEST,
    PERSISTENT_OBJECT_RTC_HIGH,           // high 32 bits of rtcTime_t
    PERSISTENT_OBJECT_RTC_LOW,            // low 32 bits of rtcTime_t
    PERSISTENT_OBJECT_COUNT,
} persistentObjectId_e;

void persistentObjectInit(void);
uint32_t persistentObjectRead(persistentObjectId_e id);
void persistentObjectWrite(persistentObjectId_e id, uint32_t value);
