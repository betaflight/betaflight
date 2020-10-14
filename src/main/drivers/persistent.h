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
#ifdef USE_SPRACING_PERSISTENT_RTC_WORKAROUND
    // SPRACING H7 firmware always leaves this value reset so only use this location to invoke DFU
    PERSISTENT_OBJECT_RESET_REASON_FWONLY,
#else
    PERSISTENT_OBJECT_RESET_REASON,
#endif
    PERSISTENT_OBJECT_RTC_HIGH,           // high 32 bits of rtcTime_t
    PERSISTENT_OBJECT_RTC_LOW,            // low 32 bits of rtcTime_t
    PERSISTENT_OBJECT_COUNT,
#ifdef USE_SPRACING_PERSISTENT_RTC_WORKAROUND
    // On SPRACING H7 firmware use this alternate location for all reset reasons interpreted by this firmware
    PERSISTENT_OBJECT_RESET_REASON,
#endif
} persistentObjectId_e;

// Values for PERSISTENT_OBJECT_RESET_REASON
#define RESET_NONE                      0
#define RESET_BOOTLOADER_REQUEST_ROM    1  // Boot loader invocation was requested
#define RESET_BOOTLOADER_POST           2  // Reset after boot loader activity
#define RESET_MSC_REQUEST               3  // MSC invocation was requested
#define RESET_FORCED                    4  // Reset due to unknown reset reason
#define RESET_BOOTLOADER_REQUEST_FLASH  5

void persistentObjectInit(void);
uint32_t persistentObjectRead(persistentObjectId_e id);
void persistentObjectWrite(persistentObjectId_e id, uint32_t value);
