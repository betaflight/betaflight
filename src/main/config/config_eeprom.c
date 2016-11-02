/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/color.h"
#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/motors.h"
#include "io/servos.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "io/ledstrip.h"
#include "io/gps.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/failsafe.h"
#include "flight/navigation_rewrite.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#if !defined(FLASH_SIZE)
#error "Flash size not defined for target. (specify in KB)"
#endif


#ifndef FLASH_PAGE_SIZE
    #ifdef STM32F303xC
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif

    #ifdef STM32F10X_MD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif

    #if defined(STM32F40_41xxx)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
    #endif

    #if defined (STM32F411xE)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
    #endif

#endif

#if !defined(FLASH_SIZE) && !defined(FLASH_PAGE_COUNT)
    #ifdef STM32F10X_MD
        #define FLASH_PAGE_COUNT 128
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_COUNT 128
    #endif
#endif

#if defined(FLASH_SIZE)
#if defined(STM32F40_41xxx)
#define FLASH_PAGE_COUNT 4 // just to make calculations work
#elif defined (STM32F411xE)
#define FLASH_PAGE_COUNT 4 // just to make calculations work
#else
#define FLASH_PAGE_COUNT ((FLASH_SIZE * 0x400) / FLASH_PAGE_SIZE)
#endif
#endif

#if !defined(FLASH_PAGE_SIZE)
#error "Flash page size not defined for target."
#endif

#if !defined(FLASH_PAGE_COUNT)
#error "Flash page count not defined for target."
#endif

#if FLASH_SIZE <= 128
#define FLASH_TO_RESERVE_FOR_CONFIG 0x800
#else
#define FLASH_TO_RESERVE_FOR_CONFIG 0x1000
#endif

// use the last flash pages for storage
#ifdef CUSTOM_FLASH_MEMORY_ADDRESS
size_t custom_flash_memory_address = 0;
#define CONFIG_START_FLASH_ADDRESS (custom_flash_memory_address)
#else
// use the last flash pages for storage
#ifndef CONFIG_START_FLASH_ADDRESS 
#define CONFIG_START_FLASH_ADDRESS (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_TO_RESERVE_FOR_CONFIG))
#endif
#endif

void initEEPROM(void)
{
}

static uint8_t calculateChecksum(const uint8_t *data, uint32_t length)
{
    uint8_t checksum = 0;
    const uint8_t *byteOffset;

    for (byteOffset = data; byteOffset < (data + length); byteOffset++)
        checksum ^= *byteOffset;
    return checksum;
}

bool isEEPROMContentValid(void)
{
    const master_t *temp = (const master_t *) CONFIG_START_FLASH_ADDRESS;
    uint8_t checksum = 0;

    // check version number
    if (EEPROM_CONF_VERSION != temp->version)
        return false;

    // check size and magic numbers
    if (temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return false;

    // verify integrity of temporary copy
    checksum = calculateChecksum((const uint8_t *) temp, sizeof(master_t));
    if (checksum != 0)
        return false;

    // looks good, let's roll!
    return true;
}

void writeEEPROM(void)
{
    // Generate compile time error if the config does not fit in the reserved area of flash.
    BUILD_BUG_ON(sizeof(master_t) > FLASH_TO_RESERVE_FOR_CONFIG);

    FLASH_Status status = 0;
    uint32_t wordOffset;
    int8_t attemptsRemaining = 3;

    suspendRxSignal();

    // prepare checksum/version constants
    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.size = sizeof(master_t);
    masterConfig.magic_be = 0xBE;
    masterConfig.magic_ef = 0xEF;
    masterConfig.chk = 0; // erase checksum before recalculating
    masterConfig.chk = calculateChecksum((const uint8_t *) &masterConfig, sizeof(master_t));

    // write it
    FLASH_Unlock();
    while (attemptsRemaining--) {
#ifdef STM32F40_41xxx
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#endif
#ifdef STM32F303
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#endif
#ifdef STM32F10X
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#endif
        for (wordOffset = 0; wordOffset < sizeof(master_t); wordOffset += 4) {
            if (wordOffset % FLASH_PAGE_SIZE == 0) {
#if defined(STM32F40_41xxx)
                status = FLASH_EraseSector(FLASH_Sector_8, VoltageRange_3); //0x08080000 to 0x080A0000
#elif defined (STM32F411xE)
                status = FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3); //0x08060000 to 0x08080000
#else
                status = FLASH_ErasePage(CONFIG_START_FLASH_ADDRESS + wordOffset);
#endif
                if (status != FLASH_COMPLETE) {
                    break;
                }
            }

            status = FLASH_ProgramWord(CONFIG_START_FLASH_ADDRESS + wordOffset,
                    *(uint32_t *) ((char *) &masterConfig + wordOffset));
            if (status != FLASH_COMPLETE) {
                break;
            }
        }
        if (status == FLASH_COMPLETE) {
            break;
        }
    }
    FLASH_Lock();

    // Flash write failed - just die now
    if (status != FLASH_COMPLETE || !isEEPROMContentValid()) {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
    }

    resumeRxSignal();
}

void readEEPROM(void)
{
    // Sanity check
    if (!isEEPROMContentValid())
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);

    suspendRxSignal();

    // Read flash
    memcpy(&masterConfig, (char *) CONFIG_START_FLASH_ADDRESS, sizeof(master_t));

    if (masterConfig.current_profile_index > MAX_PROFILE_COUNT - 1) // sanity check
        masterConfig.current_profile_index = 0;

    setProfile(masterConfig.current_profile_index);

    if (currentProfile->defaultRateProfileIndex > MAX_CONTROL_RATE_PROFILE_COUNT - 1) // sanity check
        currentProfile->defaultRateProfileIndex = 0;

    setControlRateProfile(currentProfile->defaultRateProfileIndex);

    validateAndFixConfig();
    activateConfig();

    resumeRxSignal();
}
