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

#include "drivers/system.h"

#include "config/config_master.h"

#include "build/build_config.h"

#include "config/config_eeprom.h"

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

    #if defined(STM32F745xx)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x40000)
    #endif

    #if defined(STM32F746xx)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x40000)
    #endif

    #if defined(STM32F722xx)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
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
#define FLASH_PAGE_COUNT 4
#elif defined (STM32F411xE)
#define FLASH_PAGE_COUNT 3
#elif defined (STM32F722xx)
#define FLASH_PAGE_COUNT 3
#elif defined (STM32F745xx)
#define FLASH_PAGE_COUNT 4
#elif defined (STM32F746xx)
#define FLASH_PAGE_COUNT 4
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

    if (strncasecmp(temp->boardIdentifier, TARGET_BOARD_IDENTIFIER, sizeof(TARGET_BOARD_IDENTIFIER)))
        return false;

    // verify integrity of temporary copy
    checksum = calculateChecksum((const uint8_t *) temp, sizeof(master_t));
    if (checksum != 0)
        return false;

    // looks good, let's roll!
    return true;
}

#if defined(STM32F7)

// FIXME: HAL for now this will only work for F4/F7 as flash layout is different
void writeEEPROM(void)
{
    // Generate compile time error if the config does not fit in the reserved area of flash.
    BUILD_BUG_ON(sizeof(master_t) > FLASH_TO_RESERVE_FOR_CONFIG);

    HAL_StatusTypeDef status;
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
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();
    while (attemptsRemaining--)
    {
        /* Fill EraseInit structure*/
        FLASH_EraseInitTypeDef EraseInitStruct = {0};
        EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3; // 2.7-3.6V
        EraseInitStruct.Sector        = (FLASH_SECTOR_TOTAL-1);
        EraseInitStruct.NbSectors     = 1;
        uint32_t SECTORError;
        status = HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
        if (status != HAL_OK)
        {
            continue;
        }
        else
        {
            for (wordOffset = 0; wordOffset < sizeof(master_t); wordOffset += 4)
            {
                status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CONFIG_START_FLASH_ADDRESS + wordOffset, *(uint32_t *) ((char *) &masterConfig + wordOffset));
                if(status != HAL_OK)
                {
                    break;
                }
            }
        }
        if (status == HAL_OK) {
            break;
        }
    }
    HAL_FLASH_Lock();

    // Flash write failed - just die now
    if (status != HAL_OK || !isEEPROMContentValid()) {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
    }

    resumeRxSignal();
}
#else
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
#if defined(STM32F4)
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32F303)
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F10X)
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
#endif

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

    validateAndFixConfig();
    activateConfig();

    resumeRxSignal();
}
